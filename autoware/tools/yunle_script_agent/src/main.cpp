#include <boost/asio.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <nlohmann/json.hpp>

#include <fcntl.h>
#include <signal.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

namespace asio = boost::asio;
namespace beast = boost::beast;
namespace websocket = beast::websocket;
using tcp = asio::ip::tcp;
using json = nlohmann::json;

namespace {

std::atomic<bool> g_stop{false};

struct Options {
  std::string listen{"10.66.0.2"};
  uint16_t port{8766};
  std::string token;
  bool self_test{false};
};

struct ScriptDefinition {
  std::string id;
  std::string name;
  std::string description;
  std::string path;
  std::string exclusive_group;
  bool one_shot{false};
};

const std::vector<ScriptDefinition> &scriptDefinitions() {
  static const std::vector<ScriptDefinition> definitions{
    {"autoware", "启动 Autoware",
      "启动定位、规划、感知和车辆接口",
      "/home/tang/autoware/scripts/web_start_autoware.sh", "", false},
    {"rosbridge", "连接工控机",
      "启动 rosbridge WebSocket 服务（端口 9090）",
      "/home/tang/autoware/scripts/web_start_rosbridge.sh", "", false},
    {"camera", "启动相机",
      "启动相机视频推流，等待 Autoware 相机图像就绪",
      "/home/tang/autoware/scripts/web_start_camera.sh", "", false},
  };
  return definitions;
}

struct Runtime {
  pid_t pid{-1};
  std::string state{"idle"};
  int exit_code{0};
  std::string message;
  std::string log_path;
};

class ScriptManager {
 public:
  ScriptManager() {
    std::filesystem::create_directories(log_dir_);
    for (const auto &definition : scriptDefinitions()) {
      Runtime runtime;
      runtime.log_path = log_dir_ + "/" + definition.id + ".log";
      runtimes_.emplace(definition.id, std::move(runtime));
    }
  }

  ~ScriptManager() {
    std::lock_guard<std::mutex> lock(mutex_);
    refreshLocked();
    for (auto &[id, runtime] : runtimes_) {
      (void)id;
      if (runtime.pid > 0) ::kill(-runtime.pid, SIGTERM);
    }
  }

  json list() {
    std::lock_guard<std::mutex> lock(mutex_);
    refreshLocked();
    json scripts = json::array();
    for (const auto &definition : scriptDefinitions()) {
      const auto &runtime = runtimes_.at(definition.id);
      scripts.push_back(toJson(definition, runtime));
    }
    return {{"type", "script_list"}, {"scripts", std::move(scripts)}};
  }

  json start(const std::string &id) {
    std::lock_guard<std::mutex> lock(mutex_);
    refreshLocked();
    const ScriptDefinition *definition = findDefinition(id);
    if (!definition) return error("unknown_script", "脚本不在白名单中");
    Runtime &runtime = runtimes_.at(id);
    if (runtime.pid > 0) return error("already_running", "脚本已经在运行");
    if (!definition->exclusive_group.empty()) {
      for (const auto &other : scriptDefinitions()) {
        if (other.id != id && other.exclusive_group == definition->exclusive_group &&
            runtimes_.at(other.id).pid > 0) {
          return error("resource_busy", other.name + " 正在占用同一相机资源");
        }
      }
    }
    if (::access(definition->path.c_str(), X_OK) != 0) {
      runtime.state = "failed";
      runtime.message = "脚本不存在或不可执行";
      return error("not_executable", runtime.message);
    }

    const pid_t pid = ::fork();
    if (pid < 0) return error("fork_failed", "无法创建脚本进程");
    if (pid == 0) {
      ::setpgid(0, 0);
      const int log_fd = ::open(runtime.log_path.c_str(), O_CREAT | O_WRONLY | O_TRUNC, 0600);
      if (log_fd >= 0) {
        ::dup2(log_fd, STDOUT_FILENO);
        ::dup2(log_fd, STDERR_FILENO);
        if (log_fd > STDERR_FILENO) ::close(log_fd);
      }
      ::chdir("/home/tang/autoware");
      ::setenv("HOME", "/home/tang", 1);
      ::setenv("USER", "tang", 1);
      ::setenv("LOGNAME", "tang", 1);
      ::setenv("SHELL", "/bin/bash", 1);
      ::setenv("PATH", "/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin", 1);
      ::setenv("DISPLAY", ":0", 0);
      ::setenv("XDG_RUNTIME_DIR", "/run/user/1000", 0);
      ::setenv("DBUS_SESSION_BUS_ADDRESS", "unix:path=/run/user/1000/bus", 0);
      ::execl(definition->path.c_str(), definition->path.c_str(), static_cast<char *>(nullptr));
      const char msg[] = "exec failed\n";
      (void)::write(STDERR_FILENO, msg, sizeof(msg) - 1);
      _exit(127);
    }

    ::setpgid(pid, pid);
    runtime.pid = pid;
    runtime.state = "running";
    runtime.exit_code = 0;
    runtime.message = "已启动";
    return {{"type", "script_action"}, {"ok", true}, {"action", "start"},
      {"script", toJson(*definition, runtime)}};
  }

  json stop(const std::string &id) {
    std::lock_guard<std::mutex> lock(mutex_);
    refreshLocked();
    const ScriptDefinition *definition = findDefinition(id);
    if (!definition) return error("unknown_script", "脚本不在白名单中");
    Runtime &runtime = runtimes_.at(id);
    if (runtime.pid <= 0) return error("not_running", "脚本当前未运行");
    if (::kill(-runtime.pid, SIGTERM) != 0) return error("stop_failed", "无法停止脚本进程");
    runtime.state = "stopping";
    runtime.message = "正在停止";
    return {{"type", "script_action"}, {"ok", true}, {"action", "stop"},
      {"script", toJson(*definition, runtime)}};
  }

  json log(const std::string &id) {
    std::lock_guard<std::mutex> lock(mutex_);
    refreshLocked();
    const ScriptDefinition *definition = findDefinition(id);
    if (!definition) return error("unknown_script", "脚本不在白名单中");
    const Runtime &runtime = runtimes_.at(id);
    return {{"type", "script_log"}, {"id", id}, {"name", definition->name},
      {"log", readTail(runtime.log_path, 12000)}};
  }

 private:
  static json error(const std::string &code, const std::string &message) {
    return {{"type", "script_action"}, {"ok", false}, {"code", code}, {"message", message}};
  }

  static const ScriptDefinition *findDefinition(const std::string &id) {
    const auto &definitions = scriptDefinitions();
    const auto it = std::find_if(definitions.begin(), definitions.end(),
      [&](const ScriptDefinition &definition) { return definition.id == id; });
    return it == definitions.end() ? nullptr : &*it;
  }

  static json toJson(const ScriptDefinition &definition, const Runtime &runtime) {
    return {{"id", definition.id}, {"name", definition.name},
      {"description", definition.description}, {"path", definition.path},
      {"one_shot", definition.one_shot}, {"state", runtime.state},
      {"pid", runtime.pid > 0 ? runtime.pid : 0}, {"exit_code", runtime.exit_code},
      {"message", runtime.message}};
  }

  void refreshLocked() {
    for (auto &[id, runtime] : runtimes_) {
      (void)id;
      if (runtime.pid <= 0) continue;
      int status = 0;
      const pid_t result = ::waitpid(runtime.pid, &status, WNOHANG);
      if (result == 0) continue;
      if (result < 0) {
        runtime.pid = -1;
        runtime.state = "failed";
        runtime.message = "无法读取进程状态";
        continue;
      }
      runtime.pid = -1;
      if (WIFEXITED(status)) {
        runtime.exit_code = WEXITSTATUS(status);
        runtime.state = runtime.exit_code == 0 ? "completed" : "failed";
        runtime.message = runtime.exit_code == 0 ? "执行完成" : "退出码 " + std::to_string(runtime.exit_code);
      } else if (WIFSIGNALED(status)) {
        runtime.exit_code = 128 + WTERMSIG(status);
        runtime.state = WTERMSIG(status) == SIGTERM ? "stopped" : "failed";
        runtime.message = WTERMSIG(status) == SIGTERM ? "已停止" : "被信号终止";
      }
    }
  }

  static std::string readTail(const std::string &path, std::size_t max_bytes) {
    std::ifstream input(path, std::ios::binary);
    if (!input) return "（暂无日志）";
    input.seekg(0, std::ios::end);
    const std::streamoff size = static_cast<std::streamoff>(input.tellg());
    const std::streamoff start = size > static_cast<std::streamoff>(max_bytes)
      ? size - static_cast<std::streamoff>(max_bytes) : std::streamoff{0};
    input.seekg(start);
    std::string text((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    if (start > 0) text.insert(0, "…（仅显示最后一部分）\n");
    return text.empty() ? "（暂无日志）" : text;
  }

  std::string log_dir_{"/home/tang/.local/state/yunle-script-agent"};
  std::mutex mutex_;
  std::unordered_map<std::string, Runtime> runtimes_;
};

class Server;

class Session : public std::enable_shared_from_this<Session> {
 public:
  Session(tcp::socket socket, Server &server, bool authenticated)
  : ws_(std::move(socket)), server_(server), authenticated_(authenticated) {}

  void run();
  bool send(const json &value);
  bool authenticated() const { return authenticated_.load(); }
  void authenticate() { authenticated_.store(true); }

 private:
  websocket::stream<tcp::socket> ws_;
  Server &server_;
  std::atomic<bool> authenticated_{false};
  std::mutex write_mutex_;
};

class Server {
 public:
  explicit Server(Options options)
  : options_(std::move(options)), acceptor_(ioc_) {
    const auto address = asio::ip::make_address(options_.listen);
    tcp::endpoint endpoint(address, options_.port);
    acceptor_.open(endpoint.protocol());
    acceptor_.set_option(asio::socket_base::reuse_address(true));
    acceptor_.bind(endpoint);
    acceptor_.listen();
    acceptor_.non_blocking(true);
  }

  void run() {
    std::cout << "yunle_script_agent ws://" << options_.listen << ':' << options_.port << '\n';
    while (!g_stop.load()) {
      beast::error_code ec;
      tcp::socket socket(ioc_);
      acceptor_.accept(socket, ec);
      if (!ec) {
        auto session = std::make_shared<Session>(std::move(socket), *this, options_.token.empty());
        std::thread([session] { session->run(); }).detach();
      } else if (ec != asio::error::would_block && ec != asio::error::try_again) {
        std::cerr << "accept: " << ec.message() << '\n';
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }

  void handle(const std::shared_ptr<Session> &session, const json &message) {
    const std::string type = message.value("type", "");
    if (!session->authenticated()) {
      if (type == "auth" && !options_.token.empty() && message.value("token", "") == options_.token) {
        session->authenticate();
        session->send({{"type", "auth"}, {"ok", true}});
      } else {
        session->send({{"type", "error"}, {"code", "unauthorized"},
          {"message", "authentication required"}});
      }
      return;
    }
    if (type == "script_list") session->send(manager_.list());
    else if (type == "script_start") session->send(manager_.start(message.value("id", "")));
    else if (type == "script_stop") session->send(manager_.stop(message.value("id", "")));
    else if (type == "script_log") session->send(manager_.log(message.value("id", "")));
    else if (type == "ping") session->send({{"type", "pong"}});
    else session->send({{"type", "error"}, {"code", "unsupported_message"}});
  }

  const Options &options() const { return options_; }

 private:
  Options options_;
  asio::io_context ioc_;
  tcp::acceptor acceptor_;
  ScriptManager manager_;
};

void Session::run() {
  beast::error_code ec;
  ws_.set_option(websocket::stream_base::timeout::suggested(beast::role_type::server));
  ws_.read_message_max(32 * 1024);
  ws_.accept(ec);
  if (ec) return;
  send({{"type", "hello"}, {"protocol", 1},
    {"authentication_required", !authenticated_.load()}});
  beast::flat_buffer buffer;
  while (!g_stop.load()) {
    buffer.clear();
    ws_.read(buffer, ec);
    if (ec) break;
    const json message = json::parse(beast::buffers_to_string(buffer.data()), nullptr, false);
    if (message.is_discarded() || !message.is_object()) {
      send({{"type", "error"}, {"code", "invalid_json"}});
      continue;
    }
    server_.handle(shared_from_this(), message);
  }
}

bool Session::send(const json &value) {
  std::lock_guard<std::mutex> lock(write_mutex_);
  beast::error_code ec;
  ws_.text(true);
  ws_.write(asio::buffer(value.dump()), ec);
  return !ec;
}

void usage(const char *name) {
  std::cout << "Usage: " << name << " [options]\n"
    << "  --listen ADDRESS    default 10.66.0.2\n"
    << "  --port PORT         default 8766\n"
    << "  --token TOKEN       required authentication token\n"
    << "  --self-test         validate whitelist without starting server\n";
}

Options parseArgs(int argc, char **argv) {
  Options options;
  auto take = [&](int &i) -> std::string {
    if (++i >= argc) throw std::runtime_error("missing option value");
    return argv[i];
  };
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--listen") options.listen = take(i);
    else if (arg == "--port") options.port = static_cast<uint16_t>(std::stoi(take(i)));
    else if (arg == "--token") options.token = take(i);
    else if (arg == "--self-test") options.self_test = true;
    else if (arg == "--help" || arg == "-h") { usage(argv[0]); std::exit(0); }
    else throw std::runtime_error("unknown option: " + arg);
  }
  if (!options.self_test && options.token.size() < 12) {
    throw std::runtime_error("token must contain at least 12 characters");
  }
  return options;
}

int selfTest() {
  bool ok = scriptDefinitions().size() == 3;
  for (const auto &definition : scriptDefinitions()) {
    ok = ok && !definition.id.empty() && definition.path.rfind("/home/tang/autoware/", 0) == 0;
  }
  std::cout << (ok ? "self-test passed\n" : "self-test failed\n");
  return ok ? 0 : 1;
}

void signalHandler(int) { g_stop.store(true); }

}  // namespace

int main(int argc, char **argv) {
  try {
    const Options options = parseArgs(argc, argv);
    if (options.self_test) return selfTest();
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    Server server(options);
    server.run();
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "yunle_script_agent: " << error.what() << '\n';
    return 2;
  }
}

