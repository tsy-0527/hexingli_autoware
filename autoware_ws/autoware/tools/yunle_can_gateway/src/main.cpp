#include <boost/asio.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <nlohmann/json.hpp>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace asio = boost::asio;
namespace beast = boost::beast;
namespace websocket = beast::websocket;
using tcp = asio::ip::tcp;
using json = nlohmann::json;
using Clock = std::chrono::steady_clock;

namespace {

constexpr canid_t kCcuStatusId = 0x51;
constexpr canid_t kBmsStatusId = 0x100;
constexpr canid_t kScuControlId = 0x121;

std::atomic<bool> g_stop{false};

uint64_t extractIntel(const uint8_t *data, unsigned start, unsigned length) {
  uint64_t out = 0;
  for (unsigned i = 0; i < length; ++i) {
    const unsigned bit = start + i;
    out |= static_cast<uint64_t>((data[bit / 8] >> (bit % 8)) & 1U) << i;
  }
  return out;
}

void insertIntel(uint8_t *data, unsigned start, unsigned length, uint64_t value) {
  for (unsigned i = 0; i < length; ++i) {
    const unsigned bit = start + i;
    const uint8_t mask = static_cast<uint8_t>(1U << (bit % 8));
    data[bit / 8] = static_cast<uint8_t>((data[bit / 8] & ~mask) |
      (((value >> i) & 1U) ? mask : 0U));
  }
}

struct CcuStatus {
  int shift{0};
  bool parking{false};
  int ignition{0};
  bool auto_button{false};
  bool steering_direction{false};
  double steering_deg{0.0};
  double speed_kmh{0.0};
  int drive_mode{0};
  bool remote_brake{false};
  bool emergency_brake{false};
  bool scu_brake{false};
  bool touch_brake{false};
  bool handle_brake{false};
  bool handle_mode{false};
  bool left_turn{false};
  bool right_turn{false};
  bool position_light{false};
  bool low_beam{false};
};

CcuStatus decodeCcu(const can_frame &frame, double physical_max_steering_deg) {
  CcuStatus s;
  s.shift = static_cast<int>(extractIntel(frame.data, 0, 2));
  s.parking = extractIntel(frame.data, 2, 1) != 0;
  s.ignition = static_cast<int>(extractIntel(frame.data, 3, 2));
  s.auto_button = extractIntel(frame.data, 5, 1) != 0;
  s.steering_direction = extractIntel(frame.data, 7, 1) != 0;
  const double code = static_cast<double>(extractIntel(frame.data, 8, 12));
  // The DBC decodes this field at 0.1 unit/bit, but that value is the same
  // steering code used by the actuator (about +/-120 at full lock), not the
  // physical front-tire angle. Convert it with the configured mechanical
  // limit so the WebSocket API reports actual degrees to the UI.
  const double angle = code * 0.1 / 120.0 * physical_max_steering_deg;
  // Chassis-native convention: direction bit 1 is right/positive, 0 is
  // left/negative.  The web UI converts this to Autoware's left-positive
  // convention at its direct-CAN boundary.
  s.steering_deg = s.steering_direction ? angle : -angle;
  s.speed_kmh = static_cast<double>(extractIntel(frame.data, 20, 9)) * 0.1;
  s.drive_mode = static_cast<int>(extractIntel(frame.data, 29, 3));
  s.remote_brake = extractIntel(frame.data, 32, 1) != 0;
  s.emergency_brake = extractIntel(frame.data, 33, 1) != 0;
  s.scu_brake = extractIntel(frame.data, 34, 1) != 0;
  s.touch_brake = extractIntel(frame.data, 35, 1) != 0;
  s.handle_brake = extractIntel(frame.data, 36, 1) != 0;
  s.handle_mode = extractIntel(frame.data, 37, 1) != 0;
  s.left_turn = extractIntel(frame.data, 56, 1) != 0;
  s.right_turn = extractIntel(frame.data, 57, 1) != 0;
  s.position_light = extractIntel(frame.data, 59, 1) != 0;
  s.low_beam = extractIntel(frame.data, 60, 1) != 0;
  return s;
}

json ccuJson(const CcuStatus &s, const std::string &interface_name) {
  return {
    {"type", "ccu_status"}, {"interface", interface_name},
    {"ccu_shift_level_status", s.shift}, {"ccu_parking_status", s.parking},
    {"ccu_ignition_status", s.ignition},
    {"ccu_drive_mode_shift_button", s.auto_button},
    {"steering_wheel_direction", s.steering_direction},
    {"ccu_steering_wheel_angle", s.steering_deg},
    {"ccu_vehicle_speed", s.speed_kmh}, {"ccu_drive_mode", s.drive_mode},
    {"remote_brake_request_status", s.remote_brake},
    {"emergency_brake_request_status", s.emergency_brake},
    {"scu_brake_signal_status", s.scu_brake},
    {"touch_brake_request_status", s.touch_brake},
    {"handle_brake_request_status", s.handle_brake},
    {"handle_mode_flag_status", s.handle_mode},
    {"left_turn_light_status", s.left_turn},
    {"right_turn_light_status", s.right_turn},
    {"position_light_status", s.position_light}, {"low_beam_status", s.low_beam}
  };
}

json bmsJson(const can_frame &frame, const std::string &interface_name) {
  const auto voltage_raw = static_cast<uint16_t>(extractIntel(frame.data, 0, 16));
  const auto current_raw = static_cast<int16_t>(extractIntel(frame.data, 16, 16));
  const auto soc_raw = static_cast<uint16_t>(extractIntel(frame.data, 32, 16));
  return {
    {"type", "bms_status"}, {"interface", interface_name},
    {"bms_voltage", voltage_raw * 0.1}, {"bms_current", current_raw * 0.1},
    {"bms_soc", static_cast<double>(soc_raw)}
  };
}

struct Command {
  double speed_mps{0.0};
  double steering_rad{0.0};
  bool force_neutral{false};
  Clock::time_point at{};
};

struct Options {
  std::string listen{"127.0.0.1"};
  uint16_t port{8765};
  std::vector<std::string> interfaces{"can0", "can1"};
  std::string control_interface{"can0"};
  std::string token;
  double max_speed_kmh{5.0};
  double max_command_steering_deg{21.0};
  double physical_max_steering_deg{21.0};
  double steering_command_sign{-1.0};
  int command_timeout_ms{200};
  int feedback_timeout_ms{500};
  bool self_test{false};
};

can_frame encodeControl(const Command &cmd, const CcuStatus &ccu, int &gear,
                        const Options &opt) {
  constexpr int kDrive = 1;
  constexpr int kNeutral = 2;
  constexpr int kReverse = 3;
  can_frame frame{};
  frame.can_id = kScuControlId;
  frame.can_dlc = 8;

  const bool wants_drive = cmd.speed_mps > 1e-4;
  const bool wants_reverse = cmd.speed_mps < -1e-4;
  const bool wants_motion = wants_drive || wants_reverse;
  const int desired = wants_drive ? kDrive : wants_reverse ? kReverse : gear;
  const bool changing_direction = wants_motion && gear != kNeutral && gear != desired;
  const bool direction_blocked = changing_direction && std::abs(ccu.speed_kmh) >= 0.2;
  if (wants_motion && !direction_blocked && std::abs(ccu.speed_kmh) < 0.2) gear = desired;
  if (cmd.force_neutral && std::abs(ccu.speed_kmh) < 0.2) gear = kNeutral;

  // This Yunle chassis keeps CCU_Parking_Status asserted after entering AUTO,
  // even when the physical brake controls are released. The production
  // yunle_autoware_bridge therefore deliberately does not use that bit as a
  // motion interlock. Keep the actual emergency/remote/touch/handle brake
  // request bits as hard interlocks.
  const bool safety_blocked = !ccu.auto_button || ccu.emergency_brake ||
    ccu.remote_brake || ccu.touch_brake || ccu.handle_brake;
  const bool allow_motion = wants_motion && !safety_blocked && !direction_blocked;
  const double speed_kmh = allow_motion
    ? std::min(std::abs(cmd.speed_mps) * 3.6, opt.max_speed_kmh) : 0.0;
  const double steering_deg = std::clamp(
    cmd.steering_rad * 180.0 / M_PI * opt.steering_command_sign,
    -opt.max_command_steering_deg, opt.max_command_steering_deg);
  // SCU_Steering_Angle_Front is an encoded signed 8-bit demand: the Yunle
  // driver contract maps the configured physical steering limit to +/-120.
  // It is not a whole-degree field even though the DBC scale is listed as 1.
  const int steering_signed = static_cast<int>(std::llround(std::clamp(
    steering_deg / opt.physical_max_steering_deg * 120.0, -120.0, 120.0)));
  const uint8_t steering_raw = static_cast<uint8_t>(steering_signed);

  insertIntel(frame.data, 0, 2, static_cast<uint64_t>(gear));
  insertIntel(frame.data, 6, 2, 1);  // automatic/ACU request
  insertIntel(frame.data, 8, 8, steering_raw);
  insertIntel(frame.data, 16, 8, 0); // rear steering centred
  insertIntel(frame.data, 24, 9, static_cast<uint64_t>(std::llround(speed_kmh * 10.0)));
  insertIntel(frame.data, 33, 1, allow_motion ? 0 : 1);
  // The current Yunle VCU accepts steering independently, but ignores the
  // requested shift/target speed unless the combined steering-speed command
  // is marked valid. This matches yunle_autoware_bridge::_base_command().
  insertIntel(frame.data, 60, 1, 1);
  return frame;
}

class Gateway;

class Session : public std::enable_shared_from_this<Session> {
 public:
  Session(tcp::socket socket, Gateway &gateway, bool authenticated)
      : ws_(std::move(socket)), gateway_(gateway), authenticated_(authenticated) {}
  void run();
  bool send(const json &value);
  void close();
  bool authenticated() const { return authenticated_; }
  void authenticate() { authenticated_ = true; }

 private:
  websocket::stream<tcp::socket> ws_;
  Gateway &gateway_;
  std::mutex write_mutex_;
  std::atomic<bool> closed_{false};
  std::atomic<bool> authenticated_{false};
};

class Gateway {
 public:
  explicit Gateway(Options options)
      : options_(std::move(options)), ioc_(1),
        acceptor_(ioc_, tcp::endpoint(asio::ip::make_address(options_.listen), options_.port)) {
    acceptor_.non_blocking(true);
  }

  ~Gateway() { stop(); }

  bool startCan() {
    for (const auto &name : options_.interfaces) {
      const int fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
      if (fd < 0) {
        std::cerr << "socket(" << name << "): " << std::strerror(errno) << '\n';
        return false;
      }
      ifreq ifr{};
      std::strncpy(ifr.ifr_name, name.c_str(), IFNAMSIZ - 1);
      if (::ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
        std::cerr << "interface " << name << ": " << std::strerror(errno) << '\n';
        ::close(fd);
        return false;
      }
      sockaddr_can addr{};
      addr.can_family = AF_CAN;
      addr.can_ifindex = ifr.ifr_ifindex;
      if (::bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
        std::cerr << "bind(" << name << "): " << std::strerror(errno) << '\n';
        ::close(fd);
        return false;
      }
      int disable_loopback = 0;
      ::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS,
                   &disable_loopback, sizeof(disable_loopback));
      can_fds_[name] = fd;
      rx_threads_.emplace_back([this, name, fd] { receiveLoop(name, fd); });
    }
    if (can_fds_.find(options_.control_interface) == can_fds_.end()) {
      std::cerr << "control interface is not in --interfaces: "
                << options_.control_interface << '\n';
      return false;
    }
    control_thread_ = std::thread([this] { controlLoop(); });
    status_thread_ = std::thread([this] { statusLoop(); });
    return true;
  }

  void run() {
    std::cout << "yunle_can_gateway ws://" << options_.listen << ':' << options_.port
              << " control=" << options_.control_interface << " interfaces=";
    for (const auto &name : options_.interfaces) std::cout << name << ' ';
    std::cout << '\n';
    while (!g_stop.load()) {
      beast::error_code ec;
      tcp::socket socket(ioc_);
      acceptor_.accept(socket, ec);
      if (!ec) {
        auto session = std::make_shared<Session>(
          std::move(socket), *this, options_.token.empty());
        std::thread([session] { session->run(); }).detach();
      } else if (ec != asio::error::would_block && ec != asio::error::try_again) {
        std::cerr << "accept: " << ec.message() << '\n';
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }

  void stop() {
    if (stopped_.exchange(true)) return;
    g_stop.store(true);
    beast::error_code ignored;
    acceptor_.close(ignored);
    {
      std::lock_guard<std::mutex> lock(sessions_mutex_);
      for (auto &weak : sessions_) if (auto s = weak.lock()) s->close();
    }
    for (auto &[name, fd] : can_fds_) {
      (void)name;
      ::shutdown(fd, SHUT_RDWR);
      ::close(fd);
    }
    can_fds_.clear();
    for (auto &thread : rx_threads_) if (thread.joinable()) thread.join();
    if (control_thread_.joinable()) control_thread_.join();
    if (status_thread_.joinable()) status_thread_.join();
  }

  void addSession(const std::shared_ptr<Session> &session) {
    std::lock_guard<std::mutex> lock(sessions_mutex_);
    sessions_.push_back(session);
  }

  void removeSession(const std::shared_ptr<Session> &session) {
    {
      std::lock_guard<std::mutex> lock(sessions_mutex_);
      sessions_.erase(std::remove_if(sessions_.begin(), sessions_.end(),
        [&](const std::weak_ptr<Session> &w) {
          auto p = w.lock();
          return !p || p == session;
        }), sessions_.end());
    }
    std::lock_guard<std::mutex> lock(control_mutex_);
    if (owner_.lock() == session) {
      stop_until_ = Clock::now() + std::chrono::milliseconds(options_.command_timeout_ms);
      owner_.reset();
      latest_command_.reset();
    }
  }

  void handle(const std::shared_ptr<Session> &session, const json &message) {
    const std::string type = message.value("type", "");
    if (!session->authenticated()) {
      if (type == "auth" && message.value("token", "") == options_.token) {
        session->authenticate();
        session->send({{"type", "auth"}, {"ok", true}});
      } else {
        session->send({{"type", "error"}, {"code", "unauthorized"},
                       {"message", "authentication required"}});
      }
      return;
    }
    if (type == "claim_control") {
      std::lock_guard<std::mutex> lock(control_mutex_);
      if (auto owner = owner_.lock(); owner && owner != session) {
        session->send({{"type", "control_claim"}, {"ok", false},
                       {"message", "another client owns control"}});
      } else {
        owner_ = session;
        latest_command_.reset();
        session->send({{"type", "control_claim"}, {"ok", true}});
      }
      return;
    }
    if (type == "release_control") {
      std::lock_guard<std::mutex> lock(control_mutex_);
      if (owner_.lock() == session) {
        stop_until_ = Clock::now() + std::chrono::milliseconds(options_.command_timeout_ms);
        latest_command_.reset();
        owner_.reset();
      }
      session->send({{"type", "control_release"}, {"ok", true}});
      return;
    }
    if (type == "command") {
      std::lock_guard<std::mutex> lock(control_mutex_);
      if (owner_.lock() != session) {
        session->send({{"type", "error"}, {"code", "not_control_owner"},
                       {"message", "claim control before sending commands"}});
        return;
      }
      Command cmd;
      cmd.speed_mps = message.value("speed_mps", 0.0);
      cmd.steering_rad = message.value("steering_rad", 0.0);
      cmd.force_neutral = message.value("force_neutral", false);
      if (!std::isfinite(cmd.speed_mps) || !std::isfinite(cmd.steering_rad)) {
        session->send({{"type", "error"}, {"code", "invalid_command"}});
        return;
      }
      cmd.speed_mps = std::clamp(cmd.speed_mps,
        -options_.max_speed_kmh / 3.6, options_.max_speed_kmh / 3.6);
      cmd.steering_rad = std::clamp(cmd.steering_rad,
        -options_.max_command_steering_deg * M_PI / 180.0,
         options_.max_command_steering_deg * M_PI / 180.0);
      cmd.at = Clock::now();
      latest_command_ = cmd;
      return;
    }
    if (type == "ping") session->send({{"type", "pong"}});
  }

  const Options &options() const { return options_; }

 private:
  void broadcast(const json &value) {
    std::vector<std::shared_ptr<Session>> targets;
    {
      std::lock_guard<std::mutex> lock(sessions_mutex_);
      for (auto it = sessions_.begin(); it != sessions_.end();) {
        if (auto session = it->lock()) {
          if (session->authenticated()) targets.push_back(std::move(session));
          ++it;
        } else {
          it = sessions_.erase(it);
        }
      }
    }
    for (const auto &session : targets) session->send(value);
  }

  void receiveLoop(const std::string &name, int fd) {
    pollfd pfd{fd, POLLIN, 0};
    while (!g_stop.load()) {
      const int ready = ::poll(&pfd, 1, 100);
      if (ready <= 0 || !(pfd.revents & POLLIN)) continue;
      can_frame frame{};
      const ssize_t got = ::read(fd, &frame, sizeof(frame));
      if (got != static_cast<ssize_t>(sizeof(frame))) continue;
      const canid_t id = frame.can_id & CAN_SFF_MASK;
      if (id == kCcuStatusId && frame.can_dlc >= 8) {
        const CcuStatus status = decodeCcu(frame, options_.physical_max_steering_deg);
        bool duplicate = false;
        {
          std::lock_guard<std::mutex> lock(feedback_mutex_);
          const auto now = Clock::now();
          duplicate = ccu_at_ != Clock::time_point{} &&
            now - ccu_at_ < std::chrono::milliseconds(2) &&
            std::equal(last_ccu_payload_.begin(), last_ccu_payload_.end(), frame.data);
          if (!duplicate) {
            ccu_ = status;
            ccu_at_ = now;
            std::copy_n(frame.data, last_ccu_payload_.size(), last_ccu_payload_.begin());
            ++ccu_count_;
          }
        }
        // The current dual-channel bridge mirrors 0x51 onto can0 and can1.
        // Count/broadcast it once so the browser sees the real bus frequency.
        if (!duplicate) broadcast(ccuJson(status, name));
      } else if (id == kBmsStatusId && frame.can_dlc >= 6) {
        broadcast(bmsJson(frame, name));
      }
    }
  }

  void controlLoop() {
    auto next = Clock::now();
    while (!g_stop.load()) {
      next += std::chrono::milliseconds(10);
      std::optional<Command> command;
      {
        std::lock_guard<std::mutex> lock(control_mutex_);
        const auto now = Clock::now();
        if (now < stop_until_) {
          command = Command{0.0, 0.0, true, now};
        } else {
          const auto owner = owner_.lock();
          if (owner && latest_command_) command = latest_command_;
        }
      }
      std::optional<CcuStatus> ccu;
      Clock::time_point ccu_at;
      {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        ccu = ccu_;
        ccu_at = ccu_at_;
      }
      const auto now = Clock::now();
      const bool command_fresh = command &&
        now - command->at <= std::chrono::milliseconds(options_.command_timeout_ms);
      const bool feedback_fresh = ccu &&
        now - ccu_at <= std::chrono::milliseconds(options_.feedback_timeout_ms);
      if (command_fresh && feedback_fresh) {
        const can_frame frame = encodeControl(*command, *ccu, current_gear_, options_);
        const auto it = can_fds_.find(options_.control_interface);
        if (it != can_fds_.end()) {
          std::lock_guard<std::mutex> lock(can_tx_mutex_);
          if (::write(it->second, &frame, sizeof(frame)) != static_cast<ssize_t>(sizeof(frame))) {
            std::cerr << "CAN write " << options_.control_interface << ": "
                      << std::strerror(errno) << '\n';
          }
        }
      }
      std::this_thread::sleep_until(next);
    }
  }

  void statusLoop() {
    uint64_t previous_count = 0;
    while (!g_stop.load()) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
      uint64_t count;
      Clock::time_point at;
      {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        count = ccu_count_;
        at = ccu_at_;
      }
      bool owned;
      {
        std::lock_guard<std::mutex> lock(control_mutex_);
        owned = !owner_.expired();
      }
      const auto age_ms = at == Clock::time_point{} ? -1LL :
        std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - at).count();
      broadcast({{"type", "gateway_status"}, {"control_interface", options_.control_interface},
        {"interfaces", options_.interfaces}, {"control_owned", owned},
        {"ccu_hz", count - previous_count}, {"ccu_age_ms", age_ms}});
      previous_count = count;
    }
  }

  Options options_;
  asio::io_context ioc_;
  tcp::acceptor acceptor_;
  std::atomic<bool> stopped_{false};
  std::unordered_map<std::string, int> can_fds_;
  std::vector<std::thread> rx_threads_;
  std::thread control_thread_;
  std::thread status_thread_;
  std::mutex can_tx_mutex_;
  std::mutex sessions_mutex_;
  std::vector<std::weak_ptr<Session>> sessions_;
  std::mutex feedback_mutex_;
  std::optional<CcuStatus> ccu_;
  Clock::time_point ccu_at_{};
  std::array<uint8_t, 8> last_ccu_payload_{};
  uint64_t ccu_count_{0};
  std::mutex control_mutex_;
  std::weak_ptr<Session> owner_;
  std::optional<Command> latest_command_;
  Clock::time_point stop_until_{};
  int current_gear_{2};
};

void Session::run() {
  beast::error_code ec;
  ws_.set_option(websocket::stream_base::timeout::suggested(beast::role_type::server));
  ws_.accept(ec);
  if (ec) return;
  gateway_.addSession(shared_from_this());
  send({{"type", "hello"}, {"protocol", 1},
        {"authentication_required", !authenticated_.load()},
        {"control_interface", gateway_.options().control_interface}});
  beast::flat_buffer buffer;
  while (!g_stop.load() && !closed_.load()) {
    buffer.clear();
    ws_.read(buffer, ec);
    if (ec) break;
    const std::string text = beast::buffers_to_string(buffer.data());
    const json message = json::parse(text, nullptr, false);
    if (message.is_discarded() || !message.is_object()) {
      send({{"type", "error"}, {"code", "invalid_json"}});
      continue;
    }
    gateway_.handle(shared_from_this(), message);
  }
  gateway_.removeSession(shared_from_this());
  close();
}

bool Session::send(const json &value) {
  if (closed_.load()) return false;
  std::lock_guard<std::mutex> lock(write_mutex_);
  beast::error_code ec;
  ws_.text(true);
  ws_.write(asio::buffer(value.dump()), ec);
  if (ec) closed_.store(true);
  return !ec;
}

void Session::close() {
  if (closed_.exchange(true)) return;
  std::lock_guard<std::mutex> lock(write_mutex_);
  beast::error_code ignored;
  ws_.close(websocket::close_code::normal, ignored);
}

void usage(const char *name) {
  std::cout
    << "Usage: " << name << " [options]\n"
    << "  --listen ADDRESS                 default 127.0.0.1\n"
    << "  --port PORT                      default 8765\n"
    << "  --interfaces can0,can1           SocketCAN receive interfaces\n"
    << "  --control-interface can0         interface carrying CAN 0x121\n"
    << "  --token TOKEN                    require WebSocket authentication\n"
    << "  --max-speed-kmh VALUE            hard speed limit, default 5.0\n"
    << "  --max-steering-deg VALUE         command limit, default 21.0\n"
    << "  --physical-max-steering-deg V    angle represented by raw +/-120, default 21.0\n"
    << "  --steering-sign -1|1             default -1\n"
    << "  --self-test                      test codec without opening CAN\n";
}

std::vector<std::string> splitComma(const std::string &value) {
  std::vector<std::string> result;
  size_t from = 0;
  while (from <= value.size()) {
    const size_t comma = value.find(',', from);
    const std::string item = value.substr(from, comma - from);
    if (!item.empty()) result.push_back(item);
    if (comma == std::string::npos) break;
    from = comma + 1;
  }
  return result;
}

Options parseArgs(int argc, char **argv) {
  Options o;
  auto take = [&](int &i) -> std::string {
    if (++i >= argc) throw std::runtime_error("missing option value");
    return argv[i];
  };
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--listen") o.listen = take(i);
    else if (arg == "--port") o.port = static_cast<uint16_t>(std::stoi(take(i)));
    else if (arg == "--interfaces") o.interfaces = splitComma(take(i));
    else if (arg == "--control-interface") o.control_interface = take(i);
    else if (arg == "--token") o.token = take(i);
    else if (arg == "--max-speed-kmh") o.max_speed_kmh = std::stod(take(i));
    else if (arg == "--max-steering-deg") o.max_command_steering_deg = std::stod(take(i));
    else if (arg == "--physical-max-steering-deg") o.physical_max_steering_deg = std::stod(take(i));
    else if (arg == "--steering-sign") o.steering_command_sign = std::stod(take(i));
    else if (arg == "--self-test") o.self_test = true;
    else if (arg == "--help" || arg == "-h") { usage(argv[0]); std::exit(0); }
    else throw std::runtime_error("unknown option: " + arg);
  }
  if (o.interfaces.empty() || o.max_speed_kmh <= 0 || o.max_command_steering_deg <= 0 ||
      o.physical_max_steering_deg <= 0 || (o.steering_command_sign != -1.0 && o.steering_command_sign != 1.0)) {
    throw std::runtime_error("invalid option value");
  }
  return o;
}

int selfTest() {
  Options opt;
  CcuStatus ccu;
  ccu.auto_button = true;
  ccu.parking = true;  // asserted continuously on the current vehicle in AUTO
  int gear = 2;
  Command cmd{0.15, 21.0 * M_PI / 180.0, false, Clock::now()};
  const can_frame out = encodeControl(cmd, ccu, gear, opt);
  const can_frame right = encodeControl(
    Command{0.15, -21.0 * M_PI / 180.0, false, Clock::now()}, ccu, gear, opt);
  const bool encode_ok = out.can_id == 0x121 && gear == 1 &&
    extractIntel(out.data, 0, 2) == 1 && extractIntel(out.data, 6, 2) == 1 &&
    extractIntel(out.data, 8, 8) == 136 && extractIntel(right.data, 8, 8) == 120 &&
    extractIntel(out.data, 24, 9) == 5 && extractIntel(out.data, 33, 1) == 0 &&
    extractIntel(out.data, 60, 1) == 1;

  ccu.speed_kmh = 1.0;
  gear = 1;
  const can_frame blocked_reverse = encodeControl(
    Command{-0.15, 0.0, false, Clock::now()}, ccu, gear, opt);
  const bool direction_lock_ok = gear == 1 &&
    extractIntel(blocked_reverse.data, 24, 9) == 0 &&
    extractIntel(blocked_reverse.data, 33, 1) == 1;

  ccu.speed_kmh = 0.0;
  ccu.auto_button = false;
  const can_frame physical_lock = encodeControl(cmd, ccu, gear, opt);
  const bool physical_lock_ok = extractIntel(physical_lock.data, 24, 9) == 0 &&
    extractIntel(physical_lock.data, 33, 1) == 1;

  can_frame in{};
  in.can_id = 0x51;
  in.can_dlc = 8;
  insertIntel(in.data, 0, 2, 1);
  insertIntel(in.data, 5, 1, 1);
  insertIntel(in.data, 7, 1, 1);
  insertIntel(in.data, 8, 12, 60);
  insertIntel(in.data, 20, 9, 37);
  insertIntel(in.data, 29, 3, 1);
  const CcuStatus decoded = decodeCcu(in, opt.physical_max_steering_deg);
  const bool decode_ok = decoded.shift == 1 && decoded.auto_button &&
    std::abs(decoded.steering_deg - 1.05) < 1e-9 &&
    std::abs(decoded.speed_kmh - 3.7) < 1e-9 && decoded.drive_mode == 1;
  if (!encode_ok || !direction_lock_ok || !physical_lock_ok || !decode_ok) {
    std::cerr << "self-test failed: encode=" << encode_ok
              << " direction_lock=" << direction_lock_ok
              << " physical_lock=" << physical_lock_ok
              << " decode=" << decode_ok << '\n';
    return 1;
  }
  std::cout << "self-test passed\n";
  return 0;
}

}  // namespace

int main(int argc, char **argv) {
  try {
    const Options options = parseArgs(argc, argv);
    if (options.self_test) return selfTest();
    std::signal(SIGINT, [](int) { g_stop.store(true); });
    std::signal(SIGTERM, [](int) { g_stop.store(true); });
    Gateway gateway(options);
    if (!gateway.startCan()) return 1;
    gateway.run();
    gateway.stop();
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "yunle_can_gateway: " << e.what() << '\n';
    usage(argv[0]);
    return 2;
  }
}

