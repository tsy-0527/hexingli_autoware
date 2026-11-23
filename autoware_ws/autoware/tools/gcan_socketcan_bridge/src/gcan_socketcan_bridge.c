#define _GNU_SOURCE

#include <errno.h>
#include <getopt.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include "ECanVci.h"

#define GCAN_USBCAN1 3U
#define GCAN_USBCAN2 4U
#define MAX_BRIDGE_CHANNELS 2U
#define RX_BATCH 256U

struct bridge_channel {
  const char *interface_name;
  unsigned int bitrate;
  DWORD channel;
  int can_fd;
  unsigned long long tx_count;
  unsigned long long rx_count;
};

static volatile sig_atomic_t keep_running = 1;

static void stop_handler(int signo)
{
  (void)signo;
  keep_running = 0;
}

static void log_message(const char *level, const char *message)
{
  struct timespec now;
  clock_gettime(CLOCK_REALTIME, &now);
  fprintf(stderr, "%lld.%03ld [%s] %s\n",
          (long long)now.tv_sec, now.tv_nsec / 1000000L, level, message);
}

static int bitrate_to_timing(unsigned int bitrate, UCHAR *timing0, UCHAR *timing1)
{
  struct timing_entry {
    unsigned int bitrate;
    UCHAR timing0;
    UCHAR timing1;
  };
  static const struct timing_entry timings[] = {
    {10000U, 0x31, 0x1c},
    {20000U, 0x18, 0x1c},
    {50000U, 0x09, 0x1c},
    {100000U, 0x04, 0x1c},
    {125000U, 0x03, 0x1c},
    {250000U, 0x01, 0x1c},
    {500000U, 0x00, 0x1c},
    {800000U, 0x00, 0x16},
    {1000000U, 0x00, 0x14},
  };

  for (size_t i = 0; i < sizeof(timings) / sizeof(timings[0]); ++i) {
    if (timings[i].bitrate == bitrate) {
      *timing0 = timings[i].timing0;
      *timing1 = timings[i].timing1;
      return 0;
    }
  }
  return -1;
}

static int open_socketcan(const char *interface_name)
{
  int fd = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
  if (fd < 0) {
    perror("socket(PF_CAN)");
    return -1;
  }

  struct ifreq ifr;
  memset(&ifr, 0, sizeof(ifr));
  snprintf(ifr.ifr_name, sizeof(ifr.ifr_name), "%s", interface_name);
  if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
    perror("SIOCGIFINDEX");
    close(fd);
    return -1;
  }

  struct sockaddr_can address;
  memset(&address, 0, sizeof(address));
  address.can_family = AF_CAN;
  address.can_ifindex = ifr.ifr_ifindex;
  if (bind(fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
    perror("bind(PF_CAN)");
    close(fd);
    return -1;
  }
  return fd;
}

static int open_gcan_device(DWORD device_type, DWORD device_index, BOARD_INFO *board_info)
{
  if (OpenDevice(device_type, device_index, 0) != STATUS_OK) {
    log_message("ERROR", "OpenDevice failed; check USB presence and permissions");
    return -1;
  }

  memset(board_info, 0, sizeof(*board_info));
  if (ReadBoardInfo(device_type, device_index, board_info) == STATUS_OK) {
    fprintf(stderr, "GCAN serial=%.20s hardware=%.40s channels=%u\n",
            board_info->str_Serial_Num, board_info->str_hw_Type, board_info->can_Num);
    if (board_info->can_Num > MAX_BRIDGE_CHANNELS) {
      log_message("WARN", "device returned an invalid channel count; ignoring that field");
      board_info->can_Num = 0U;
    }
  } else {
    log_message("WARN", "ReadBoardInfo failed; continuing without channel-count validation");
  }
  return 0;
}

static int initialize_gcan_channel(
  DWORD device_type, DWORD device_index, DWORD channel, unsigned int bitrate)
{
  UCHAR timing0;
  UCHAR timing1;
  if (bitrate_to_timing(bitrate, &timing0, &timing1) != 0) {
    fprintf(stderr, "Unsupported bitrate: %u\n", bitrate);
    return -1;
  }

  INIT_CONFIG config;
  memset(&config, 0, sizeof(config));
  config.AccCode = 0x00000000U;
  config.AccMask = 0xffffffffU;
  config.Filter = 0;
  config.Timing0 = timing0;
  config.Timing1 = timing1;
  config.Mode = 0;

  if (InitCAN(device_type, device_index, channel, &config) != STATUS_OK) {
    fprintf(stderr, "InitCAN failed for channel %u\n", channel);
    return -1;
  }
  if (StartCAN(device_type, device_index, channel) != STATUS_OK) {
    fprintf(stderr, "StartCAN failed for channel %u\n", channel);
    return -1;
  }
  ClearBuffer(device_type, device_index, channel);
  fprintf(stderr, "GCAN channel %u started at %u bit/s\n", channel, bitrate);
  return 0;
}

static int forward_socketcan_to_gcan(
  struct bridge_channel *bridge, DWORD device_type, DWORD device_index)
{
  for (;;) {
    struct can_frame frame;
    ssize_t received = read(bridge->can_fd, &frame, sizeof(frame));
    if (received < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        return 0;
      }
      perror("read(SocketCAN)");
      return -1;
    }
    if ((size_t)received != sizeof(frame)) {
      log_message("WARN", "ignored short SocketCAN frame");
      continue;
    }
    if ((frame.can_id & CAN_ERR_FLAG) != 0U) {
      continue;
    }

    CAN_OBJ object;
    memset(&object, 0, sizeof(object));
    object.ExternFlag = (frame.can_id & CAN_EFF_FLAG) ? 1U : 0U;
    object.RemoteFlag = (frame.can_id & CAN_RTR_FLAG) ? 1U : 0U;
    object.ID = frame.can_id & (object.ExternFlag ? CAN_EFF_MASK : CAN_SFF_MASK);
    object.DataLen = frame.can_dlc > CAN_MAX_DLEN ? CAN_MAX_DLEN : frame.can_dlc;
    memcpy(object.Data, frame.data, object.DataLen);

    if (Transmit(device_type, device_index, bridge->channel, &object, 1) != 1U) {
      fprintf(stderr, "GCAN transmit failed on channel %u\n", bridge->channel);
      return -1;
    }
    ++bridge->tx_count;
  }
}

static int forward_gcan_to_socketcan(
  struct bridge_channel *bridge, DWORD device_type, DWORD device_index)
{
  CAN_OBJ objects[RX_BATCH];
  DWORD count = Receive(
    device_type, device_index, bridge->channel, objects, RX_BATCH, 0);
  if (count > RX_BATCH) {
    log_message("ERROR", "GCAN receive returned an invalid frame count");
    return -1;
  }

  for (DWORD i = 0; i < count; ++i) {
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id = objects[i].ID & (objects[i].ExternFlag ? CAN_EFF_MASK : CAN_SFF_MASK);
    if (objects[i].ExternFlag) {
      frame.can_id |= CAN_EFF_FLAG;
    }
    if (objects[i].RemoteFlag) {
      frame.can_id |= CAN_RTR_FLAG;
    }
    frame.can_dlc = objects[i].DataLen > CAN_MAX_DLEN ? CAN_MAX_DLEN : objects[i].DataLen;
    memcpy(frame.data, objects[i].Data, frame.can_dlc);

    ssize_t written = write(bridge->can_fd, &frame, sizeof(frame));
    if (written < 0) {
      perror("write(SocketCAN)");
      return -1;
    }
    ++bridge->rx_count;
  }
  return 0;
}

static void print_usage(const char *program)
{
  fprintf(stderr,
          "Usage: %s [--interface can0] [--bitrate 500000] [--device-type 3] "
          "[--device-index 0] [--channel 0] [--dual] "
          "[--second-interface can1] [--second-channel 1] "
          "[--second-bitrate 500000] [--probe]\n"
          "  --dual  bridge one USBCAN-II (device type 4) as can0 and can1\n",
          program);
}

int main(int argc, char **argv)
{
  struct bridge_channel bridges[MAX_BRIDGE_CHANNELS] = {
    {.interface_name = "can0", .bitrate = 500000U, .channel = 0U, .can_fd = -1},
    {.interface_name = "can1", .bitrate = 0U, .channel = 1U, .can_fd = -1},
  };
  size_t bridge_count = 1U;
  DWORD device_type = GCAN_USBCAN1;
  DWORD device_index = 0U;
  bool device_type_was_set = false;
  bool second_bitrate_was_set = false;
  bool probe_only = false;

  static const struct option options[] = {
    {"interface", required_argument, NULL, 'i'},
    {"bitrate", required_argument, NULL, 'b'},
    {"device-type", required_argument, NULL, 't'},
    {"device-index", required_argument, NULL, 'd'},
    {"channel", required_argument, NULL, 'c'},
    {"dual", no_argument, NULL, '2'},
    {"second-interface", required_argument, NULL, 'I'},
    {"second-channel", required_argument, NULL, 'C'},
    {"second-bitrate", required_argument, NULL, 'B'},
    {"probe", no_argument, NULL, 'p'},
    {"help", no_argument, NULL, 'h'},
    {NULL, 0, NULL, 0},
  };

  int option;
  while ((option = getopt_long(argc, argv, "i:b:t:d:c:2I:C:B:ph", options, NULL)) != -1) {
    switch (option) {
      case 'i': bridges[0].interface_name = optarg; break;
      case 'b': bridges[0].bitrate = (unsigned int)strtoul(optarg, NULL, 0); break;
      case 't':
        device_type = (DWORD)strtoul(optarg, NULL, 0);
        device_type_was_set = true;
        break;
      case 'd': device_index = (DWORD)strtoul(optarg, NULL, 0); break;
      case 'c': bridges[0].channel = (DWORD)strtoul(optarg, NULL, 0); break;
      case '2': bridge_count = MAX_BRIDGE_CHANNELS; break;
      case 'I':
        bridges[1].interface_name = optarg;
        bridge_count = MAX_BRIDGE_CHANNELS;
        break;
      case 'C':
        bridges[1].channel = (DWORD)strtoul(optarg, NULL, 0);
        bridge_count = MAX_BRIDGE_CHANNELS;
        break;
      case 'B':
        bridges[1].bitrate = (unsigned int)strtoul(optarg, NULL, 0);
        second_bitrate_was_set = true;
        bridge_count = MAX_BRIDGE_CHANNELS;
        break;
      case 'p': probe_only = true; break;
      case 'h': print_usage(argv[0]); return EXIT_SUCCESS;
      default: print_usage(argv[0]); return EXIT_FAILURE;
    }
  }

  if (bridge_count == MAX_BRIDGE_CHANNELS && !device_type_was_set) {
    device_type = GCAN_USBCAN2;
  }
  if (!second_bitrate_was_set) {
    bridges[1].bitrate = bridges[0].bitrate;
  }
  if (bridge_count == MAX_BRIDGE_CHANNELS &&
      (bridges[0].channel == bridges[1].channel ||
       strcmp(bridges[0].interface_name, bridges[1].interface_name) == 0)) {
    log_message("ERROR", "dual mode requires two distinct channels and interfaces");
    return EXIT_FAILURE;
  }

  BOARD_INFO board_info;
  if (open_gcan_device(device_type, device_index, &board_info) != 0) {
    return EXIT_FAILURE;
  }

  size_t initialized_count = 0U;
  for (size_t i = 0; i < bridge_count; ++i) {
    if (board_info.can_Num != 0U && bridges[i].channel >= board_info.can_Num) {
      fprintf(stderr, "GCAN channel %u is unavailable; device reports %u channel(s)\n",
              bridges[i].channel, board_info.can_Num);
      goto cleanup_device;
    }
    if (initialize_gcan_channel(
          device_type, device_index, bridges[i].channel, bridges[i].bitrate) != 0) {
      goto cleanup_device;
    }
    ++initialized_count;
  }
  if (probe_only) {
    for (size_t i = 0; i < initialized_count; ++i) {
      ResetCAN(device_type, device_index, bridges[i].channel);
    }
    CloseDevice(device_type, device_index);
    log_message("INFO", "probe completed without transmitting frames");
    return EXIT_SUCCESS;
  }

  for (size_t i = 0; i < bridge_count; ++i) {
    bridges[i].can_fd = open_socketcan(bridges[i].interface_name);
    if (bridges[i].can_fd < 0) {
      goto cleanup_sockets;
    }
  }

  signal(SIGINT, stop_handler);
  signal(SIGTERM, stop_handler);
  for (size_t i = 0; i < bridge_count; ++i) {
    fprintf(stderr, "Bridge ready: %s <-> GCAN device %u channel %u\n",
            bridges[i].interface_name, device_index, bridges[i].channel);
  }

  while (keep_running) {
    bool failed = false;
    struct pollfd poll_fds[MAX_BRIDGE_CHANNELS];
    for (size_t i = 0; i < bridge_count; ++i) {
      if (forward_gcan_to_socketcan(&bridges[i], device_type, device_index) != 0) {
        failed = true;
        break;
      }
      poll_fds[i].fd = bridges[i].can_fd;
      poll_fds[i].events = POLLIN;
      poll_fds[i].revents = 0;
    }
    if (failed) {
      break;
    }

    int poll_result = poll(poll_fds, bridge_count, 2);
    if (poll_result < 0 && errno != EINTR) {
      perror("poll(SocketCAN)");
      break;
    }
    if (poll_result > 0) {
      for (size_t i = 0; i < bridge_count; ++i) {
        if ((poll_fds[i].revents & POLLIN) != 0 &&
            forward_socketcan_to_gcan(&bridges[i], device_type, device_index) != 0) {
          failed = true;
          break;
        }
      }
    }
    if (failed) {
      break;
    }
  }

cleanup_sockets:
  for (size_t i = 0; i < bridge_count; ++i) {
    if (bridges[i].can_fd >= 0) {
      close(bridges[i].can_fd);
    }
  }
cleanup_device:
  for (size_t i = 0; i < initialized_count; ++i) {
    ResetCAN(device_type, device_index, bridges[i].channel);
  }
  CloseDevice(device_type, device_index);
  for (size_t i = 0; i < bridge_count; ++i) {
    fprintf(stderr, "Bridge stopped: %s host->bus=%llu bus->host=%llu\n",
            bridges[i].interface_name, bridges[i].tx_count, bridges[i].rx_count);
  }
  return keep_running ? EXIT_FAILURE : EXIT_SUCCESS;
}
