#include "chassis_driver/socketcan_channel.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cstring>

namespace chassis_driver
{

SocketCanChannel::~SocketCanChannel()
{
  close();
}

bool SocketCanChannel::open(const std::string & interface_name, int receive_timeout_ms)
{
  close();

  fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd_ < 0) {
    return false;
  }

  ifreq request{};
  std::strncpy(request.ifr_name, interface_name.c_str(), IFNAMSIZ - 1U);
  request.ifr_name[IFNAMSIZ - 1U] = '\0';
  if (::ioctl(fd_, SIOCGIFINDEX, &request) < 0) {
    close();
    return false;
  }

  const int receive_own_messages = 0;
  if (
    ::setsockopt(
      fd_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &receive_own_messages,
      sizeof(receive_own_messages)) < 0) {
    close();
    return false;
  }

  if (receive_timeout_ms > 0) {
    timeval timeout{};
    timeout.tv_sec = receive_timeout_ms / 1000;
    timeout.tv_usec = (receive_timeout_ms % 1000) * 1000;
    if (::setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
      close();
      return false;
    }
  }

  sockaddr_can address{};
  address.can_family = AF_CAN;
  address.can_ifindex = request.ifr_ifindex;
  if (::bind(fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
    close();
    return false;
  }

  interface_name_ = interface_name;
  opened_ = true;
  return true;
}

bool SocketCanChannel::receive(CanFrame & output)
{
  if (!opened_) {
    return false;
  }

  can_frame input{};
  const ssize_t received = ::read(fd_, &input, sizeof(input));
  if (received != static_cast<ssize_t>(sizeof(input))) {
    return false;
  }

  if ((input.can_id & CAN_ERR_FLAG) != 0U) {
    return false;
  }

  output = CanFrame{};
  output.is_extended = (input.can_id & CAN_EFF_FLAG) != 0U;
  output.is_remote = (input.can_id & CAN_RTR_FLAG) != 0U;
  output.can_id =
    output.is_extended ? (input.can_id & CAN_EFF_MASK) : (input.can_id & CAN_SFF_MASK);
  output.dlc = std::min<uint8_t>(input.can_dlc, static_cast<uint8_t>(8U));
  std::copy_n(input.data, output.dlc, output.data.begin());
  return true;
}

bool SocketCanChannel::send(const CanFrame & input)
{
  if (!opened_) {
    return false;
  }

  can_frame output{};
  output.can_id = input.is_extended ? ((input.can_id & CAN_EFF_MASK) | CAN_EFF_FLAG)
                                    : (input.can_id & CAN_SFF_MASK);
  if (input.is_remote) {
    output.can_id |= CAN_RTR_FLAG;
  }
  output.can_dlc = std::min<uint8_t>(input.dlc, static_cast<uint8_t>(8U));
  std::copy_n(input.data.begin(), output.can_dlc, output.data);

  return ::write(fd_, &output, sizeof(output)) == static_cast<ssize_t>(sizeof(output));
}

void SocketCanChannel::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  opened_ = false;
  interface_name_.clear();
}

}  // namespace chassis_driver
