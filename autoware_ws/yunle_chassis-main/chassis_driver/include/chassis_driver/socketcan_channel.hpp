#pragma once

#include "chassis_driver/can_types.hpp"

#include <string>

namespace chassis_driver
{

/** Thin wrapper around a Linux classic-CAN RAW socket. */
class SocketCanChannel
{
public:
  SocketCanChannel() = default;
  ~SocketCanChannel();

  SocketCanChannel(const SocketCanChannel &) = delete;
  SocketCanChannel & operator=(const SocketCanChannel &) = delete;

  bool open(const std::string & interface_name, int receive_timeout_ms);
  bool receive(CanFrame & frame);
  bool send(const CanFrame & frame);
  void close();

  const std::string & interfaceName() const { return interface_name_; }

private:
  int fd_{-1};
  bool opened_{false};
  std::string interface_name_;
};

}  // namespace chassis_driver
