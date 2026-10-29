#pragma once

#include <netinet/in.h>

#include <string>

#include "s1/bridge_protocol.h"

namespace s1 {

// The replay end of the UDP link to scripts/s1_bridge.py: binds 127.0.0.1:port, receives the feedback of each SDK
// period and answers it to the address it came from.
class UdpBridge {
 public:
  UdpBridge() = default;
  ~UdpBridge();
  UdpBridge(const UdpBridge&) = delete;
  UdpBridge& operator=(const UdpBridge&) = delete;

  bool Open(int port);
  // Waits up to `timeout_s` for the next valid feedback frame, older queued frames are dropped so that the answer is
  // always for the newest one.
  bool Receive(double timeout_s, Feedback& feedback);
  // Answers the last received feedback.
  bool Send(const Command& command);

  const std::string& last_error() const { return last_error_; }

 private:
  int fd_ = -1;
  sockaddr_in peer_{};
  bool has_peer_ = false;
  std::string last_error_;
};

}  // namespace s1
