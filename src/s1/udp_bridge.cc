#include "s1/udp_bridge.h"

#include <arpa/inet.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <cstring>

namespace s1 {

UdpBridge::~UdpBridge() {
  if (fd_ >= 0) {
    close(fd_);
  }
}

bool UdpBridge::Open(int port) {
  fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd_ < 0) {
    last_error_ = std::string("socket: ") + std::strerror(errno);
    return false;
  }
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(static_cast<uint16_t>(port));
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  if (bind(fd_, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) != 0) {
    last_error_ = "bind 127.0.0.1:" + std::to_string(port) + ": " + std::strerror(errno);
    return false;
  }
  return true;
}

bool UdpBridge::Receive(double timeout_s, Feedback& feedback) {
  last_error_.clear();
  bool received = false;
  pollfd pfd{fd_, POLLIN, 0};
  int timeout_ms = static_cast<int>(std::ceil(timeout_s * 1e3));
  // Drain the queue: wait for the first frame, then take whatever else is already there.
  while (poll(&pfd, 1, timeout_ms) > 0) {
    Feedback frame;
    sockaddr_in from{};
    socklen_t from_len = sizeof(from);
    const ssize_t n = recvfrom(fd_, &frame, sizeof(frame), 0, reinterpret_cast<sockaddr*>(&from), &from_len);
    if (n == static_cast<ssize_t>(sizeof(frame)) && frame.magic == kFeedbackMagic && frame.version == kVersion) {
      feedback = frame;
      peer_ = from;
      has_peer_ = true;
      received = true;
    } else if (n >= 0) {
      last_error_ = "dropped a malformed frame of " + std::to_string(n) + " bytes";
    }
    timeout_ms = 0;
  }
  if (!received) {
    last_error_ = "no feedback within " + std::to_string(timeout_s) + " s";
  }
  return received;
}

bool UdpBridge::Send(const Command& command) {
  if (!has_peer_) {
    last_error_ = "no feedback received yet, nowhere to send";
    return false;
  }
  const ssize_t n = sendto(fd_, &command, sizeof(command), 0, reinterpret_cast<const sockaddr*>(&peer_), sizeof(peer_));
  if (n != static_cast<ssize_t>(sizeof(command))) {
    last_error_ = std::string("sendto: ") + std::strerror(errno);
    return false;
  }
  return true;
}

}  // namespace s1
