#include "kunwei_ft_driver/client.hpp"
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <system_error>
#include <arpa/inet.h>
#include <fcntl.h>
#include <limits>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

namespace kunwei_ft_driver {
namespace {
constexpr std::uint8_t start[]{0x48, 0xaa, 0x0d, 0x0a};
constexpr std::uint8_t stop[]{0x43, 0xaa, 0x0d, 0x0a};
void fail(const char* what) { throw std::system_error(errno, std::generic_category(), what); }
int wait_fd(int fd, short events, int timeout) {
  if (timeout < 0) throw std::invalid_argument("timeout must be nonnegative");
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout);
  for (;;) {
    pollfd p{fd, events, 0};
    const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(
      deadline - std::chrono::steady_clock::now()).count();
    const int result = ::poll(&p, 1, static_cast<int>(std::max<std::int64_t>(0, left)));
    if (result >= 0) return result;
    if (errno != EINTR) fail("poll");
  }
}
}
void Parser::feed(const std::uint8_t* data, std::size_t size, const Callback& callback) {
  static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559);
  for (std::size_t i = 0; i < size; ++i) {
    buffer_[used_++] = data[i];
    while (used_ && (buffer_[0] != 0x48 && buffer_[0] != 0x49)) {
      std::move(buffer_.begin()+1, buffer_.begin()+used_, buffer_.begin()); --used_;
    }
    if (used_ >= 2 && buffer_[1] != 0xaa) {
      std::move(buffer_.begin()+1, buffer_.begin()+used_, buffer_.begin()); --used_;
      // Reconsider remaining bytes on the next input byte.
      continue;
    }
    if (used_ != 28) continue;
    if (buffer_[26] != 13 || buffer_[27] != 10) {
      std::move(buffer_.begin()+1, buffer_.end(), buffer_.begin()); --used_;
      continue;
    }
    std::array<float, 6> values{};
    for (std::size_t axis = 0; axis < 6; ++axis) {
      const auto* b = buffer_.data()+2+4*axis;
      std::uint32_t bits = std::uint32_t(b[0]) | (std::uint32_t(b[1])<<8) |
                           (std::uint32_t(b[2])<<16) | (std::uint32_t(b[3])<<24);
      std::memcpy(&values[axis], &bits, 4);
    }
    used_ = 0;
    if (std::all_of(values.begin(), values.end(), [](float v){return std::isfinite(v);}))
      callback(values);
  }
}
Client::Client(double scale) : scale_(scale) {
  if (!std::isfinite(scale) || scale <= 0) throw std::invalid_argument("invalid unit scale");
}
Client::~Client() { close(); }
void Client::connect(const std::string& ipv4, std::uint16_t port, int timeout_ms) {
  if (timeout_ms < 0) throw std::invalid_argument("negative timeout");
  close();
  sockaddr_in address{};
  address.sin_family = AF_INET; address.sin_port = htons(port);
  if (::inet_pton(AF_INET, ipv4.c_str(), &address.sin_addr) != 1)
    throw std::invalid_argument("host must be numeric IPv4");
  fd_ = ::socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
  if (fd_ < 0) fail("socket");
  try {
    if (::connect(fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
      if (errno != EINPROGRESS) fail("connect");
      if (!wait_fd(fd_, POLLOUT, timeout_ms)) throw std::runtime_error("connect timeout");
      int error = 0; socklen_t size = sizeof(error);
      if (::getsockopt(fd_, SOL_SOCKET, SO_ERROR, &error, &size) < 0) fail("getsockopt");
      if (error) throw std::system_error(error, std::generic_category(), "connect");
    }
    std::size_t written = 0;
    const auto deadline = std::chrono::steady_clock::now()+std::chrono::milliseconds(timeout_ms);
    while (written < sizeof(start)) {
      const auto n = ::send(fd_, start+written, sizeof(start)-written, MSG_NOSIGNAL);
      if (n > 0) { written += static_cast<std::size_t>(n); continue; }
      if (n < 0 && errno == EINTR) continue;
      if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        auto left = std::chrono::duration_cast<std::chrono::milliseconds>(deadline-std::chrono::steady_clock::now()).count();
        if (left <= 0 || !wait_fd(fd_, POLLOUT, static_cast<int>(left)))
          throw std::runtime_error("start command timeout");
        continue;
      }
      fail("send start");
    }
    parser_.reset(); sequence_ = 0;
  } catch (...) { close(); throw; }
}
bool Client::read(const std::function<void(const Sample&)>& callback, int timeout_ms) {
  if (fd_ < 0) throw std::logic_error("client is disconnected");
  if (!wait_fd(fd_, POLLIN, timeout_ms)) return false;
  std::array<std::uint8_t, 8192> bytes{};
  const auto n = ::recv(fd_, bytes.data(), bytes.size(), 0);
  if (n < 0) {
    if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) return false;
    fail("recv");
  }
  if (!n) throw std::runtime_error("sensor closed connection");
  const auto received = std::chrono::steady_clock::now();
  parser_.feed(bytes.data(), static_cast<std::size_t>(n), [&](const auto& values) {
    Sample sample;
    sample.received_at = received; sample.sequence = ++sequence_;
    for (std::size_t i=0; i<6; ++i) sample.wrench[i] = double(values[i])*scale_;
    callback(sample);
  });
  return true;
}
void Client::close() noexcept {
  if (fd_ >= 0) {
    // Best effort: close never blocks waiting for a device response.
    ::send(fd_, stop, sizeof(stop), MSG_NOSIGNAL | MSG_DONTWAIT);
    ::close(fd_); fd_ = -1;
  }
  parser_.reset();
}
}
