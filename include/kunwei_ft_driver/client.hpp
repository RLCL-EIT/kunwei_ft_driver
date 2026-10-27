#pragma once
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>

namespace kunwei_ft_driver {
struct Sample {
  // Fx,Fy,Fz [N], Tx,Ty,Tz [N m]; native sensor frame, no compensation.
  std::array<double, 6> wrench{};
  std::chrono::steady_clock::time_point received_at{};
  std::uint64_t sequence{};  // Local accepted-frame count, NOT a device counter.
};

class Parser {
 public:
  using Callback = std::function<void(const std::array<float, 6>&)>;
  void feed(const std::uint8_t* data, std::size_t size, const Callback& callback);
  void reset() noexcept { used_ = 0; }
 private:
  std::array<std::uint8_t, 28> buffer_{};
  std::size_t used_{};
};

// Linux/POSIX TCP client. Single-owner API; never call concurrently.
// All network calls belong on a non-real-time acquisition thread.
class Client {
 public:
  explicit Client(double unit_scale = 9.80665);
  ~Client();
  Client(const Client&) = delete;
  Client& operator=(const Client&) = delete;
  void connect(const std::string& ipv4 = "192.168.1.88", std::uint16_t port = 5152,
               int timeout_ms = 2000);
  // Calls callback once per valid frame in a receive batch. False means poll timeout.
  // Throws on disconnect/network error. Timeout is not itself a valid-frame watchdog.
  bool read(const std::function<void(const Sample&)>& callback, int timeout_ms = 100);
  void close() noexcept;
 private:
  int fd_{-1};
  double scale_;
  Parser parser_;
  std::uint64_t sequence_{};
};
}  // namespace kunwei_ft_driver
