#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace lunabot_hardware_interface {

struct CommandPacket {
  float left_wheel_rad_s{0.0F};
  float right_wheel_rad_s{0.0F};
  float lift_target{0.0F};
  float blade_target{0.0F};
};

struct FeedbackPacket {
  int32_t fl_ticks{0};
  int32_t fr_ticks{0};
  int32_t rl_ticks{0};
  int32_t rr_ticks{0};
  uint16_t lift_adc{0};
  uint16_t blade_adc{0};
  uint8_t fault_flags{0};
  rclcpp::Time stamp;
};

class TeensyTransport {
public:
  TeensyTransport();
  ~TeensyTransport();

  bool open(const std::string & port, int baudrate, rclcpp::Logger logger);
  void close();

  bool is_open() const;
  bool write_command(const CommandPacket & command);

  std::optional<FeedbackPacket> latest_feedback() const;
  rclcpp::Time last_feedback_time() const;

private:
  void read_loop();
  bool configure_port(int fd, int baudrate);
  static uint8_t checksum(const uint8_t * data, size_t length);
  void process_bytes(const uint8_t * data, size_t length);
  bool parse_feedback_packet(const std::vector<uint8_t> & buffer, FeedbackPacket & out) const;

  int fd_;
  std::atomic<bool> running_;
  std::thread read_thread_;
  rclcpp::Logger logger_;

  mutable std::mutex feedback_mutex_;
  std::optional<FeedbackPacket> last_feedback_;

  std::vector<uint8_t> parse_buffer_;
};

}  // namespace lunabot_hardware_interface
