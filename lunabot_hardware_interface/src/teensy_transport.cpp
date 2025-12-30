#include "lunabot_hardware_interface/teensy_transport.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace lunabot_hardware_interface {

namespace {
constexpr uint8_t kCommandHeader = 0xAA;
constexpr uint8_t kFeedbackHeader = 0x55;
constexpr size_t kCommandPacketSize = 1 + 4 * sizeof(float) + 1;
constexpr size_t kFeedbackPacketSize = 1 + 4 * sizeof(int32_t) + 2 * sizeof(uint16_t) + 1 + 1;
}  // namespace

TeensyTransport::TeensyTransport()
: fd_(-1), running_(false), logger_(rclcpp::get_logger("teensy_transport")) {}

TeensyTransport::~TeensyTransport() {
  close();
}

bool TeensyTransport::open(const std::string & port, int baudrate, rclcpp::Logger logger) {
  logger_ = logger;
  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    RCLCPP_ERROR(logger_, "Failed to open serial port %s: %s", port.c_str(), std::strerror(errno));
    return false;
  }

  if (!configure_port(fd_, baudrate)) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  running_.store(true);
  read_thread_ = std::thread(&TeensyTransport::read_loop, this);
  RCLCPP_INFO(logger_, "Opened serial port %s at %d baud", port.c_str(), baudrate);
  return true;
}

void TeensyTransport::close() {
  running_.store(false);
  if (read_thread_.joinable()) {
    read_thread_.join();
  }
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool TeensyTransport::is_open() const {
  return fd_ >= 0;
}

bool TeensyTransport::write_command(const CommandPacket & command) {
  if (fd_ < 0) {
    return false;
  }

  std::array<uint8_t, kCommandPacketSize> buffer{};
  buffer[0] = kCommandHeader;

  std::memcpy(buffer.data() + 1, &command.left_wheel_rad_s, sizeof(float));
  std::memcpy(buffer.data() + 1 + sizeof(float), &command.right_wheel_rad_s, sizeof(float));
  std::memcpy(buffer.data() + 1 + 2 * sizeof(float), &command.lift_target, sizeof(float));
  std::memcpy(buffer.data() + 1 + 3 * sizeof(float), &command.blade_target, sizeof(float));

  buffer[kCommandPacketSize - 1] = checksum(buffer.data(), kCommandPacketSize - 1);

  ssize_t written = ::write(fd_, buffer.data(), buffer.size());
  if (written != static_cast<ssize_t>(buffer.size())) {
    RCLCPP_WARN(logger_, "Partial serial write: %zd/%zu", written, buffer.size());
    return false;
  }
  return true;
}

std::optional<FeedbackPacket> TeensyTransport::latest_feedback() const {
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  return last_feedback_;
}

rclcpp::Time TeensyTransport::last_feedback_time() const {
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  if (!last_feedback_) {
    return rclcpp::Time(0, 0, RCL_ROS_TIME);
  }
  return last_feedback_->stamp;
}

void TeensyTransport::read_loop() {
  std::array<uint8_t, 256> buffer{};
  while (running_.load()) {
    if (fd_ < 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      continue;
    }

    ssize_t count = ::read(fd_, buffer.data(), buffer.size());
    if (count > 0) {
      process_bytes(buffer.data(), static_cast<size_t>(count));
    } else {
      if (errno != EAGAIN && errno != EWOULDBLOCK) {
        RCLCPP_WARN(logger_, "Serial read error: %s", std::strerror(errno));
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }
}

bool TeensyTransport::configure_port(int fd, int baudrate) {
  termios tty;
  if (tcgetattr(fd, &tty) != 0) {
    RCLCPP_ERROR(logger_, "Failed to get serial attributes: %s", std::strerror(errno));
    return false;
  }

  cfmakeraw(&tty);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 1;

  speed_t speed = B115200;
  if (baudrate == 9600) {
    speed = B9600;
  } else if (baudrate == 19200) {
    speed = B19200;
  } else if (baudrate == 38400) {
    speed = B38400;
  } else if (baudrate == 57600) {
    speed = B57600;
  } else if (baudrate == 115200) {
    speed = B115200;
  }

  if (cfsetispeed(&tty, speed) != 0 || cfsetospeed(&tty, speed) != 0) {
    RCLCPP_ERROR(logger_, "Failed to set serial speed: %s", std::strerror(errno));
    return false;
  }

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(logger_, "Failed to set serial attributes: %s", std::strerror(errno));
    return false;
  }

  return true;
}

uint8_t TeensyTransport::checksum(const uint8_t * data, size_t length) {
  uint32_t sum = 0;
  for (size_t i = 0; i < length; ++i) {
    sum += data[i];
  }
  return static_cast<uint8_t>(sum & 0xFF);
}

void TeensyTransport::process_bytes(const uint8_t * data, size_t length) {
  parse_buffer_.insert(parse_buffer_.end(), data, data + length);

  while (parse_buffer_.size() >= kFeedbackPacketSize) {
    auto header_pos = std::find(parse_buffer_.begin(), parse_buffer_.end(), kFeedbackHeader);
    if (header_pos == parse_buffer_.end()) {
      parse_buffer_.clear();
      return;
    }

    size_t offset = static_cast<size_t>(std::distance(parse_buffer_.begin(), header_pos));
    if (parse_buffer_.size() - offset < kFeedbackPacketSize) {
      if (offset > 0) {
        parse_buffer_.erase(parse_buffer_.begin(), parse_buffer_.begin() + static_cast<long>(offset));
      }
      return;
    }

    std::vector<uint8_t> packet(parse_buffer_.begin() + static_cast<long>(offset),
                                parse_buffer_.begin() + static_cast<long>(offset + kFeedbackPacketSize));

    FeedbackPacket feedback;
    if (parse_feedback_packet(packet, feedback)) {
      std::lock_guard<std::mutex> lock(feedback_mutex_);
      last_feedback_ = feedback;
    }

    parse_buffer_.erase(parse_buffer_.begin(),
                        parse_buffer_.begin() + static_cast<long>(offset + kFeedbackPacketSize));
  }
}

bool TeensyTransport::parse_feedback_packet(const std::vector<uint8_t> & buffer, FeedbackPacket & out) const {
  if (buffer.size() != kFeedbackPacketSize || buffer[0] != kFeedbackHeader) {
    return false;
  }

  uint8_t expected = checksum(buffer.data(), buffer.size() - 1);
  if (expected != buffer.back()) {
    RCLCPP_WARN_THROTTLE(logger_, rclcpp::Clock(RCL_ROS_TIME), 2000,
                         "Checksum mismatch on feedback packet");
    return false;
  }

  size_t offset = 1;
  std::memcpy(&out.fl_ticks, buffer.data() + offset, sizeof(int32_t));
  offset += sizeof(int32_t);
  std::memcpy(&out.fr_ticks, buffer.data() + offset, sizeof(int32_t));
  offset += sizeof(int32_t);
  std::memcpy(&out.rl_ticks, buffer.data() + offset, sizeof(int32_t));
  offset += sizeof(int32_t);
  std::memcpy(&out.rr_ticks, buffer.data() + offset, sizeof(int32_t));
  offset += sizeof(int32_t);
  std::memcpy(&out.lift_adc, buffer.data() + offset, sizeof(uint16_t));
  offset += sizeof(uint16_t);
  std::memcpy(&out.blade_adc, buffer.data() + offset, sizeof(uint16_t));
  offset += sizeof(uint16_t);
  std::memcpy(&out.fault_flags, buffer.data() + offset, sizeof(uint8_t));

  out.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  return true;
}

}  // namespace lunabot_hardware_interface
