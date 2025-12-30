#include "lunabot_hardware_interface/drivetrain_interface.hpp"

#include <algorithm>

namespace lunabot_hardware_interface {

DrivetrainInterface::DrivetrainInterface(double wheel_radius, double wheel_base, double max_wheel_rad_s)
: wheel_radius_(wheel_radius), wheel_base_(wheel_base), max_wheel_rad_s_(max_wheel_rad_s) {}

DrivetrainCommand DrivetrainInterface::compute_command(double linear_x, double angular_z, bool enabled) const {
  DrivetrainCommand command;
  if (!enabled || wheel_radius_ <= 0.0) {
    return command;
  }

  double v_left = linear_x - angular_z * wheel_base_ / 2.0;
  double v_right = linear_x + angular_z * wheel_base_ / 2.0;

  command.left_wheel_rad_s = static_cast<float>(v_left / wheel_radius_);
  command.right_wheel_rad_s = static_cast<float>(v_right / wheel_radius_);

  if (std::isfinite(max_wheel_rad_s_) && max_wheel_rad_s_ > 0.0) {
    command.left_wheel_rad_s = static_cast<float>(std::clamp(
      static_cast<double>(command.left_wheel_rad_s), -max_wheel_rad_s_, max_wheel_rad_s_));
    command.right_wheel_rad_s = static_cast<float>(std::clamp(
      static_cast<double>(command.right_wheel_rad_s), -max_wheel_rad_s_, max_wheel_rad_s_));
  }

  return command;
}

}  // namespace lunabot_hardware_interface
