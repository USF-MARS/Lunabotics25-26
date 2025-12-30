#pragma once

#include <cmath>

namespace lunabot_hardware_interface {

struct DrivetrainCommand {
  float left_wheel_rad_s{0.0F};
  float right_wheel_rad_s{0.0F};
};

class DrivetrainInterface {
public:
  DrivetrainInterface(double wheel_radius, double wheel_base, double max_wheel_rad_s);

  DrivetrainCommand compute_command(double linear_x, double angular_z, bool enabled) const;

private:
  double wheel_radius_;
  double wheel_base_;
  double max_wheel_rad_s_;
};

}  // namespace lunabot_hardware_interface
