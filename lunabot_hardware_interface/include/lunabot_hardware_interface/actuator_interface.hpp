#pragma once

#include <cstdint>

namespace lunabot_hardware_interface {

struct ActuatorState {
  float lift_position{0.0F};
  float blade_position{0.0F};
};

class ActuatorInterface {
public:
  ActuatorInterface(uint16_t adc_min, uint16_t adc_max);

  ActuatorState from_feedback(uint16_t lift_adc, uint16_t blade_adc) const;
  float clamp_target(float target) const;

private:
  uint16_t adc_min_;
  uint16_t adc_max_;
};

}  // namespace lunabot_hardware_interface
