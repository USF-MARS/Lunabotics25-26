#include "lunabot_hardware_interface/actuator_interface.hpp"

#include <algorithm>

namespace lunabot_hardware_interface {

ActuatorInterface::ActuatorInterface(uint16_t adc_min, uint16_t adc_max)
: adc_min_(adc_min), adc_max_(adc_max) {}

ActuatorState ActuatorInterface::from_feedback(uint16_t lift_adc, uint16_t blade_adc) const {
  ActuatorState state;
  if (adc_max_ <= adc_min_) {
    return state;
  }

  auto normalize = [this](uint16_t adc) {
    double clamped = std::clamp(static_cast<double>(adc), static_cast<double>(adc_min_),
                                static_cast<double>(adc_max_));
    return static_cast<float>((clamped - adc_min_) / static_cast<double>(adc_max_ - adc_min_));
  };

  state.lift_position = normalize(lift_adc);
  state.blade_position = normalize(blade_adc);
  return state;
}

float ActuatorInterface::clamp_target(float target) const {
  return std::clamp(target, 0.0F, 1.0F);
}

}  // namespace lunabot_hardware_interface
