#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"

// joy_mode_switch — publishes std_msgs/Bool on /mode_switch for lunabot_arbitration.
//
// Team instructions (how this fits the stack):
// - Automation: Operator presses a button such as Menu; each rising edge toggles /mode_switch so the
//   rover can enter or leave automation (arbitration uses this with /cmd_vel_nav when safe).
// - Emergency stop and "send zeros": That path is NOT this topic. Arbitration listens for
//   geometry_msgs/Twist on /cmd_STOP_teleop (e.g. bind a Foxglove button, or a separate controller
//   button, to publish there). To map the Menu button to zeros as well, use another node or Foxglove
//   to publish to /cmd_STOP_teleop when appropriate; do not overload /mode_switch for E-stop.
// - Lost Foxglove / micro-ROS: Arbitration failsafe and Teensy watchdog stop the robot if commands
//   stop; see lunabot_arbitration and lunabot_hardware rover firmware.
// - Manual override while auto: Handled inside arbitration when teleop uses *_teleop topics.
//
// Button index is controller-specific; use `ros2 topic echo /joy` while pressing Menu, then set
// `menu_button_index` if the default does not match your gamepad.
class JoyModeSwitch : public rclcpp::Node {
public:
  JoyModeSwitch() : Node("joy_mode_switch") {
    this->declare_parameter("menu_button_index", 7);
    menu_button_index_ = static_cast<size_t>(this->get_parameter("menu_button_index").as_int());

    pub_ = this->create_publisher<std_msgs::msg::Bool>("/mode_switch", 10);
    sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&JoyModeSwitch::joy_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "joy_mode_switch: toggle Menu -> /mode_switch (menu_button_index=%zu)",
      menu_button_index_);
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    if (menu_button_index_ >= msg->buttons.size()) {
      if (!warned_oversize_) {
        RCLCPP_WARN(
          this->get_logger(),
          "/joy has %zu buttons; menu_button_index=%zu is invalid. "
          "Set param menu_button_index to a valid index.",
          msg->buttons.size(), menu_button_index_);
        warned_oversize_ = true;
      }
      return;
    }

    const bool pressed = msg->buttons[menu_button_index_] != 0;

    if (pressed && !prev_pressed_) {
      auto_mode_ = !auto_mode_;
      auto out = std_msgs::msg::Bool();
      out.data = auto_mode_;
      pub_->publish(out);
      RCLCPP_INFO(
        this->get_logger(), "Published /mode_switch: %s",
        auto_mode_ ? "AUTONOMOUS (true)" : "MANUAL (false)");
    }
    prev_pressed_ = pressed;
  }

  size_t menu_button_index_;
  bool prev_pressed_{false};
  bool auto_mode_{false};
  bool warned_oversize_{false};

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyModeSwitch>());
  rclcpp::shutdown();
  return 0;
}
