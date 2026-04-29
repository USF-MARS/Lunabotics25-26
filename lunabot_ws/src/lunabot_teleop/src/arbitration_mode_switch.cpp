#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"

class ArbitrationModeSwitch : public rclcpp::Node {
public:
  ArbitrationModeSwitch() : Node("arbitration_mode_switch") {
    // Defaulting to index 8 as requested
    this->declare_parameter("menu_button_index", 8);
    menu_button_index_ = static_cast<size_t>(this->get_parameter("menu_button_index").as_int());

    pub_ = this->create_publisher<std_msgs::msg::Bool>("/mode_switch", 10);
    sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&ArbitrationModeSwitch::joy_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Arbitration Switch set to button index: %zu", menu_button_index_);
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    if (menu_button_index_ >= msg->buttons.size()) return;

    const bool pressed = msg->buttons[menu_button_index_] != 0;

    // Detect Rising Edge (Initial press only)
    if (pressed && !prev_pressed_) {
      auto_mode_ = !auto_mode_; 
      auto out = std_msgs::msg::Bool();
      out.data = auto_mode_;
      pub_->publish(out);
      
      RCLCPP_INFO(this->get_logger(), "Mode Toggled via Button 8: %s", 
                  auto_mode_ ? "AUTONOMOUS" : "MANUAL");
    }
    prev_pressed_ = pressed;
  }

  size_t menu_button_index_;
  bool prev_pressed_{false};
  bool auto_mode_{false};
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArbitrationModeSwitch>());
  rclcpp::shutdown();
  return 0;
}