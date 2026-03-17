#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

class JoyToCmdVel : public rclcpp::Node {
public:
    JoyToCmdVel() : Node("joy_to_cmd_vel") {
        // Subscribe to /joy (from Foxglove)
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&JoyToCmdVel::joy_callback, this, std::placeholders::_1));

        // Publish to /cmd_vel (to Teensy)
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) const {
        auto twist = geometry_msgs::msg::Twist();

        // Xbox Left Stick Up/Down is usually index 1
        // We multiply by a scale factor (e.g., 1.0)
        twist.linear.x = msg->axes[1] * 1.0; 
        
        // Right Stick Left/Right is usually index 3 (for future turning)
        twist.angular.z = msg->axes[3] * 1.0;

        publisher_->publish(twist);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyToCmdVel>());
    rclcpp::shutdown();
    return 0;
}