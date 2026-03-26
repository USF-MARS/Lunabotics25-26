#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32.hpp" // Added for the actuator

class JoyToCmdVel : public rclcpp::Node {
public:
    JoyToCmdVel() : Node("joy_to_cmd_vel") {
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&JoyToCmdVel::joy_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        // New publisher for the lift
        lift_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/lift_actuator", 10);
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) const {
        // 1. Handle Drive (Twist)
        auto twist = geometry_msgs::msg::Twist();
        twist.linear.x = msg->axes[1] * 1.0; 
        twist.angular.z = msg->axes[3] * -1.0; // Inverting Z is common for natural steering
        publisher_->publish(twist);

        // 2. Handle Lift Logic
        auto lift_msg = std_msgs::msg::Int32();
        
        if (msg->buttons[5] == 1) {
            lift_msg.data = 1;  // Extend
        } 
        else if (msg->buttons[4] == 1) {
            lift_msg.data = -1; // Retract
        } 
        else {
            lift_msg.data = 0;  // Stop
        }
        
        lift_publisher_->publish(lift_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr lift_publisher_;
};