#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32.hpp"

class JoyToCmdVel : public rclcpp::Node {
public:
    JoyToCmdVel() : Node("joy_to_cmd_vel") {
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&JoyToCmdVel::joy_callback, this, std::placeholders::_1));

        drive_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        arm_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/arm_actuator", 10);
        bucket_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/bucket_actuator", 10);
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) const {
        // --- 1. Handle Drive (Twist) ---
        auto twist = geometry_msgs::msg::Twist();
        
        // Basic deadzone logic (0.05) to prevent drifting
        double linear_val = msg->axes[1];
        double angular_val = msg->axes[3];

        twist.linear.x = (std::abs(linear_val) > 0.05) ? linear_val * 1.0 : 0.0;
        twist.angular.z = (std::abs(angular_val) > 0.05) ? angular_val * -1.0 : 0.0;
        
        drive_publisher_->publish(twist);

        // --- 2. Handle Arm Logic ---
        auto arm_msg = std_msgs::msg::Int32();
        
        // Using Buttons 5 (R1) and 4 (L1) as per your original code
        if (msg->buttons[5] == 1) {
            arm_msg.data = 1;  // Extend
        } 
        else if (msg->buttons[4] == 1) {
            arm_msg.data = -1; // Retract
        } 
        else {
            arm_msg.data = 0;  // Stop
        }
        arm_publisher_->publish(arm_msg);

        // --- 3. Handle Bucket Logic ---
        auto bucket_msg = std_msgs::msg::Int32();

        // Added Bucket Logic: Using Buttons 0 and 1 as placeholders
        // Change these indices once you know which buttons you prefer!
        if (msg->buttons[0] == 1) {
            bucket_msg.data = 1;  // Extend/Dump
        }
        else if (msg->buttons[1] == 1) {
            bucket_msg.data = -1; // Retract/Tilt
        }
        else {
            bucket_msg.data = 0;  // Stop
        }
        bucket_publisher_->publish(bucket_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr drive_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr arm_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr bucket_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyToCmdVel>());
  rclcpp::shutdown();
  return 0;
}