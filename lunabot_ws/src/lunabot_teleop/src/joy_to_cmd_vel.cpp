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
        auto twist = geometry_msgs::msg::Twist();
        auto arm_msg = std_msgs::msg::Int32();
        auto bucket_msg = std_msgs::msg::Int32();

        // --- STOP BUTTON LOGIC (Index 8) ---
        // Safety: If index 8 is pressed, force stop everything and skip processing
        if (msg->buttons.size() > 8 && msg->buttons[8] == 1) {
            drive_publisher_->publish(twist); // Sends zeros
            arm_msg.data = 0;
            arm_publisher_->publish(arm_msg);
            bucket_msg.data = 0;
            bucket_publisher_->publish(bucket_msg);
            return; 
        }

        // --- Drive Logic ---
        double linear_val = msg->axes[1];
        double angular_val = msg->axes[3];
        twist.linear.x = (std::abs(linear_val) > 0.05) ? linear_val * 1.0 : 0.0;
        twist.angular.z = (std::abs(angular_val) > 0.05) ? angular_val * -1.0 : 0.0;
        drive_publisher_->publish(twist);

        // --- Arm Logic (Buttons 5/4) ---
        if (msg->buttons[5] == 1) arm_msg.data = 1;
        else if (msg->buttons[4] == 1) arm_msg.data = -1;
        else arm_msg.data = 0;
        arm_publisher_->publish(arm_msg);

        // --- Bucket Logic (Buttons 0/1) ---
        if (msg->buttons[0] == 1) bucket_msg.data = 1;
        else if (msg->buttons[1] == 1) bucket_msg.data = -1;
        else bucket_msg.data = 0;
        bucket_publisher_->publish(bucket_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr drive_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr arm_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr bucket_publisher_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyToCmdVel>());
  rclcpp::shutdown();
  return 0;
}