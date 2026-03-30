// lunabot_arbitration — Arbitration node (see package README.md in this repo for the full topic map).
//
// Design intent (team / operator requirements):
// 1) Automation mode: When the operator enables automation (e.g. Menu button publishes /mode_switch true
//    via lunabot_teleop/joy_mode_switch), Nav2 (or similar) commands on /cmd_vel_nav are forwarded only
//    while auto is allowed (see can_auto_publish()).
// 2) Connection loss (Foxglove bridge down, /joy or teleop stream gone, etc.): If no valid command updates
//    the failsafe watchdog for failsafe_timeout seconds, publish zero Twist on /cmd_vel. Extend this
//    node if you also need zeros on /cmd_tilt_actuator and /cmd_lift_actuator on timeout.
// 3) Manual override during auto: Any message on /cmd_vel_teleop, /cmd_tilt_actuator_teleop, or
//    /cmd_lift_actuator_teleop refreshes last_teleop_time_; while override_duration has not elapsed,
//    autonomous inputs are not forwarded (operator "wins").
// 4) Emergency stop: Publish geometry_msgs/Twist on /cmd_STOP_teleop (e.g. from a Foxglove button or a
//    controller mapping). stop_callback() forwards to /cmd_STOP and forces zero velocity on /cmd_vel.
//    micro-ROS on the Teensy also watchdog-stops drive if /cmd_vel stops arriving (see lunabot_hardware).
//
// Note: Teleop nodes must publish to the *_teleop topics above, not directly to /cmd_vel, when this node
// is in the launch graph; otherwise arbitration is bypassed.

#include <chrono>
#include <memory>
#include <string>

// ROS 2 core library. This gives us the "Node", "Publisher", "Subscriber" functionality.
#include "rclcpp/rclcpp.hpp"

// Message types we need to talk to other nodes. 
// "Twist" = Velocity (Linear + Angular)
// "Float32" = Simple numbers (for actuator position/speed)
// "Bool" = True/False (for switches)
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"

// These make the code cleaner so we don't have to type "std::chrono::seconds" every time.
using namespace std::chrono_literals;
using std::placeholders::_1;

// We create a class that inherits from "rclcpp::Node". 
// This essentially says: "This class IS a ROS node."
class ArbitrationNode : public rclcpp::Node
{
public:
    // Constructor: This runs ONCE when the node starts up.
    // It sets up all the connections (publishers/subscribers).
    ArbitrationNode() : Node("arbitration_node")
    {
        // --- Parameters ---
        // We define settings here so you can change them in a config file later 
        // without recompiling the code.
        
        // "override_duration": If you move the joystick, how many seconds should we 
        // ignore the autonomous code for? (Safety buffer).
        this->declare_parameter("override_duration", 1.0);
        
        // "failsafe_timeout": If the robot hears NOTHING from any controller for 
        // this many seconds, it should stop.
        this->declare_parameter("failsafe_timeout", 0.5);

        // Convert these parameters into usable time variables
        override_duration_ = std::chrono::duration<double>(this->get_parameter("override_duration").as_double());
        failsafe_timeout_ = std::chrono::duration<double>(this->get_parameter("failsafe_timeout").as_double());

        // --- Publishers (Outputs to Teensy) ---
        // These are the "speaking" tubes. We speak to the topics the Teensy is listening to.
        pub_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        pub_tilt_ = this->create_publisher<std_msgs::msg::Float32>("/cmd_tilt_actuator", 10);
        pub_lift_ = this->create_publisher<std_msgs::msg::Float32>("/cmd_lift_actuator", 10);
        pub_stop_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_STOP", 10);

        // --- Subscribers (Inputs) ---
        // These are the "listening" tubes. When data arrives, it triggers a specific function (callback).
        
        // 1. Mode Switch (From Foxglove dashboard)
        // std::bind connects the incoming message to the 'mode_callback' function.
        sub_mode_ = this->create_subscription<std_msgs::msg::Bool>(
            "/mode_switch", 10, std::bind(&ArbitrationNode::mode_callback, this, _1));

        // 2. STOP (High Priority)
        // If this triggers, we stop everything immediately.
        sub_stop_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_STOP_teleop", 10, std::bind(&ArbitrationNode::stop_callback, this, _1));

        // 3. Teleop (Manual Joystick - Priority Override)
        // These topics end in '_teleop' so we know they come from the human controller.
        sub_teleop_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_teleop", 10, std::bind(&ArbitrationNode::teleop_vel_callback, this, _1));
        sub_teleop_tilt_ = this->create_subscription<std_msgs::msg::Float32>(
            "/cmd_tilt_actuator_teleop", 10, std::bind(&ArbitrationNode::teleop_tilt_callback, this, _1));
        sub_teleop_lift_ = this->create_subscription<std_msgs::msg::Float32>(
            "/cmd_lift_actuator_teleop", 10, std::bind(&ArbitrationNode::teleop_lift_callback, this, _1));

        // 4. Autonomous (Nav2 & Dig Logic)
        // These topics end in '_nav'. They only get passed through if Auto Mode is ON.
        sub_nav_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_nav", 10, std::bind(&ArbitrationNode::nav_vel_callback, this, _1));
        sub_nav_tilt_ = this->create_subscription<std_msgs::msg::Float32>(
            "/cmd_tilt_actuator_nav", 10, std::bind(&ArbitrationNode::nav_tilt_callback, this, _1));
        sub_nav_lift_ = this->create_subscription<std_msgs::msg::Float32>(
            "/cmd_lift_actuator_nav", 10, std::bind(&ArbitrationNode::nav_lift_callback, this, _1));

        // --- Failsafe Timer ---
        // This function 'failsafe_check' will run automatically 10 times a second (100ms).
        // It acts like a dead man's switch.
        timer_ = this->create_wall_timer(
            100ms, std::bind(&ArbitrationNode::failsafe_check, this));

        RCLCPP_INFO(this->get_logger(), "Arbitration Node Started. Default: MANUAL");
    }

private:
    // --- State Variables (Memory) ---
    bool auto_mode_enabled_ = false;  // Is the "Auto" switch flipped on Foxglove?
    rclcpp::Time last_teleop_time_;   // When did the human last touch the joystick?
    rclcpp::Time last_stop_time_;     // When was the Emergency Stop last pressed?
    rclcpp::Time last_input_time_;    // When did we receive ANY valid command?

    std::chrono::duration<double> override_duration_;
    std::chrono::duration<double> failsafe_timeout_;

    // --- Pointers for ROS Objects ---
    // In C++, we store these as "Shared Pointers" (SharedPtr) so memory is managed automatically.
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_tilt_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_lift_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_stop_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_mode_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_stop_;
    
    // Teleop Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_teleop_vel_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_teleop_tilt_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_teleop_lift_;

    // Nav Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_nav_vel_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_nav_tilt_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_nav_lift_;

    rclcpp::TimerBase::SharedPtr timer_;

    // =============================================================
    //       HELPER FUNCTIONS (The Decision Logic)
    // =============================================================
    
    // Check if we are currently in a "STOPPED" state.
    // Logic: If the STOP button was pressed less than 1 second ago, return True.
    bool is_stopped() {
        if (last_stop_time_.nanoseconds() == 0) return false; // Never stopped
        return (this->now() - last_stop_time_).seconds() < 1.0;
    }

    // Check if the human is overriding the robot.
    // Logic: If human moved joystick less than 1.0s ago, return True.
    bool is_override_active() {
        if (last_teleop_time_.nanoseconds() == 0) return false;
        return (this->now() - last_teleop_time_).seconds() < override_duration_.count();
    }

    // The Master Decision: Should we let the Autonomous code drive?
    bool can_auto_publish() {
        if (is_stopped()) return false;        // No, we are E-Stopped.
        if (is_override_active()) return false; // No, human is steering.
        return auto_mode_enabled_;             // Yes, if the switch is ON.
    }

    // Update the "watchdog" timer whenever we get a valid command.
    void update_failsafe() {
        last_input_time_ = this->now();
    }

    // =============================================================
    //       CALLBACKS (Reacting to incoming data)
    // =============================================================

    // Called when you toggle the switch in Foxglove
    void mode_callback(const std_msgs::msg::Bool & msg) {
        auto_mode_enabled_ = msg.data;
        // %s is a placeholder for strings. the ? : is a shorthand if/else statement.
        RCLCPP_INFO(this->get_logger(), "Mode switched to: %s", auto_mode_enabled_ ? "AUTONOMOUS" : "MANUAL");
    }

    // Called when STOP button is pressed
    void stop_callback(const geometry_msgs::msg::Twist & msg) {
        last_stop_time_ = this->now();
        pub_stop_->publish(msg); // Forward the stop command to Teensy
        
        // SAFETY CRITICAL: Explicitly send 0 velocity to wheels immediately.
        auto zero_twist = geometry_msgs::msg::Twist();
        pub_vel_->publish(zero_twist);
        
        RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP RECEIVED");
    }

    // --- TELEOP HANDLERS (Manual Control) ---
    // These always update the 'last_teleop_time_' to trigger the Override.
    
    void teleop_vel_callback(const geometry_msgs::msg::Twist & msg) {
        last_teleop_time_ = this->now(); // Reset override timer
        update_failsafe();               // Pet the watchdog
        if (is_stopped()) return;        // Ignore if E-Stop is active
        pub_vel_->publish(msg);          // Send to Teensy
    }

    void teleop_tilt_callback(const std_msgs::msg::Float32 & msg) {
        last_teleop_time_ = this->now();
        update_failsafe();
        if (is_stopped()) return;
        pub_tilt_->publish(msg);
    }

    void teleop_lift_callback(const std_msgs::msg::Float32 & msg) {
        last_teleop_time_ = this->now();
        update_failsafe();
        if (is_stopped()) return;
        pub_lift_->publish(msg);
    }

    // --- AUTO HANDLERS (Navigation/Digging) ---
    // These check 'can_auto_publish()' before doing anything.

    void nav_vel_callback(const geometry_msgs::msg::Twist & msg) {
        if (can_auto_publish()) {
            update_failsafe(); // Pet the watchdog because Auto is driving correctly
            pub_vel_->publish(msg);
        }
    }

    void nav_tilt_callback(const std_msgs::msg::Float32 & msg) {
        if (can_auto_publish()) {
            update_failsafe();
            pub_tilt_->publish(msg);
        }
    }

    void nav_lift_callback(const std_msgs::msg::Float32 & msg) {
        if (can_auto_publish()) {
            update_failsafe();
            pub_lift_->publish(msg);
        }
    }

    // --- FAILSAFE LOOP ---
    // Runs 10 times a second
    void failsafe_check() {
        if (last_input_time_.nanoseconds() == 0) return; // System just started, give it a moment.
        
        // Calculate how long it has been since the last valid command
        double time_diff = (this->now() - last_input_time_).seconds();
        
        // If it's been too long (e.g. > 0.5s), assume connection lost
        if (time_diff > failsafe_timeout_.count()) {
            // Publish ZEROs to stop the robot from "ghost riding"
            auto zero_twist = geometry_msgs::msg::Twist();
            pub_vel_->publish(zero_twist);
        }
    }
};

// Standard C++ main function to start the node
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv); // Initialize ROS 2
    rclcpp::spin(std::make_shared<ArbitrationNode>()); // Keep node alive
    rclcpp::shutdown(); // Clean up on exit
    return 0;
}