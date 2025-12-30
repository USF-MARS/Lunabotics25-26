// Assumptions:\n+// - Teensy and host use little-endian float/int32 encoding for packet fields.\n+// - Checksum is the 8-bit sum of all bytes excluding the checksum byte.\n+// - Encoder tick counts are cumulative and increasing in forward direction.\n+// - Wheel radius and wheel base are provided via parameters and remain constant at runtime.\n+// - Actuator ADC min/max map linearly to normalized position [0.0, 1.0].\n+#include \"lunabot_hardware_interface/actuator_interface.hpp\"
#include "lunabot_hardware_interface/drivetrain_interface.hpp"
#include "lunabot_hardware_interface/teensy_transport.hpp"

#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

namespace lunabot_hardware_interface {

class HardwareInterfaceNode : public rclcpp::Node {
public:
  HardwareInterfaceNode()
  : Node("lunabot_hardware_interface"),
    wheel_radius_(declare_parameter("wheel_radius", 0.1524)),
    wheel_base_(declare_parameter("wheel_base", 0.6)),
    max_wheel_rad_s_(declare_parameter("max_wheel_rad_s", 20.0)),
    drivetrain_(wheel_radius_, wheel_base_, max_wheel_rad_s_),
    actuators_(declare_parameter("adc_min", 0),
               declare_parameter("adc_max", 4095)) {
    serial_port_ = declare_parameter("serial_port", std::string("/dev/ttyACM0"));
    baudrate_ = declare_parameter("baudrate", 115200);
    ticks_per_rev_ = declare_parameter("ticks_per_revolution", 2048.0);

    cmd_vel_timeout_ = declare_parameter("cmd_vel_timeout", 0.25);
    actuator_timeout_ = declare_parameter("actuator_timeout", 0.5);
    feedback_timeout_ = declare_parameter("feedback_timeout", 0.5);

    odom_frame_id_ = declare_parameter("odom_frame_id", std::string("odom"));
    base_frame_id_ = declare_parameter("base_frame_id", std::string("base_link"));

    last_cmd_vel_time_ = now();
    last_actuator_time_ = now();

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        last_cmd_vel_time_ = now();
        last_cmd_vel_ = *msg;
      });

    actuator_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/actuator_cmds", 10,
      [this](std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 2) {
          last_actuator_time_ = now();
          lift_target_ = actuators_.clamp_target(msg->data[0]);
          blade_target_ = actuators_.clamp_target(msg->data[1]);
        } else {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                               "Actuator command array too small; expected 2 entries");
        }
      });

    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/hardware_status", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    if (!transport_.open(serial_port_, baudrate_, get_logger())) {
      RCLCPP_ERROR(get_logger(), "Failed to open Teensy serial port; node will retry in timer");
    }

    timer_ = create_wall_timer(std::chrono::milliseconds(20), std::bind(&HardwareInterfaceNode::on_timer, this));
  }

private:
  void on_timer() {
    if (!transport_.is_open()) {
      transport_.open(serial_port_, baudrate_, get_logger());
    }

    rclcpp::Time current_time = now();
    bool cmd_timeout = (current_time - last_cmd_vel_time_).seconds() > cmd_vel_timeout_;
    bool actuator_timeout = (current_time - last_actuator_time_).seconds() > actuator_timeout_;

    auto feedback_opt = transport_.latest_feedback();
    bool feedback_ok = false;
    uint8_t fault_flags = 0;
    if (feedback_opt) {
      feedback_ok = (current_time - feedback_opt->stamp).seconds() <= feedback_timeout_;
      fault_flags = feedback_opt->fault_flags;
    }

    bool enabled = feedback_ok && fault_flags == 0;

    geometry_msgs::msg::Twist cmd_vel = last_cmd_vel_;
    if (cmd_timeout || !enabled) {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;
    }

    float lift_target = lift_target_;
    float blade_target = blade_target_;
    if (actuator_timeout) {
      lift_target = lift_target_;
      blade_target = blade_target_;
    }
    if (!enabled) {
      lift_target = lift_target_;
      blade_target = blade_target_;
    }

    DrivetrainCommand drive_cmd = drivetrain_.compute_command(cmd_vel.linear.x, cmd_vel.angular.z, enabled);

    CommandPacket packet;
    packet.left_wheel_rad_s = drive_cmd.left_wheel_rad_s;
    packet.right_wheel_rad_s = drive_cmd.right_wheel_rad_s;
    packet.lift_target = actuators_.clamp_target(lift_target);
    packet.blade_target = actuators_.clamp_target(blade_target);
    transport_.write_command(packet);

    if (feedback_opt && feedback_opt->stamp > last_feedback_time_) {
      handle_feedback(*feedback_opt);
      last_feedback_time_ = feedback_opt->stamp;
    }

    publish_diagnostics(current_time, feedback_ok, fault_flags, enabled, cmd_timeout, actuator_timeout);
  }

  void handle_feedback(const FeedbackPacket & feedback) {
    double rad_per_tick = 2.0 * M_PI / ticks_per_rev_;

    if (!have_ticks_) {
      last_fl_ticks_ = feedback.fl_ticks;
      last_fr_ticks_ = feedback.fr_ticks;
      last_rl_ticks_ = feedback.rl_ticks;
      last_rr_ticks_ = feedback.rr_ticks;
      last_odom_time_ = feedback.stamp;
      have_ticks_ = true;
      publish_joint_states(feedback, rad_per_tick, 0.0, 0, 0, 0, 0);
      return;
    }

    int32_t d_fl = feedback.fl_ticks - last_fl_ticks_;
    int32_t d_fr = feedback.fr_ticks - last_fr_ticks_;
    int32_t d_rl = feedback.rl_ticks - last_rl_ticks_;
    int32_t d_rr = feedback.rr_ticks - last_rr_ticks_;

    double dt = (feedback.stamp - last_odom_time_).seconds();
    if (dt <= 0.0) {
      last_fl_ticks_ = feedback.fl_ticks;
      last_fr_ticks_ = feedback.fr_ticks;
      last_rl_ticks_ = feedback.rl_ticks;
      last_rr_ticks_ = feedback.rr_ticks;
      last_odom_time_ = feedback.stamp;
      return;
    }

    double left_ticks = 0.5 * (static_cast<double>(d_fl) + static_cast<double>(d_rl));
    double right_ticks = 0.5 * (static_cast<double>(d_fr) + static_cast<double>(d_rr));

    double left_dist = left_ticks * rad_per_tick * wheel_radius_;
    double right_dist = right_ticks * rad_per_tick * wheel_radius_;

    double delta_s = 0.5 * (left_dist + right_dist);
    double delta_theta = (right_dist - left_dist) / wheel_base_;

    double mid_theta = theta_ + delta_theta * 0.5;
    x_ += delta_s * std::cos(mid_theta);
    y_ += delta_s * std::sin(mid_theta);
    theta_ = normalize_angle(theta_ + delta_theta);

    publish_joint_states(feedback, rad_per_tick, dt, d_fl, d_fr, d_rl, d_rr);
    publish_odometry(feedback.stamp, left_dist / dt, right_dist / dt);

    last_fl_ticks_ = feedback.fl_ticks;
    last_fr_ticks_ = feedback.fr_ticks;
    last_rl_ticks_ = feedback.rl_ticks;
    last_rr_ticks_ = feedback.rr_ticks;
    last_odom_time_ = feedback.stamp;
  }

  void publish_joint_states(const FeedbackPacket & feedback, double rad_per_tick, double dt,
                            int32_t d_fl, int32_t d_fr, int32_t d_rl, int32_t d_rr) {
    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = feedback.stamp;
    joint_state.name = {
      "front_left_wheel",
      "front_right_wheel",
      "rear_left_wheel",
      "rear_right_wheel",
      "lift_actuator",
      "blade_actuator"
    };

    joint_state.position.resize(6);
    joint_state.velocity.resize(4);

    joint_state.position[0] = feedback.fl_ticks * rad_per_tick;
    joint_state.position[1] = feedback.fr_ticks * rad_per_tick;
    joint_state.position[2] = feedback.rl_ticks * rad_per_tick;
    joint_state.position[3] = feedback.rr_ticks * rad_per_tick;

    ActuatorState actuator_state = actuators_.from_feedback(feedback.lift_adc, feedback.blade_adc);
    joint_state.position[4] = actuator_state.lift_position;
    joint_state.position[5] = actuator_state.blade_position;

    if (dt > 0.0) {
      joint_state.velocity[0] = d_fl * rad_per_tick / dt;
      joint_state.velocity[1] = d_fr * rad_per_tick / dt;
      joint_state.velocity[2] = d_rl * rad_per_tick / dt;
      joint_state.velocity[3] = d_rr * rad_per_tick / dt;
    } else {
      joint_state.velocity[0] = 0.0;
      joint_state.velocity[1] = 0.0;
      joint_state.velocity[2] = 0.0;
      joint_state.velocity[3] = 0.0;
    }

    joint_state_pub_->publish(joint_state);
  }

  void publish_odometry(const rclcpp::Time & stamp, double left_vel, double right_vel) {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_id_;
    odom.child_frame_id = base_frame_id_;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.twist.twist.linear.x = 0.5 * (left_vel + right_vel);
    odom.twist.twist.angular.z = (right_vel - left_vel) / wheel_base_;

    odom_pub_->publish(odom);

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = odom_frame_id_;
    tf_msg.child_frame_id = base_frame_id_;
    tf_msg.transform.translation.x = x_;
    tf_msg.transform.translation.y = y_;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf_msg);
  }

  void publish_diagnostics(const rclcpp::Time & stamp, bool feedback_ok, uint8_t fault_flags,
                           bool enabled, bool cmd_timeout, bool actuator_timeout) {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = stamp;

    diagnostic_msgs::msg::DiagnosticStatus serial_status;
    serial_status.name = "serial_link";
    serial_status.hardware_id = serial_port_;
    serial_status.level = feedback_ok ? diagnostic_msgs::msg::DiagnosticStatus::OK
                                      : diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    serial_status.message = feedback_ok ? "Feedback streaming" : "Feedback timeout";
    serial_status.values.push_back(key_value("feedback_timeout", std::to_string(feedback_timeout_)));

    diagnostic_msgs::msg::DiagnosticStatus motor_status;
    motor_status.name = "motor_enable";
    motor_status.hardware_id = serial_port_;
    motor_status.level = enabled ? diagnostic_msgs::msg::DiagnosticStatus::OK
                                 : diagnostic_msgs::msg::DiagnosticStatus::WARN;
    motor_status.message = enabled ? "Outputs enabled" : "Outputs disabled";
    motor_status.values.push_back(key_value("cmd_vel_timeout", std::to_string(cmd_timeout)));

    diagnostic_msgs::msg::DiagnosticStatus fault_status;
    fault_status.name = "teensy_fault";
    fault_status.hardware_id = serial_port_;
    fault_status.level = fault_flags == 0 ? diagnostic_msgs::msg::DiagnosticStatus::OK
                                          : diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    fault_status.message = fault_flags == 0 ? "No faults" : "Faults reported";
    fault_status.values.push_back(key_value("fault_flags", std::to_string(fault_flags)));

    diagnostic_msgs::msg::DiagnosticStatus actuator_status;
    actuator_status.name = "actuator_limits";
    actuator_status.hardware_id = serial_port_;
    actuator_status.level = actuator_timeout ? diagnostic_msgs::msg::DiagnosticStatus::WARN
                                             : diagnostic_msgs::msg::DiagnosticStatus::OK;
    actuator_status.message = actuator_timeout ? "Holding last actuator targets" : "Actuator targets fresh";
    actuator_status.values.push_back(key_value("actuator_timeout", std::to_string(actuator_timeout)));

    array.status = {serial_status, motor_status, fault_status, actuator_status};
    diag_pub_->publish(array);
  }

  diagnostic_msgs::msg::KeyValue key_value(const std::string & key, const std::string & value) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = value;
    return kv;
  }

  double normalize_angle(double angle) const {
    while (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
      angle += 2.0 * M_PI;
    }
    return angle;
  }

  std::string serial_port_;
  int baudrate_{};
  double ticks_per_rev_{};
  double cmd_vel_timeout_{};
  double actuator_timeout_{};
  double feedback_timeout_{};
  double wheel_radius_{};
  double wheel_base_{};
  double max_wheel_rad_s_{};
  std::string odom_frame_id_;
  std::string base_frame_id_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr actuator_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;

  TeensyTransport transport_;
  DrivetrainInterface drivetrain_;
  ActuatorInterface actuators_;

  geometry_msgs::msg::Twist last_cmd_vel_;
  rclcpp::Time last_cmd_vel_time_;
  rclcpp::Time last_actuator_time_;
  float lift_target_{0.0F};
  float blade_target_{0.0F};

  rclcpp::Time last_feedback_time_{0, 0, RCL_ROS_TIME};

  bool have_ticks_{false};
  int32_t last_fl_ticks_{0};
  int32_t last_fr_ticks_{0};
  int32_t last_rl_ticks_{0};
  int32_t last_rr_ticks_{0};
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};

  double x_{0.0};
  double y_{0.0};
  double theta_{0.0};
};

}  // namespace lunabot_hardware_interface

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lunabot_hardware_interface::HardwareInterfaceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
