#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h>
#include <ctype.h>

// --- Hardware Toggles ---
const bool invert_right_side = false; 

// --- Locale Fix for Teensy/micro-ROS ---
#ifdef __locale_ctype_ptr
#undef __locale_ctype_ptr
#endif

extern "C" {
  extern const char _ctype_[];
  const unsigned short* __locale_ctype_ptr(void) {
    return (const unsigned short*)_ctype_;
  }
}

// --- Hardware Pins ---
const int motorLeft1 = 0; 
const int motorLeft2 = 1;
const int motorRight1 = 7; 
const int motorRight2 = 8;
const int LIFT_PWM_PIN = 19;
const int LIFT_DIR_PIN = 17;

// --- PWM Constants (12-bit Resolution) ---
const int driveNeutral = 307;   // 1500us at 50Hz
const int driveRange   = 100;   
const int liftFullPower = 4095; // 100% duty cycle at 12-bit

// --- micro-ROS Objects ---
rclc_support_t support;
rcl_node_t node;
rcl_allocator_t allocator;
rclc_executor_t executor;

rcl_subscription_t drive_sub;
geometry_msgs__msg__Twist drive_msg;

rcl_subscription_t lift_sub;
std_msgs__msg__Int32 lift_msg;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

// --- Drivetrain Callback ---
void drive_callback(const void * msvin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msvin;
  
  float left_val  = msg->linear.x - msg->angular.z;
  float right_val = msg->linear.x + msg->angular.z;

  // Normalization
  float max_val = max(abs(left_val), abs(right_val));
  if (max_val > 1.0) {
    left_val /= max_val;
    right_val /= max_val;
  }

  if (invert_right_side) right_val = -right_val;

  int leftPWM  = driveNeutral + (int)(left_val * driveRange);
  int rightPWM = driveNeutral + (int)(right_val * driveRange);
  
  analogWrite(motorLeft1, leftPWM);
  analogWrite(motorLeft2, leftPWM);
  analogWrite(motorRight1, rightPWM);
  analogWrite(motorRight2, rightPWM);
}

// --- Lift Callback ---
void lift_callback(const void * msvin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msvin;
  int command = msg->data;

  if (command == 1) {
    digitalWrite(LIFT_DIR_PIN, HIGH);
    analogWrite(LIFT_PWM_PIN, liftFullPower);
  } else if (command == -1) {
    digitalWrite(LIFT_DIR_PIN, LOW);
    analogWrite(LIFT_PWM_PIN, liftFullPower);
  } else {
    analogWrite(LIFT_PWM_PIN, 0);
  }
}

void setup() {
  set_microros_transports();
  
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(motorLeft1, OUTPUT); pinMode(motorLeft2, OUTPUT);
  pinMode(motorRight1, OUTPUT); pinMode(motorRight2, OUTPUT);
  pinMode(LIFT_PWM_PIN, OUTPUT); pinMode(LIFT_DIR_PIN, OUTPUT);

  // Frequency Setup
  // NOTE: Pins 0, 1, 7, 8 are on different timers than Pin 19 on Teensy 4.1
  analogWriteFrequency(motorLeft1, 50);
  analogWriteFrequency(motorRight1, 50);
  analogWriteFrequency(LIFT_PWM_PIN, 5000); 
  
  analogWriteResolution(12);
  
  // Initial States
  analogWrite(motorLeft1, driveNeutral);
  analogWrite(motorLeft2, driveNeutral);
  analogWrite(motorRight1, driveNeutral);
  analogWrite(motorRight2, driveNeutral);
  analogWrite(LIFT_PWM_PIN, 0);

  // Wait for Agent Handshake (Prevents the crash/blink)
  while (RMW_RET_OK != rmw_uros_ping_agent(100, 1)) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(500); 
  }

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "teensy_robot_node", "", &support));

  // Subscriptions
  RCCHECK(rclc_subscription_init_default(&drive_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
  RCCHECK(rclc_subscription_init_default(&lift_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "lift_actuator"));

  // Executor (Handle count MUST be 2)
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &drive_sub, &drive_msg, &drive_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &lift_sub, &lift_msg, &lift_callback, ON_NEW_DATA));
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}