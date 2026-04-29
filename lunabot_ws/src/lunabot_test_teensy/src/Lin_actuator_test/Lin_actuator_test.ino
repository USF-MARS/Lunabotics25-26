#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h>
#include <ctype.h>

// --- Locale Fix for Teensy ---
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
const int S1_PIN = 0; // Sabertooth S1 (Left)
const int S2_PIN = 7; // Sabertooth S2 (Right)
const int ARM_PWM_PIN = 19;
const int ARM_DIR_PIN = 17;
const int BUCKET_PWM_PIN = 18;
const int BUCKET_DIR_PIN = 20;

// --- PWM Constants (12-bit @ 50Hz) ---
const int driveNeutral = 307;  // 1500us
const int driveRange   = 102;  // Full sweep 1ms to 2ms
const int ACTUATOR_FULL = 4095; 

// --- EMA Smoothing Variables ---
float left_ema = 0.0;
float right_ema = 0.0;
const float alpha = 0.2; // Smoothing factor (Adjust 0.05 to 0.5)

// --- Watchdog ---
unsigned long last_msg_time = 0;
const unsigned long TIMEOUT_MS = 500;

// --- micro-ROS Objects ---
rclc_support_t support;
rcl_node_t node;
rcl_allocator_t allocator;
rclc_executor_t executor;

rcl_subscription_t drive_sub;
geometry_msgs__msg__Twist drive_msg;
rcl_subscription_t arm_sub;
std_msgs__msg__Int32 arm_msg;
rcl_subscription_t bucket_sub;
std_msgs__msg__Int32 bucket_msg;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

void drive_callback(const void * msvin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msvin;
  last_msg_time = millis();

  float target_left  = msg->linear.x - msg->angular.z;
  float target_right = msg->linear.x + msg->angular.z;

  // Normalization
  float max_val = fmaxf(1.0, fmaxf(fabs(target_left), fabs(target_right)));
  target_left /= max_val;
  target_right /= max_val;

  // Apply EMA Smoothing
  left_ema  = (alpha * target_left) + ((1.0 - alpha) * left_ema);
  right_ema = (alpha * target_right) + ((1.0 - alpha) * right_ema);

  int leftPWM  = driveNeutral + (int)(left_ema * driveRange);
  int rightPWM = driveNeutral + (int)(right_ema * driveRange);
  
  analogWrite(S1_PIN, leftPWM);
  analogWrite(S2_PIN, rightPWM);
}

void arm_callback(const void * msvin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msvin;
  int command = msg->data;
  last_msg_time = millis();

  if (command == 1) {
    digitalWrite(ARM_DIR_PIN, HIGH);
    analogWrite(ARM_PWM_PIN, ACTUATOR_FULL);
  } else if (command == -1) {
    digitalWrite(ARM_DIR_PIN, LOW);
    analogWrite(ARM_PWM_PIN, ACTUATOR_FULL);
  } else {
    analogWrite(ARM_PWM_PIN, 0);
  }
}

void bucket_callback(const void * msvin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msvin;
  int command = msg->data;
  last_msg_time = millis();

  if (command == 1) {
    digitalWrite(BUCKET_DIR_PIN, HIGH);
    analogWrite(BUCKET_PWM_PIN, ACTUATOR_FULL);
  } else if (command == -1) {
    digitalWrite(BUCKET_DIR_PIN, LOW);
    analogWrite(BUCKET_PWM_PIN, ACTUATOR_FULL);
  } else {
    analogWrite(BUCKET_PWM_PIN, 0);
  }
}

void setup() {
  set_microros_transports();
  
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(S1_PIN, OUTPUT); 
  pinMode(S2_PIN, OUTPUT);
  pinMode(ARM_PWM_PIN, OUTPUT); 
  pinMode(ARM_DIR_PIN, OUTPUT);
  pinMode(BUCKET_PWM_PIN, OUTPUT); 
  pinMode(BUCKET_DIR_PIN, OUTPUT);

  // Frequency Setup
  analogWriteFrequency(S1_PIN, 50); // R/C Pulse frequency
  analogWriteFrequency(S2_PIN, 50);
  analogWriteFrequency(ARM_PWM_PIN, 5000); 
  analogWriteFrequency(BUCKET_PWM_PIN, 5000); 

  analogWriteResolution(12);
  
  // Initial States (Stop)
  analogWrite(S1_PIN, driveNeutral);
  analogWrite(S2_PIN, driveNeutral);
  analogWrite(ARM_PWM_PIN, 0);
  analogWrite(BUCKET_PWM_PIN, 0);

  // Agent Handshake
  while (RMW_RET_OK != rmw_uros_ping_agent(100, 1)) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(250); 
  }

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "teensy_robot_node", "", &support));

  RCCHECK(rclc_subscription_init_default(&drive_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
  RCCHECK(rclc_subscription_init_default(&arm_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "arm_actuator"));
  RCCHECK(rclc_subscription_init_default(&bucket_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "bucket_actuator"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &drive_sub, &drive_msg, &drive_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &arm_sub, &arm_msg, &arm_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &bucket_sub, &bucket_msg, &bucket_callback, ON_NEW_DATA));
}

void loop() {
  // Check for communication timeout
  if (millis() - last_msg_time > TIMEOUT_MS) {
    left_ema = 0;
    right_ema = 0;
    analogWrite(S1_PIN, driveNeutral);
    analogWrite(S2_PIN, driveNeutral);
    analogWrite(ARM_PWM_PIN, 0);
    analogWrite(BUCKET_PWM_PIN, 0);
  }

  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}