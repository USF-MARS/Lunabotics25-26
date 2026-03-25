#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <ctype.h>

// --- Hardware Toggles ---
// Change to true if the right motors spin backward when commanded forward
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

// --- PWM Constants (12-bit at 50Hz) ---
const int neutral = 307;   // 1500us (Center/Stop)
const int range = 100;     // Max speed offset (approx 500us)

// --- micro-ROS Objects ---
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

// --- Differential Drive Callback ---
void subscription_callback(const void * msvin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msvin;
  
  float linear_x = msg->linear.x;   // Forward/Backward
  float angular_z = msg->angular.z; // Left/Right rotation

  // --- Differential Drive Logic (Arcade Mix) ---
  float left_val  = linear_x - angular_z;
  float right_val = linear_x + angular_z;

  // --- Normalization ---
  // If the requested speed exceeds 1.0, scale both sides down proportionally
  // so the rover maintains its turning arc.
  float max_val = max(abs(left_val), abs(right_val));
  if (max_val > 1.0) {
    left_val /= max_val;
    right_val /= max_val;
  }

  // Handle hardware mirroring if necessary
  if (invert_right_side) {
    right_val = -right_val;
  }

  // Map to PWM values
  int leftPWM  = neutral + (int)(left_val * range);
  int rightPWM = neutral + (int)(right_val * range);
  
  // Write to Left H-Bridge/ESCs
  analogWrite(motorLeft1, leftPWM);
  analogWrite(motorLeft2, leftPWM);

  // Write to Right H-Bridge/ESCs
  analogWrite(motorRight1, rightPWM);
  analogWrite(motorRight2, rightPWM);
}

void setup() {
  set_microros_transports();
  
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(motorLeft1, OUTPUT);
  pinMode(motorLeft2, OUTPUT);
  pinMode(motorRight1, OUTPUT);
  pinMode(motorRight2, OUTPUT);

  // Set 50Hz frequency for all motor pins
  analogWriteFrequency(motorLeft1, 50);
  analogWriteFrequency(motorLeft2, 50);
  analogWriteFrequency(motorRight1, 50);
  analogWriteFrequency(motorRight2, 50);
  analogWriteResolution(12);
  
  // Initialize motors at neutral
  analogWrite(motorLeft1, neutral);
  analogWrite(motorLeft2, neutral);
  analogWrite(motorRight1, neutral);
  analogWrite(motorRight2, neutral);

  delay(2000);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "teensy_motor_node", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}
