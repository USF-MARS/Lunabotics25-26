#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

#include <ctype.h>
// 1. Remove the macro definition that's causing the "passed 1 arguments" error
#ifdef __locale_ctype_ptr
#undef __locale_ctype_ptr
#endif

extern "C" {
  // 2. Declare the actual array as it exists in the Teensy toolchain (as char)
  extern const char _ctype_[];

  // 3. Provide the function the micro-ROS library is looking for
  // and cast the char array to the unsigned short pointer it expects
  const unsigned short* __locale_ctype_ptr(void) {
    return (const unsigned short*)_ctype_;
  }
}

// --- Hardware Pins ---
const int motor1Pin = 0; 
const int motor2Pin = 1;

// --- PWM Constants (12-bit at 50Hz) ---
const int neutral = 307;   // 1500us
const int range = 100;     // Max speed offset

// --- micro-ROS Objects ---
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;

// Error handling macro
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

// --- Callback Function ---
// This runs every time a /cmd_vel message is received
void subscription_callback(const void * msvin) {
  const geometry_msgsmsgTwist * msg = (const geometry_msgsmsgTwist )msvin;

  float val_x = msg->linear.x;

  // 1. Constrain to valid ROS range
  val_x = constrain(val_x, -1.0, 1.0);

  // 2. Convert float (-1.0 to 1.0) to integer (-100 to 100)
  int scaled_x = (int)(val_x 100);

  // 3. Map -100...100 to your PWM range 207...407 (Neutral 307)
  // 1000us (reverse) is ~205, 2000us (forward) is ~410
  int motorOutput = map(scaled_x, -100, 100, 207, 407);

  analogWrite(motor1Pin, motorOutput);
  analogWrite(motor2Pin, motorOutput);
}

void setup() {
  set_microros_transports();
  
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(motor1Pin, OUTPUT);
  pinMode(motor2Pin, OUTPUT);

  analogWriteFrequency(motor1Pin, 50);
  analogWriteFrequency(motor2Pin, 50);
  analogWriteResolution(12);
  
  // Initial Neutral
  analogWrite(motor1Pin, neutral);
  analogWrite(motor2Pin, neutral);

  delay(2000);

  allocator = rcl_get_default_allocator();

  // Initialize micro-ROS support
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "teensy_motor_node", "", &support));

  // Create subscriber for cmd_vel
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  // Create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}

void loop() {
  // Executor handles the callbacks
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}