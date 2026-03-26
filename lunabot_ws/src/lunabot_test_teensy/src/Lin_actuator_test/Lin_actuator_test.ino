#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

#include <ctype.h>

// --- Teensy/micro-ROS Locale Fix ---
#ifdef __locale_ctype_ptr
#undef __locale_ctype_ptr
#endif

extern "C" {
  extern const char _ctype_[];
  const unsigned short* __locale_ctype_ptr(void) {
    return (const unsigned short*)_ctype_;
  }
}

// --- Pin Definitions ---
const int PWM_PIN = 19;  // PWM speed control
const int DIR_PIN = 17;  // Direction control
const int LED_PIN = LED_BUILTIN;

// --- PWM Settings ---
const int pwmFreq = 5000;    // 5 kHz
const int pwmResolution = 8; // 0-255

// --- micro-ROS Objects ---
rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg;
rclc_executor_t executor;
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;

// Error handling macro
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// --- Actuator Control Function ---
void moveActuator(int command) {
  if (command == 1) {
    // EXTEND (Button 5)
    digitalWrite(DIR_PIN, HIGH);
    analogWrite(PWM_PIN, 255);
  } 
  else if (command == -1) {
    // RETRACT (Button 4)
    digitalWrite(DIR_PIN, LOW);
    analogWrite(PWM_PIN, 255);
  } 
  else {
    // STOP (No buttons pressed)
    analogWrite(PWM_PIN, 0);
  }
}

// --- ROS Callback ---
void subscription_callback(const void * msvin) {
  const std_msgs__msg__Int32 * incoming_msg = (const std_msgs__msg__Int32 *)msvin;
  moveActuator(incoming_msg->data);
}

void setup() {
  // Set up Serial for micro-ROS
  set_microros_transports();
  
  // Pin Configuration
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  
  analogWriteFrequency(PWM_PIN, pwmFreq);
  analogWriteResolution(pwmResolution);
  
  // Initialize at stop
  analogWrite(PWM_PIN, 0);

  delay(2000);

  allocator = rcl_get_default_allocator();

  // Initialize micro-ROS support
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "teensy_actuator_node", "", &support));

  // Create subscriber for /lift_actuator
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "lift_actuator"));

  // Create executor (1 handle for 1 subscriber)
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}

void loop() {
  // Spin the executor to handle incoming messages
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}