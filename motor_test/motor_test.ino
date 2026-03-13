#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

// Updated Pins
const int pinFL = 0; 
const int pinFR = 7; 
const int pinBL = 1; 
const int pinBR = 8; 

const int NEUTRAL = 307; // 1500us for Sabertooth/PWM H-Bridges
const int RANGE = 100;   // Power scaling

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

void error_loop() {
  while(1) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

void subscription_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

  float linear = msg->linear.x;
  float angular = msg->angular.z;

  float leftSpeed = linear - angular;
  float rightSpeed = linear + angular;

  int leftPWM = NEUTRAL + (constrain(leftSpeed, -1.0, 1.0) * RANGE);
  int rightPWM = NEUTRAL + (constrain(rightSpeed, -1.0, 1.0) * RANGE);

  analogWrite(pinFL, leftPWM);
  analogWrite(pinBL, leftPWM);
  analogWrite(pinFR, rightPWM);
  analogWrite(pinBR, rightPWM);
}

void setup() {
  set_microros_transports();
  pinMode(LED_BUILTIN, OUTPUT);

  int pins[] = {pinFL, pinFR, pinBL, pinBR};
  for(int p : pins) {
    pinMode(p, OUTPUT);
    analogWriteFrequency(p, 50); // 50Hz for Sabertooth
    analogWriteResolution(12);
    analogWrite(p, NEUTRAL);
  }

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "teensy_rover_node", "", &support));
  RCCHECK(rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}

void loop() {
  // If the agent is disconnected, this will fail and trigger error_loop (blinking LED)
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}