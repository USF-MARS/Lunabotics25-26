#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

// --- Hardware Pins ---
const int motor1Pin = 0; 
const int motor2Pin = 1;

// --- PWM Constants (12-bit) ---
const int neutral = 307;   // 1500us
const int range = 100;     // Max deviation from neutral (approx 500us)

// --- micro-ROS Objects ---
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

// --- Callback: This runs whenever Foxglove sends a cmd_vel message ---
void subscription_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

  // Linear x: Forward/Reverse (-1.0 to 1.0)
  // Angular z: Left/Right (-1.0 to 1.0)
  float linear = msg->linear.x;
  float angular = msg->angular.z;

  // Differential Drive Mixing
  float left_val = linear - angular;
  float right_val = linear + angular;

  // Map to PWM values (12-bit)
  int motor1Output = neutral + (left_val * range);
  int motor2Output = neutral + (right_val * range);

  // Constrain to prevent over-driving
  motor1Output = constrain(motor1Output, neutral - range, neutral + range);
  motor2Output = constrain(motor2Output, neutral - range, neutral + range);

  analogWrite(motor1Pin, motor1Output);
  analogWrite(motor2Pin, motor2Output);
}

void setup() {
  set_microros_transports();
  
  pinMode(motor1Pin, OUTPUT);
  pinMode(motor2Pin, OUTPUT);
  analogWriteFrequency(motor1Pin, 50);
  analogWriteFrequency(motor2Pin, 50);
  analogWriteResolution(12);

  // Initialize to Neutral
  analogWrite(motor1Pin, neutral);
  analogWrite(motor2Pin, neutral);

  delay(2000);

  allocator = rcl_get_default_allocator();

  // Initialize micro-ROS support
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "teensy_rover_node", "", &support));

  // Initialize subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  // Initialize executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}

void loop() {
  // Check for new messages
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}