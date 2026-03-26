#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h>
#include <ctype.h>

// --- 1. Locale Fix for Teensy/micro-ROS Toolchain ---
#ifdef __locale_ctype_ptr
#undef __locale_ctype_ptr
#endif

extern "C" {
  extern const char _ctype_[];
  const unsigned short* __locale_ctype_ptr(void) {
    return (const unsigned short*)_ctype_;
  }
}

// --- 2. Pin Definitions ---
// Drive Motors (H-Bridges)
const int motorLeft1 = 0; 
const int motorLeft2 = 1;
const int motorRight1 = 7; 
const int motorRight2 = 8;

// Linear Actuator
const int LIFT_PWM_PIN = 19; 
const int LIFT_DIR_PIN = 17;

// --- 3. Constants ---
const int neutral = 307;   // 1500us for 50Hz/12-bit
const int range = 100;     // Speed offset
const int liftFreq = 5000; // 5kHz for Actuator
const int liftRes = 8;     // 8-bit (0-255) for Actuator

// --- 4. micro-ROS Objects ---
rcl_subscription_t drive_subscriber;
rcl_subscription_t lift_subscriber;
geometry_msgs__msg__Twist drive_msg;
std_msgs__msg__Int32 lift_msg;
rclc_executor_t executor;
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;

// Error handling macros
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

// --- 5. Callback: Drive Motors (/cmd_vel) ---
void drive_callback(const void * msvin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msvin;
  
  float linear_x = msg->linear.x;   
  float angular_z = msg->angular.z; 

  // Mix Linear and Angular for Differential Drive
  float left_val  = linear_x - angular_z;
  float right_val = linear_x + angular_z;

  left_val  = constrain(left_val, -1.0, 1.0);
  right_val = constrain(right_val, -1.0, 1.0);

  int leftPWM  = neutral + (int)(left_val * range);
  int rightPWM = neutral + (int)(right_val * range);
  
  analogWrite(motorLeft1, leftPWM);
  analogWrite(motorLeft2, leftPWM);
  analogWrite(motorRight1, rightPWM);
  analogWrite(motorRight2, rightPWM);
}

// --- 6. Callback: Linear Actuator (/lift_actuator) ---
void lift_callback(const void * msvin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msvin;
  int command = msg->data;

  if (command == 1) {        // Extend
    digitalWrite(LIFT_DIR_PIN, HIGH);
    analogWrite(LIFT_PWM_PIN, 255);
  } 
  else if (command == -1) {  // Retract
    digitalWrite(LIFT_DIR_PIN, LOW);
    analogWrite(LIFT_PWM_PIN, 255);
  } 
  else {                     // Stop
    analogWrite(LIFT_PWM_PIN, 0);
  }
}

void setup() {
  set_microros_transports();
  
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Setup Drive Pins
  pinMode(motorLeft1, OUTPUT);
  pinMode(motorLeft2, OUTPUT);
  pinMode(motorRight1, OUTPUT);
  pinMode(motorRight2, OUTPUT);
  analogWriteFrequency(motorLeft1, 50);
  analogWriteFrequency(motorRight1, 50); // Setting freq for one pin in group is usually enough
  analogWriteResolution(12);
  
  // Setup Actuator Pins
  pinMode(LIFT_PWM_PIN, OUTPUT);
  pinMode(LIFT_DIR_PIN, OUTPUT);
  analogWriteFrequency(LIFT_PWM_PIN, liftFreq);
  // Note: analogWriteResolution is global; we'll handle the 8-bit scale manually or set to 12.
  // To keep it simple, we use the 12-bit resolution (0-4095) for everything.
  
  // Initialize to Neutral/Stop
  analogWrite(motorLeft1, neutral);
  analogWrite(motorLeft2, neutral);
  analogWrite(motorRight1, neutral);
  analogWrite(motorRight2, neutral);
  analogWrite(LIFT_PWM_PIN, 0);

  delay(2000);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "teensy_robot_node", "", &support));

  // Init Drive Sub
  RCCHECK(rclc_subscription_init_default(
    &drive_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  // Init Lift Sub
  RCCHECK(rclc_subscription_init_default(
    &lift_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "lift_actuator"));

  // Create Executor for 2 handles
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &drive_subscriber, &drive_msg, &drive_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &lift_subscriber, &lift_msg, &lift_callback, ON_NEW_DATA));
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}