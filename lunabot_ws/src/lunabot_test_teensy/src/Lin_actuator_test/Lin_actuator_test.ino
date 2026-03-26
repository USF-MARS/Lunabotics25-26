#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
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

// --- Hardware Pins ---
const int motorLeft1  = 0; 
const int motorLeft2  = 1;
const int motorRight1 = 7; 
const int motorRight2 = 8;
const int LIFT_PWM_PIN = 19;
const int LIFT_DIR_PIN = 17;

// --- PWM Constants (12-bit = 0 to 4095) ---
const int driveNeutral = 307;  // 1500us
const int driveRange   = 100; 
const int liftFullPower = 4095; // 100% Duty Cycle at 12-bit

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

void drive_callback(const void * msvin) {
  const geometry_msgs__msg__Twist * incoming_msg = (const geometry_msgs__msg__Twist *)msvin;
  float left_val  = constrain(incoming_msg->linear.x - incoming_msg->angular.z, -1.0, 1.0);
  float right_val = constrain(incoming_msg->linear.x + incoming_msg->angular.z, -1.0, 1.0);

  analogWrite(motorLeft1, driveNeutral + (int)(left_val * driveRange));
  analogWrite(motorLeft2, driveNeutral + (int)(left_val * driveRange));
  analogWrite(motorRight1, driveNeutral + (int)(right_val * driveRange));
  analogWrite(motorRight2, driveNeutral + (int)(right_val * driveRange));
}

void lift_callback(const void * msvin) {
  const std_msgs__msg__Int32 * incoming_msg = (const std_msgs__msg__Int32 *)msvin;
  int command = incoming_msg->data;

  if (command == 1) {        // EXTEND
    digitalWrite(LIFT_DIR_PIN, HIGH);
    analogWrite(LIFT_PWM_PIN, liftFullPower); 
  } 
  else if (command == -1) {  // RETRACT
    digitalWrite(LIFT_DIR_PIN, LOW);
    analogWrite(LIFT_PWM_PIN, liftFullPower);
  } 
  else {                     // STOP
    analogWrite(LIFT_PWM_PIN, 0);
  }
}

void setup() {
  set_microros_transports();
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Drivetrain Setup
  pinMode(motorLeft1, OUTPUT); pinMode(motorLeft2, OUTPUT);
  pinMode(motorRight1, OUTPUT); pinMode(motorRight2, OUTPUT);
  analogWriteFrequency(motorLeft1, 50); 
  analogWriteFrequency(motorRight1, 50);
  
  // Actuator Setup
  pinMode(LIFT_PWM_PIN, OUTPUT);
  pinMode(LIFT_DIR_PIN, OUTPUT);
  analogWriteFrequency(LIFT_PWM_PIN, 5000); 

  // Global Resolution for Teensy
  analogWriteResolution(12);

  // Default States
  analogWrite(motorLeft1, driveNeutral);
  analogWrite(motorLeft2, driveNeutral);
  analogWrite(motorRight1, driveNeutral);
  analogWrite(motorRight2, driveNeutral);
  analogWrite(LIFT_PWM_PIN, 0);

  delay(2000);
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "teensy_robot_node", "", &support));
  RCCHECK(rclc_subscription_init_default(&drive_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
  RCCHECK(rclc_subscription_init_default(&lift_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "lift_actuator"));
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &drive_sub, &drive_msg, &drive_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &lift_sub, &lift_msg, &lift_callback, ON_NEW_DATA));
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}