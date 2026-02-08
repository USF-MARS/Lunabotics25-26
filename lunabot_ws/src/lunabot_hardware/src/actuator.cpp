#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <sensor_msgs/msg/joint_state.h>

#include "actuator.h"

// ==========================================
//      CONFIGURATION (CHANGE PINS HERE)
// ==========================================

// --- LIFT ACTUATOR (The Big Arm) ---
// Potentiometer: Measures where the arm is right now.
const int kLiftPotPin   = A10; 
// Cytron Driver Inputs:
const int kLiftPwmPin   = 10;  // "Gas Pedal" (Speed)
const int kLiftDirPin   = 11;  // "Gear Stick" (Forward/Reverse)

// CALIBRATION:
// You must test this manually! Move the arm all the way back and read the number.
// Then move it all the way out and read the number.
const int kLiftRetracted = 50;  
const int kLiftExtended  = 950;

// --- TILT ACTUATOR (The Bucket) ---
const int kTiltPotPin   = A11; 
const int kTiltPwmPin   = 12;  // "Gas Pedal"
const int kTiltDirPin   = 24;  // "Gear Stick"

const int kTiltRetracted = 50; 
const int kTiltExtended  = 950;

// --- SPEED SETTINGS ---
const int kMaxSpeed = 255;    // Maximum speed (0-255). 255 is 100% power.
const int kMinSpeed = 60;     // Minimum speed. If we give less than this, the motor just hums but doesn't move.
const int kDeadzone = 15;     // "Close Enough". If we are within 15 points of the target, stop moving.

// ==========================================
//           INTERNAL MEMORY
// ==========================================

// We store the "Target" (Where we want to go) for each motor.
// 0.0 = Fully Retracted, 1.0 = Fully Extended
static float lift_target_percent = 0.0f;
static float tilt_target_percent = 0.0f;

// ROS Communication Objects
rcl_subscription_t lift_sub;
rcl_subscription_t tilt_sub;
rcl_publisher_t feedback_pub;

std_msgs__msg__Float32 lift_msg;
std_msgs__msg__Float32 tilt_msg;
sensor_msgs__msg__JointState feedback_msg;

// ==========================================
//           HELPER FUNCTIONS
// ==========================================

// Panic Loop: Blinks the LED fast if ROS crashes.
void actuator_error_loop() {
  pinMode(LED_BUILTIN, OUTPUT);
  while (1) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(200);
  }
}
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){actuator_error_loop();}}

// CYTRON MOTOR DRIVER INTERFACE
// This function translates "Speed" (-255 to +255) into the signals the Cytron needs.
// Positive Speed = Extend (DIR HIGH)
// Negative Speed = Retract (DIR LOW)
void set_cytron_speed(int pwm_pin, int dir_pin, int speed) {
  
  // 1. Safety Cap: Ensure we never try to send more than 100% power.
  if (speed > 255) speed = 255;
  if (speed < -255) speed = -255;

  if (speed > 0) {
    // EXTEND: Set Direction to HIGH, send PWM speed.
    digitalWrite(dir_pin, HIGH); 
    analogWrite(pwm_pin, speed);
  } else if (speed < 0) {
    // RETRACT: Set Direction to LOW, send POSITIVE PWM speed.
    // (Electronics can't handle negative numbers, so we make it positive).
    digitalWrite(dir_pin, LOW); 
    analogWrite(pwm_pin, -speed); 
  } else {
    // STOP: Cut power.
    analogWrite(pwm_pin, 0);
  }
}

// ==========================================
//           ROS CALLBACKS
// ==========================================
// These functions run automatically when the Laptop sends a command.

// When the laptop says "Move Lift to 50%":
void lift_callback(const void * msgin) {
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  // Save the new target. Clamp it so we don't try to go to 110%.
  lift_target_percent = msg->data;
  if (lift_target_percent > 1.0f) lift_target_percent = 1.0f;
  if (lift_target_percent < 0.0f) lift_target_percent = 0.0f;
}

// When the laptop says "Move Tilt to 100%":
void tilt_callback(const void * msgin) {
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  tilt_target_percent = msg->data;
  if (tilt_target_percent > 1.0f) tilt_target_percent = 1.0f;
  if (tilt_target_percent < 0.0f) tilt_target_percent = 0.0f;
}

// ==========================================
//           MAIN LOGIC
// ==========================================

void actuator_init(rcl_node_t *node, rclc_executor_t *executor, rclc_support_t *support) {
  // 1. Configure Pins (Tell the Teensy which pins are Input and Output)
  pinMode(kLiftPotPin, INPUT);
  pinMode(kLiftPwmPin, OUTPUT);
  pinMode(kLiftDirPin, OUTPUT);
  
  pinMode(kTiltPotPin, INPUT);
  pinMode(kTiltPwmPin, OUTPUT);
  pinMode(kTiltDirPin, OUTPUT);

  // 2. Initialize ROS Subscribers (Listening for commands)
  
  // Topic: /bucket_lift/cmd (Expects a number 0.0 to 1.0)
  RCCHECK(rclc_subscription_init_default(
    &lift_sub, node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/bucket_lift/cmd"));

  // Topic: /bucket_tilt/cmd (Expects a number 0.0 to 1.0)
  RCCHECK(rclc_subscription_init_default(
    &tilt_sub, node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/bucket_tilt/cmd"));

  // 3. Initialize ROS Publisher (Reporting status back to laptop)
  RCCHECK(rclc_publisher_init_default(
    &feedback_pub, node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "/bucket/joint_states"));

  // 4. Hook everything up to the Executor (The ROS "Brain")
  RCCHECK(rclc_executor_add_subscription(executor, &lift_sub, &lift_msg, &lift_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(executor, &tilt_sub, &tilt_msg, &tilt_callback, ON_NEW_DATA));

  // 5. Setup the Feedback Message memory (Boilerplate ROS setup)
  static double pos_data[2];
  static rosidl_runtime_c__String name_data[2];
  static char lift_name[] = "lift_joint";
  static char tilt_name[] = "tilt_joint";

  feedback_msg.position.data = pos_data;
  feedback_msg.position.size = 2;
  feedback_msg.position.capacity = 2;
  feedback_msg.name.data = name_data;
  feedback_msg.name.size = 2;
  feedback_msg.name.capacity = 2;
  
  rosidl_runtime_c__String__assign(&feedback_msg.name.data[0], lift_name);
  rosidl_runtime_c__String__assign(&feedback_msg.name.data[1], tilt_name);
}

// HELPER: The Logic for ONE actuator (PID Control)
// We put this in a function so we can reuse it for both Lift and Tilt.
void run_pid_control(int pot_pin, int pwm_pin, int dir_pin, float target_percent, int min_val, int max_val) {
  
  // 1. READ: Where are we right now?
  int current_val = analogRead(pot_pin);

  // 2. TARGET: Where do we want to be?
  // We convert the percentage (0.0 - 1.0) into a raw sensor number (e.g., 50 - 950).
  int target_val = min_val + (target_percent * (max_val - min_val));

  // 3. ERROR: How far away are we?
  // If result is positive, we need to extend. If negative, retract.
  int error = target_val - current_val;

  // 4. DECIDE: How fast should we move?
  int speed = 0;
  
  // If the error is small (inside the deadzone), just stop. This prevents shaking.
  if (abs(error) > kDeadzone) {
    
    // "P" Gain: Multiply error by 2.5 to get speed.
    // Big Error = Big Speed. Small Error = Slow Speed.
    speed = error * 2.5; 
    
    // 5. BOOST: Motors need a minimum amount of power to start moving.
    // If the math says "speed 10", the motor might just sit there and get hot.
    // We force it to at least "kMinSpeed" (60) to ensure it actually moves.
    if (speed > 0 && speed < kMinSpeed) speed = kMinSpeed;
    if (speed < 0 && speed > -kMinSpeed) speed = -kMinSpeed;
  }

  // 6. ACTION: Send the command to the Cytron driver.
  set_cytron_speed(pwm_pin, dir_pin, speed);
}

void actuator_update() {
  // Run logic for LIFT (Calculates speed and moves motor)
  run_pid_control(kLiftPotPin, kLiftPwmPin, kLiftDirPin, lift_target_percent, kLiftRetracted, kLiftExtended);

  // Run logic for TILT (Calculates speed and moves motor)
  run_pid_control(kTiltPotPin, kTiltPwmPin, kTiltDirPin, tilt_target_percent, kTiltRetracted, kTiltExtended);

  // PUBLISH FEEDBACK (Every 100ms)
  // This lets the laptop see where the arm is in real-time.
  static unsigned long last_pub_time = 0;
  if (millis() - last_pub_time > 100) {
    last_pub_time = millis();

    // Read values again for the report
    int lift_raw = analogRead(kLiftPotPin);
    int tilt_raw = analogRead(kTiltPotPin);

    // Normalize to 0.0 - 1.0 (Percentage) for the dashboard
    feedback_msg.position.data[0] = (float)(lift_raw - kLiftRetracted) / (float)(kLiftExtended - kLiftRetracted);
    feedback_msg.position.data[1] = (float)(tilt_raw - kTiltRetracted) / (float)(kTiltExtended - kTiltRetracted);

    rcl_publish(&feedback_pub, &feedback_msg, NULL);
  }
}