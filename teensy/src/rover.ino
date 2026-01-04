#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

// Link to our custom drive code
#include "diff_drive.h"

// ==========================================
//          ROS SETUP (The Plumbing)
// ==========================================
rcl_subscription_t subscriber;  // The "mailbox" we listen to
geometry_msgs__msg__Twist msg;  // The "letter" format (Twist = speed commands)
rclc_executor_t executor;       // The "worker" that processes messages
rcl_allocator_t allocator;      // Memory manager
rclc_support_t support;         // System support
rcl_node_t node;                // This program's name in the network

#define LED_PIN 13

// A macro to check for errors. 
// If a ROS function fails, this forces the Teensy into an error loop (blinking LED).
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { error_loop(); } }

// ==========================================
//          SAFETY WATCHDOG
// ==========================================
// If the laptop crashes or Wi-Fi breaks, we need to stop the robot automatically.
// We wait 500ms (0.5 seconds). If no message arrives, we pull the brakes.
const uint32_t kCmdTimeoutMs = 500;
uint32_t last_cmd_ms = 0;

// ERROR MODE: Blinks the LED fast if something is broken.
void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// ==========================================
//          THE CALLBACK FUNCTION
// ==========================================
// This function runs AUTOMATICALLY every time the laptop sends a 'cmd_vel' message.
void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

  // 1. Read the speeds the laptop wants
  float linear_x = msg->linear.x;   // Forward/Back
  float angular_z = msg->angular.z; // Left/Right

  // 2. Send them to our custom drive engine (in diff_drive.cpp)
  //    (That engine handles the smooth acceleration and motor math)
  drive_command(linear_x, angular_z);

  // 3. Reset the safety timer because we just heard from the laptop!
  last_cmd_ms = millis();
  
  // 4. Visual Feedback: Turn LED ON if we are moving, OFF if stopped.
  //    (0.01 is a tiny "deadzone" to ignore static noise)
  bool is_moving = (abs(linear_x) > 0.01 || abs(angular_z) > 0.01);
  digitalWrite(LED_PIN, is_moving ? HIGH : LOW);
}

// ==========================================
//          SETUP (Runs Once)
// ==========================================
void setup() {
  set_microros_transports(); // Start the USB/Serial connection to the laptop
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // Initialize our custom motor code
  drive_init();

  delay(2000); // Wait 2 seconds for electronics to warm up

  allocator = rcl_get_default_allocator();

  // 1. Initialize the ROS Node
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_rover_node", "", &support));

  // 2. Create the Subscriber
  //    We tell it to listen to the topic "cmd_vel"
  //    We tell it the message type is "Twist"
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  // 3. Create the Executor
  //    This links the "Subscriber" to the "callback function" above.
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  last_cmd_ms = millis();
}

// ==========================================
//          LOOP (Runs Forever)
// ==========================================
void loop() {
  // 1. Check for new messages from the laptop
  //    This will trigger 'subscription_callback' if a message is waiting.
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(20)));

  // 2. Safety Watchdog Check
  //    Calculate how long it's been since the last message.
  if ((millis() - last_cmd_ms) > kCmdTimeoutMs) {
    // If it's been too long (>0.5s), STOP THE ROBOT.
    drive_stop();
    digitalWrite(LED_PIN, LOW);
  }

  delay(10); // Small pause to let the processor breathe
}