#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { error_loop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { error_loop(); } }

// === Rover configuration ===
// Adjust these to match your drivetrain geometry and desired max speeds.
static const float kTrackWidthMeters = 0.62f;   // Distance between left/right wheel centers.
static const float kMaxLinearMps = 1.5f;        // Max linear velocity (m/s).
static const float kMaxAngularRadps = 2.0f;     // Max angular velocity (rad/s).
static const uint32_t kCmdTimeoutMs = 500;      // Stop if no command within this window.

// === Sabertooth configuration ===
// Two Sabertooth 2x60 controllers: one front axle, one rear axle.
// Both controllers share the same UART using packetized serial.
static const uint8_t kSabertoothFrontAddress = 128; // DIP address for front controller.
static const uint8_t kSabertoothRearAddress = 129;  // DIP address for rear controller.
static const uint32_t kSabertoothBaud = 9600;

static const uint8_t kMotor1 = 1;
static const uint8_t kMotor2 = 2;

static uint32_t last_cmd_ms = 0;

void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// Clamp speed to Sabertooth's 7-bit data range (0-127).
static uint8_t clamp_7bit(int value) {
  if (value < 0) {
    value = 0;
  }
  if (value > 127) {
    value = 127;
  }
  return static_cast<uint8_t>(value);
}

// Convert normalized speed [-1, 1] into Sabertooth 7-bit magnitude [-127, 127].
static int speed_from_normalized(float speed) {
  if (speed > 1.0f) {
    speed = 1.0f;
  }
  if (speed < -1.0f) {
    speed = -1.0f;
  }
  return static_cast<int>(speed * 127.0f);
}

// Send one packetized serial command to a Sabertooth address.
static void sabertooth_write(uint8_t address, uint8_t command, uint8_t data) {
  uint8_t checksum = (address + command + data) & 0x7F;
  Serial1.write(address);
  Serial1.write(command);
  Serial1.write(data);
  Serial1.write(checksum);
}

// Command a specific motor channel on a Sabertooth controller.
static void sabertooth_set_motor(uint8_t address, uint8_t motor, int speed) {
  if (motor != kMotor1 && motor != kMotor2) {
    return;
  }

  uint8_t command_forward = (motor == kMotor1) ? 0 : 4;
  uint8_t command_backward = (motor == kMotor1) ? 1 : 5;

  if (speed >= 0) {
    sabertooth_write(address, command_forward, clamp_7bit(speed));
  } else {
    sabertooth_write(address, command_backward, clamp_7bit(-speed));
  }
}

// Apply the same speed to both motors on a Sabertooth controller (left/right).
static void set_axle_speed(uint8_t address, int speed) {
  sabertooth_set_motor(address, kMotor1, speed);
  sabertooth_set_motor(address, kMotor2, speed);
}

static void stop_rover() {
  set_axle_speed(kSabertoothFrontAddress, 0);
  set_axle_speed(kSabertoothRearAddress, 0);
}

// Twist message callback
void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

  // Desired chassis linear and angular velocities.
  float linear = msg->linear.x;
  float angular = msg->angular.z;

  if (linear > kMaxLinearMps) {
    linear = kMaxLinearMps;
  }
  if (linear < -kMaxLinearMps) {
    linear = -kMaxLinearMps;
  }

  if (angular > kMaxAngularRadps) {
    angular = kMaxAngularRadps;
  }
  if (angular < -kMaxAngularRadps) {
    angular = -kMaxAngularRadps;
  }

  // Differential drive kinematics for left/right sides.
  float left_mps = linear - (angular * kTrackWidthMeters * 0.5f);
  float right_mps = linear + (angular * kTrackWidthMeters * 0.5f);

  float max_mps = kMaxLinearMps;
  float left_norm = left_mps / max_mps;
  float right_norm = right_mps / max_mps;

  int left_cmd = speed_from_normalized(left_norm);
  int right_cmd = speed_from_normalized(right_norm);

  // Front controller drives front-left/front-right, rear drives rear-left/rear-right.
  // Each controller receives the left/right side command for its axle.
  sabertooth_set_motor(kSabertoothFrontAddress, kMotor1, left_cmd);
  sabertooth_set_motor(kSabertoothFrontAddress, kMotor2, right_cmd);
  sabertooth_set_motor(kSabertoothRearAddress, kMotor1, left_cmd);
  sabertooth_set_motor(kSabertoothRearAddress, kMotor2, right_cmd);

  last_cmd_ms = millis();

  digitalWrite(LED_PIN, (left_cmd == 0 && right_cmd == 0) ? LOW : HIGH);
}

void setup() {
  set_microros_transports();
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  Serial1.begin(kSabertoothBaud);

  delay(2000);

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_rover_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  last_cmd_ms = millis();
}

void loop() {
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(20)));

  if ((millis() - last_cmd_ms) > kCmdTimeoutMs) {
    stop_rover();
    digitalWrite(LED_PIN, LOW);
  }

  delay(20);
}
