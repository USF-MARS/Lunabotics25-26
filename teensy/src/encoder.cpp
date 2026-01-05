#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/joint_state.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <micro_ros_utilities/string_utilities.h>

#include "encoder.h"

// --- CONFIGURATION ---
#define LED_PIN 13
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { error_loop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {} }

// Update these pins to match the CIMcoder wiring.
const uint8_t kLeftFrontA = 2;
const uint8_t kLeftFrontB = 3;
const uint8_t kLeftRearA = 4;
const uint8_t kLeftRearB = 5;
const uint8_t kRightFrontA = 6;
const uint8_t kRightFrontB = 7;
const uint8_t kRightRearA = 8;
const uint8_t kRightRearB = 9;

const float kTicksPerRev = 2048.0f;
const float kRadPerTick = 2.0f * 3.1415926f / kTicksPerRev;

// --- ROS VARIABLES ---
rcl_publisher_t publisher;
rcl_timer_t timer;
sensor_msgs__msg__JointState encoder_msg;

// --- SYNC VARIABLES ---
unsigned long last_sync_time = 0;
const unsigned long kSyncIntervalMs = 60000;

// --- ENCODER STORAGE ---
enum EncoderIndex {
  kLeftFront = 0,
  kLeftRear = 1,
  kRightFront = 2,
  kRightRear = 3,
  kEncoderCount = 4
};

volatile long encoder_counts[kEncoderCount] = {0, 0, 0, 0};
long last_counts[kEncoderCount] = {0, 0, 0, 0};
unsigned long last_publish_ms = 0;

void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void left_front_isr() {
  encoder_counts[kLeftFront] += digitalRead(kLeftFrontB) ? 1 : -1;
}

void left_rear_isr() {
  encoder_counts[kLeftRear] += digitalRead(kLeftRearB) ? 1 : -1;
}

void right_front_isr() {
  encoder_counts[kRightFront] += digitalRead(kRightFrontB) ? 1 : -1;
}

void right_rear_isr() {
  encoder_counts[kRightRear] += digitalRead(kRightRearB) ? 1 : -1;
}

void timer_callback(rcl_timer_t *timer_handle, int64_t last_call_time) {
  RCLC_UNUSED(timer_handle);
  RCLC_UNUSED(last_call_time);

  struct timespec tv = {0};
  clock_gettime(CLOCK_REALTIME, &tv);
  encoder_msg.header.stamp.sec = tv.tv_sec;
  encoder_msg.header.stamp.nanosec = tv.tv_nsec;

  unsigned long now_ms = millis();
  float dt = (now_ms - last_publish_ms) / 1000.0f;
  if (dt <= 0.0f) {
    dt = 0.001f;
  }
  last_publish_ms = now_ms;

  long counts[kEncoderCount];
  noInterrupts();
  for (int i = 0; i < kEncoderCount; ++i) {
    counts[i] = encoder_counts[i];
  }
  interrupts();

  encoder_msg.position.size = kEncoderCount;
  encoder_msg.velocity.size = kEncoderCount;

  for (int i = 0; i < kEncoderCount; ++i) {
    long delta = counts[i] - last_counts[i];
    encoder_msg.position.data[i] = counts[i] * kRadPerTick;
    encoder_msg.velocity.data[i] = (delta * kRadPerTick) / dt;
    last_counts[i] = counts[i];
  }

  RCSOFTCHECK(rcl_publish(&publisher, &encoder_msg, NULL));
}

void encoder_init(rcl_node_t *node, rclc_executor_t *executor, rclc_support_t *support) {
  pinMode(kLeftFrontA, INPUT_PULLUP);
  pinMode(kLeftFrontB, INPUT_PULLUP);
  pinMode(kLeftRearA, INPUT_PULLUP);
  pinMode(kLeftRearB, INPUT_PULLUP);
  pinMode(kRightFrontA, INPUT_PULLUP);
  pinMode(kRightFrontB, INPUT_PULLUP);
  pinMode(kRightRearA, INPUT_PULLUP);
  pinMode(kRightRearB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(kLeftFrontA), left_front_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(kLeftRearA), left_rear_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(kRightFrontA), right_front_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(kRightRearA), right_rear_isr, CHANGE);

  if (!sensor_msgs__msg__JointState__init(&encoder_msg)) {
    error_loop();
  }
  if (!rosidl_runtime_c__String__Sequence__init(&encoder_msg.name, kEncoderCount)) {
    error_loop();
  }
  if (!rosidl_runtime_c__double__Sequence__init(&encoder_msg.position, kEncoderCount)) {
    error_loop();
  }
  if (!rosidl_runtime_c__double__Sequence__init(&encoder_msg.velocity, kEncoderCount)) {
    error_loop();
  }

  if (!rosidl_runtime_c__String__assign(&encoder_msg.name.data[kLeftFront], "left_front_wheel")) {
    error_loop();
  }
  if (!rosidl_runtime_c__String__assign(&encoder_msg.name.data[kLeftRear], "left_rear_wheel")) {
    error_loop();
  }
  if (!rosidl_runtime_c__String__assign(&encoder_msg.name.data[kRightFront], "right_front_wheel")) {
    error_loop();
  }
  if (!rosidl_runtime_c__String__assign(&encoder_msg.name.data[kRightRear], "right_rear_wheel")) {
    error_loop();
  }

  encoder_msg.header.frame_id = micro_ros_string_utilities_set(encoder_msg.header.frame_id, "base_link");

  RCCHECK(rclc_publisher_init_default(
    &publisher,
    node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "/wheel_encoder/joint_states"));

  RCCHECK(rclc_timer_init_default(
    &timer,
    support,
    RCL_MS_TO_NS(20),
    timer_callback));

  RCCHECK(rclc_executor_add_timer(executor, &timer));

  last_publish_ms = millis();
}

void encoder_update() {
  if (millis() - last_sync_time > kSyncIntervalMs) {
    last_sync_time = millis();
    rmw_uros_sync_session(100);
  }
}
