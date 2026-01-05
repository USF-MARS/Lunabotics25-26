#include "diff_drive.h"

// ==========================================
//          INTERNAL MEMORY
// ==========================================

// We remember the current speed so we can change it gradually.
static float current_linear_speed = 0.0f;
static float current_angular_speed = 0.0f;

// We need to track TIME to make acceleration smooth.
static unsigned long last_ramp_time = 0;

// RAMPING SETTINGS (ACCELERATION LIMITS)
// How fast are we allowed to change speed?
// 2.0 means we can go from 0 to 2.0 m/s in exactly 1 second.
// Since max speed is 1.5 m/s, it will take about 0.75 seconds to hit full throttle.
const float kLinearAccelLimit = 2.0f;  // Meters per second per second
const float kAngularAccelLimit = 4.0f; // Radians per second per second

// The Sabertooth controller has two output channels: Motor 1 and Motor 2.
static const uint8_t kMotor1 = 1;
static const uint8_t kMotor2 = 2;

// ==========================================
//        INTERNAL HELPER FUNCTIONS
// ==========================================

// TOOL 1: Number Clamper
// The Sabertooth controller requires a number between 0 and 127.
// This function ensures we never accidentally send 128 or -1.
static uint8_t clamp_7bit(int value) {
  if (value < 0) return 0;
  if (value > 127) return 127;
  return static_cast<uint8_t>(value);
}

// TOOL 2: Speed Converter
// We think in percentages (-1.0 is full reverse, 1.0 is full forward).
// The Sabertooth thinks in numbers (-127 to 127).
// This translates our language to the motor's language.
static int speed_from_normalized(float speed) {
  if (speed > 1.0f) speed = 1.0f;
  if (speed < -1.0f) speed = -1.0f;
  return static_cast<int>(speed * 127.0f);
}

// TOOL 3: Time-Based Smoother
// This calculates the new speed based on how much TIME has passed.
// current: The speed we are going right now.
// target:  The speed the laptop WANTS us to go.
// max_rate: How fast we are allowed to change speed (per second).
// dt:      How many seconds have passed since the last update.
static float ramp_value_time(float current, float target, float max_rate, float dt) {
  float error = target - current;
  float max_step = max_rate * dt; // The biggest jump we are allowed to make right now

  if (error > max_step) return current + max_step;  // Accelerate Forward
  if (error < -max_step) return current - max_step; // Accelerate Backward
  return target; // We are close enough, just go to the target
}

// TOOL 4: The Messenger
// Sends the data byte to the motor controller with a Checksum.
static void sabertooth_write(uint8_t address, uint8_t command, uint8_t data) {
  uint8_t checksum = (address + command + data) & 0x7F; 
  Serial1.write(address);
  Serial1.write(command);
  Serial1.write(data);
  Serial1.write(checksum);
}

// TOOL 5: Motor Commander
// Sends a command to a specific motor.
static void sabertooth_set_motor(uint8_t address, uint8_t motor, int speed) {
  if (motor != kMotor1 && motor != kMotor2) return;
  
  // 0 & 4 = Forward. 1 & 5 = Backward.
  uint8_t command_forward = (motor == kMotor1) ? 0 : 4;
  uint8_t command_backward = (motor == kMotor1) ? 1 : 5;

  if (speed >= 0) {
    sabertooth_write(address, command_forward, clamp_7bit(speed));
  } else {
    sabertooth_write(address, command_backward, clamp_7bit(-speed));
  }
}

// ==========================================
//           MAIN DRIVE FUNCTIONS
// ==========================================

void drive_init() {
  Serial1.begin(kSabertoothBaud);
  last_ramp_time = millis(); // Start the clock
}

// SAFETY STOP
void drive_stop() {
  sabertooth_set_motor(kSabertoothFrontAddr, kMotor1, 0);
  sabertooth_set_motor(kSabertoothFrontAddr, kMotor2, 0);
  sabertooth_set_motor(kSabertoothRearAddr, kMotor1, 0);
  sabertooth_set_motor(kSabertoothRearAddr, kMotor2, 0);
  
  current_linear_speed = 0.0f;
  current_angular_speed = 0.0f;
}

// THE MAIN LOGIC
void drive_command(float target_linear, float target_angular) {
  
  // 1. Calculate Time Passed (dt)
  // We need to know if it's been 0.01 seconds or 0.1 seconds since the last command
  // so we can calculate exactly how much to increase speed.
  unsigned long now = millis();
  float dt = (now - last_ramp_time) / 1000.0f; // Convert milliseconds to seconds
  last_ramp_time = now;

  // Safety: If the time gap is huge (first run or lag), pretend it's small
  // so the robot doesn't jump unexpectedly.
  if (dt > 0.1f) dt = 0.1f;

  // 2. Cap inputs to max limits
  if (target_linear > kMaxLinearMps) target_linear = kMaxLinearMps;
  if (target_linear < -kMaxLinearMps) target_linear = -kMaxLinearMps;
  if (target_angular > kMaxAngularRadps) target_angular = kMaxAngularRadps;
  if (target_angular < -kMaxAngularRadps) target_angular = -kMaxAngularRadps;

  // 3. Smooth the Speed (Ramping)
  current_linear_speed = ramp_value_time(current_linear_speed, target_linear, kLinearAccelLimit, dt);
  current_angular_speed = ramp_value_time(current_angular_speed, target_angular, kAngularAccelLimit, dt);

  // 4. Differential Drive Math (The Mixing)
  // Left = Forward - Turn
  // Right = Forward + Turn
  float left_mps = current_linear_speed - (current_angular_speed * kTrackWidthMeters * 0.5f);
  float right_mps = current_linear_speed + (current_angular_speed * kTrackWidthMeters * 0.5f);

  // 5. Convert to Percentages
  float left_norm = left_mps / kMaxLinearMps;
  float right_norm = right_mps / kMaxLinearMps;

  // 6. Convert to Sabertooth Integers
  int left_cmd = speed_from_normalized(left_norm);
  int right_cmd = speed_from_normalized(right_norm);

  // 7. Send to Hardware
  
  // Front Axle
  sabertooth_set_motor(kSabertoothFrontAddr, kMotor1, left_cmd);
  sabertooth_set_motor(kSabertoothFrontAddr, kMotor2, right_cmd);
  
  // Rear Axle
  sabertooth_set_motor(kSabertoothRearAddr, kMotor1, left_cmd);
  sabertooth_set_motor(kSabertoothRearAddr, kMotor2, right_cmd);
}