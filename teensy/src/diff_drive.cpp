#include "diff_drive.h"

// INTERNAL MEMORY:
// We need to remember how fast we were going 10 milliseconds ago 
// so we can calculate the "ramp" (smooth acceleration).
static float current_linear_speed = 0.0f;
static float current_angular_speed = 0.0f;

// The Sabertooth controller has two output channels: Motor 1 and Motor 2.
static const uint8_t kMotor1 = 1;
static const uint8_t kMotor2 = 2;

// ==========================================
//        INTERNAL HELPER FUNCTIONS
// ==========================================
// (These are "private" tools used only inside this file)

// TOOL 1: Number Clamper
// The Sabertooth controller requires a number between 0 and 127.
// This function ensures we never accidentally send 128 or -1, which would confuse it.
static uint8_t clamp_7bit(int value) {
  if (value < 0) return 0;
  if (value > 127) return 127;
  return static_cast<uint8_t>(value);
}

// TOOL 2: Speed Converter
// We think in percentages (-1.0 is full reverse, 1.0 is full forward).
// The Sabertooth thinks in numbers (-127 to 127).
// This function translates our language to the motor's language.
static int speed_from_normalized(float speed) {
  // Hard limit to ensure we never exceed 100%
  if (speed > 1.0f) speed = 1.0f;
  if (speed < -1.0f) speed = -1.0f;
  return static_cast<int>(speed * 127.0f);
}

// TOOL 3: The "Soft Start" Logic
// This compares where we ARE (current) vs where we WANT to be (target).
// If the difference is too big, it only takes a small step (step) towards the target.
// This prevents the robot from doing a wheelie or stripping gears.
static float ramp_value(float current, float target, float step) {
  float difference = target - current;
  
  // Accelerating forward too fast? Limit it.
  if (difference > step) return current + step;
  
  // Accelerating backward too fast? Limit it.
  if (difference < -step) return current - step;
  
  // If the change is small/safe, just go there immediately.
  return target;
}

// TOOL 4: The Messenger
// This sends the actual data byte to the motor controller.
// It calculates a "Checksum" (a math check) so the controller knows the message didn't get corrupted by static noise.
static void sabertooth_write(uint8_t address, uint8_t command, uint8_t data) {
  uint8_t checksum = (address + command + data) & 0x7F; // Required math by Sabertooth
  Serial1.write(address);
  Serial1.write(command);
  Serial1.write(data);
  Serial1.write(checksum);
}

// TOOL 5: Motor Commander
// Sends a command to a specific motor (Left or Right) on a specific board (Front or Rear).
static void sabertooth_set_motor(uint8_t address, uint8_t motor, int speed) {
  if (motor != kMotor1 && motor != kMotor2) return;
  
  // The Sabertooth has different command codes for "Drive Forward" vs "Drive Backward"
  // 0 & 4 = Forward. 1 & 5 = Backward.
  uint8_t command_forward = (motor == kMotor1) ? 0 : 4;
  uint8_t command_backward = (motor == kMotor1) ? 1 : 5;

  if (speed >= 0) {
    sabertooth_write(address, command_forward, clamp_7bit(speed));
  } else {
    // If speed is negative, we make it positive and send the "Backward" command code
    sabertooth_write(address, command_backward, clamp_7bit(-speed));
  }
}

// ==========================================
//           MAIN DRIVE FUNCTIONS
// ==========================================

void drive_init() {
  Serial1.begin(kSabertoothBaud);
}

// SAFETY STOP
void drive_stop() {
  // Send "0 speed" to all 4 motors immediately.
  sabertooth_set_motor(kSabertoothFrontAddr, kMotor1, 0);
  sabertooth_set_motor(kSabertoothFrontAddr, kMotor2, 0);
  sabertooth_set_motor(kSabertoothRearAddr, kMotor1, 0);
  sabertooth_set_motor(kSabertoothRearAddr, kMotor2, 0);
  
  // Important: Forget our previous speed so we don't accidentally "resume" moving later.
  current_linear_speed = 0.0f;
  current_angular_speed = 0.0f;
}

// THE MAIN LOGIC
void drive_command(float target_linear, float target_angular) {
  
  // STEP 1: Safety Check
  // If the laptop asks for 1000 mph, we cap it at our max speed.
  if (target_linear > kMaxLinearMps) target_linear = kMaxLinearMps;
  if (target_linear < -kMaxLinearMps) target_linear = -kMaxLinearMps;
  if (target_angular > kMaxAngularRadps) target_angular = kMaxAngularRadps;
  if (target_angular < -kMaxAngularRadps) target_angular = -kMaxAngularRadps;

  // STEP 2: Smooth the inputs (Ramping)
  // Instead of jumping instantly to the target, we step towards it.
  current_linear_speed = ramp_value(current_linear_speed, target_linear, kRampStep);
  current_angular_speed = ramp_value(current_angular_speed, target_angular, kRampStep);

  // STEP 3: "Mixing" (Differential Drive Math)
  // How do we turn? We spin the left wheels slower and right wheels faster (or vice versa).
  // Left Speed  = Forward Speed - Turning Speed
  // Right Speed = Forward Speed + Turning Speed
  float left_mps = current_linear_speed - (current_angular_speed * kTrackWidthMeters * 0.5f);
  float right_mps = current_linear_speed + (current_angular_speed * kTrackWidthMeters * 0.5f);

  // STEP 4: Convert m/s to Percentage
  float left_norm = left_mps / kMaxLinearMps;
  float right_norm = right_mps / kMaxLinearMps;

  // STEP 5: Convert Percentage to Sabertooth integers (-127 to 127)
  int left_cmd = speed_from_normalized(left_norm);
  int right_cmd = speed_from_normalized(right_norm);

  // STEP 6: Send the result to the physical hardware
  
  // Front Axle (Controller 128)
  sabertooth_set_motor(kSabertoothFrontAddr, kMotor1, left_cmd);
  sabertooth_set_motor(kSabertoothFrontAddr, kMotor2, right_cmd);
  
  // Rear Axle (Controller 129)
  sabertooth_set_motor(kSabertoothRearAddr, kMotor1, left_cmd);
  sabertooth_set_motor(kSabertoothRearAddr, kMotor2, right_cmd);
}
