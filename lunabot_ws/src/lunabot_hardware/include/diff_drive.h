#ifndef DIFF_DRIVE_H
#define DIFF_DRIVE_H

#include <Arduino.h>

// ==========================================
//          ROBOT PHYSICAL SETTINGS
// ==========================================

// MEASURE THIS ON THE REAL ROBOT:
// The distance from the center of the left tires to the center of the right tires.
// If this is wrong, the robot won't turn exactly 90 degrees when you tell it to.
const float kTrackWidthMeters = 0.62f;

// SPEED LIMITS:
// 1.5 m/s is about walking pace. 
// If you set this higher, the math will allow the motors to spin faster.
const float kMaxLinearMps = 1.5f;        

// TURNING SPEED LIMIT:
// How fast it is allowed to spin in circles (Radians per second).
const float kMaxAngularRadps = 2.0f;     

// SOFT START / SMOOTHING (Very Important for 55kg robots!)
// This controls how quickly the robot is allowed to change speed.
// 0.15 = Smooth, like a heavy truck.
// 0.50 = Jerky, like a sports car (dangerous for gears).
// 0.05 = Very slow acceleration (safe but sluggish).
const float kRampStep = 0.15f; 

// ==========================================
//       MOTOR CONTROLLER SETTINGS
// ==========================================

// COMMUNICATION SPEED:
// We use 38400 because it is fast enough to be responsive.
// IMPORTANT: You must change the physical DIP switches on the Sabertooth to match this!
const uint32_t kSabertoothBaud = 38400;  

// MOTOR CONTROLLER ADDRESSES:
// We have two controllers sharing one wire. We give them ID numbers so we can talk to them separately.
// 128 is the default "Front" controller.
// 129 is the "Rear" controller (requires changing DIP switches on that unit).
const uint8_t kSabertoothFrontAddr = 128;
const uint8_t kSabertoothRearAddr = 129;

// ==========================================
//           AVAILABLE COMMANDS
// ==========================================
// These are the "buttons" the main program can press.

// 1. STARTUP: Turns on the communication line to the motors.
void drive_init();

// 2. MOVE: Tells the robot where to go.
//    linear_x:  How fast to go Forward/Back (-1.5 to 1.5)
//    angular_z: How fast to Turn Left/Right (-2.0 to 2.0)
//    (This function handles all the math to mix these two inputs together)
void drive_command(float linear_x, float angular_z);

// 3. EMERGENCY STOP: Cuts power to motors instantly. No smoothing.
void drive_stop();

#endif