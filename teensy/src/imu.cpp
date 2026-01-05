#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

#include "imu.h"

// ==========================================
//          CONFIGURATION
// ==========================================

// HARDWARE SETUP:
// We use "Serial2" (Pins 7 & 8 on Teensy 4.1) so we don't crash into the motor controller.
// Baud 115200 is the default speed for the HWT905 sensor.
#define HWT_SERIAL Serial2
#define HWT_BAUD 115200       

#define LED_PIN 13

// ERROR CHECKING MACROS (The "Did it break?" checkers)
// RCCHECK: If a ROS function fails, stop everything and blink the LED.
// RCSOFTCHECK: If a ROS function fails, just ignore it and keep going.
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// ==========================================
//          GLOBAL VARIABLES
// ==========================================

// ROS Communication Objects
rcl_publisher_t publisher;       // The "Postman" that sends data to the laptop
sensor_msgs__msg__Imu * imu_msg; // The "Letter" we are writing (the data structure)
rcl_allocator_t allocator;       // Memory manager
rcl_timer_t timer;               // The clock that tells us when to send a message

// Time Synchronization
// We need to make sure the Teensy's clock matches the Laptop's clock.
unsigned long last_sync_time = 0;
const unsigned long SYNC_INTERVAL = 60000; // Re-check the time every 60 seconds

// Raw Sensor Data Storage
// "volatile" means "this value can change at any moment" (because the sensor sends data constantly).
volatile float raw_accel[3] = {0, 0, 0}; // X, Y, Z acceleration
volatile float raw_gyro[3]  = {0, 0, 0}; // X, Y, Z rotation speed
volatile float raw_angle[3] = {0, 0, 0}; // Roll, Pitch, Yaw angles

// ==========================================
//          HELPER FUNCTIONS
// ==========================================

// Panic Mode: Blinks the LED if something critical fails.
void error_loop(){
  while(1){
    digitalRead(LED_PIN) ? digitalWrite(LED_PIN, LOW) : digitalWrite(LED_PIN, HIGH);
    delay(100);
  }
}

// MATH CONVERTER: Euler Angles -> Quaternions
// Humans understand angles (Roll, Pitch, Yaw).
// Robots understand "Quaternions" (X, Y, Z, W).
// This function translates Human -> Robot.
void euler_to_quat(float roll, float pitch, float yaw, double* q) {
    float c1 = cos((yaw * 3.14159 / 180.0) / 2);
    float c2 = cos((pitch * 3.14159 / 180.0) / 2);
    float c3 = cos((roll * 3.14159 / 180.0) / 2);

    float s1 = sin((yaw * 3.14159 / 180.0) / 2);
    float s2 = sin((pitch * 3.14159 / 180.0) / 2);
    float s3 = sin((roll * 3.14159 / 180.0) / 2);

    q[0] = c1 * c2 * c3 - s1 * s2 * s3; // W (The scalar part)
    q[1] = s1 * s2 * c3 + c1 * c2 * s3; // X
    q[2] = s1 * c2 * c3 + c1 * s2 * s3; // Y
    q[3] = c1 * s2 * c3 - s1 * c2 * s3; // Z
}

// SENSOR DRIVER: The "Ear"
// This listens to the wire coming from the IMU sensor.
// It looks for a specific pattern of bytes (starting with 0x55).
void read_imu() {
  // While there is data waiting in the buffer (at least 11 bytes, which is one full packet)...
  while (HWT_SERIAL.available() >= 11) {
    
    // 1. Check for the "Header" byte (0x55). If it's not there, skip a byte and try again.
    if (HWT_SERIAL.peek() != 0x55) {
      HWT_SERIAL.read(); 
      continue;
    }
    
    // 2. Read the full packet (11 bytes)
    uint8_t buffer[11];
    HWT_SERIAL.readBytes(buffer, 11);
    
    // 3. CHECK THE SUM (Data Integrity)
    // We add up the first 10 bytes. The result MUST match the 11th byte.
    // If it doesn't, the data is corrupted (static noise), so we throw it away.
    uint8_t sum = 0;
    for(int i=0; i<10; i++) sum += buffer[i];
    
    if(sum != buffer[10]) continue; 

    // 4. Decode the data based on the "Type" byte
    uint8_t type = buffer[1];
    
    if (type == 0x51) { // It's Acceleration Data
       // Math converts the raw binary bits into "g-force" units
       raw_accel[0] = (short)(buffer[3] << 8 | buffer[2]) / 32768.0 * 16.0;
       raw_accel[1] = (short)(buffer[5] << 8 | buffer[4]) / 32768.0 * 16.0;
       raw_accel[2] = (short)(buffer[7] << 8 | buffer[6]) / 32768.0 * 16.0;
    } 
    else if (type == 0x52) { // It's Gyroscope Data
       // Math converts bits into "degrees per second"
       raw_gyro[0] = (short)(buffer[3] << 8 | buffer[2]) / 32768.0 * 2000.0;
       raw_gyro[1] = (short)(buffer[5] << 8 | buffer[4]) / 32768.0 * 2000.0;
       raw_gyro[2] = (short)(buffer[7] << 8 | buffer[6]) / 32768.0 * 2000.0;
    } 
    else if (type == 0x53) { // It's Angle Data (Roll/Pitch/Yaw)
       // Math converts bits into "degrees"
       raw_angle[0] = (short)(buffer[3] << 8 | buffer[2]) / 32768.0 * 180.0;
       raw_angle[1] = (short)(buffer[5] << 8 | buffer[4]) / 32768.0 * 180.0;
       raw_angle[2] = (short)(buffer[7] << 8 | buffer[6]) / 32768.0 * 180.0;
    }
  }
}

// ==========================================
//          ROS CALLBACK
// ==========================================
// This runs automatically 50 times a second (every 20ms).
// It packages the data and sends it to the laptop.
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  RCLC_UNUSED(timer);

  // 1. Get the current time (Synchronized with Laptop)
  struct timespec tv = {0};
  clock_gettime(CLOCK_REALTIME, &tv);

  // Stamp the message so the laptop knows EXACTLY when this happened
  imu_msg->header.stamp.sec = tv.tv_sec;
  imu_msg->header.stamp.nanosec = tv.tv_nsec;

  // 2. Convert Angles to Quaternions
  // Note: We invert Z (-1.0) because the IMU follows a different "North" convention than ROS.
  double q[4];
  euler_to_quat(raw_angle[0], raw_angle[1], -1.0 * raw_angle[2], q);
  
  imu_msg->orientation.w = q[0];
  imu_msg->orientation.x = q[1];
  imu_msg->orientation.y = q[2];
  imu_msg->orientation.z = q[3];

  // 3. Pack Gyro Data (Degrees -> Radians)
  // We also invert Z here to match the angle inversion.
  imu_msg->angular_velocity.x = raw_gyro[0] * (3.14159 / 180.0);
  imu_msg->angular_velocity.y = raw_gyro[1] * (3.14159 / 180.0);
  imu_msg->angular_velocity.z = -1.0 * raw_gyro[2] * (3.14159 / 180.0);

  // 4. Pack Acceleration Data (g -> m/s^2)
  imu_msg->linear_acceleration.x = raw_accel[0] * 9.80665;
  imu_msg->linear_acceleration.y = raw_accel[1] * 9.80665;
  imu_msg->linear_acceleration.z = raw_accel[2] * 9.80665;

  // 5. Send the message!
  RCSOFTCHECK(rcl_publish(&publisher, imu_msg, NULL));
}

// ==========================================
//          INITIALIZATION
// ==========================================
// This runs once when the robot turns on.
void imu_init(rcl_node_t *node, rclc_executor_t *executor, rclc_support_t *support) {
  HWT_SERIAL.begin(HWT_BAUD);
  delay(1000); // Give the sensor a second to wake up

  // --- WAIT FOR SYNC ---
  // We refuse to start until the Laptop talks to us.
  // This ensures our timestamps are correct from the very first message.
  const int timeout_ms = 1000;
  while (rmw_uros_sync_session(timeout_ms) != RMW_RET_OK) {
    // Flash LED while waiting...
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
  digitalWrite(LED_PIN, HIGH); // Solid light = Connected!

  allocator = rcl_get_default_allocator();

  // Create the Publisher (Topic: "/imu/data")
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "/imu/data"));

  // Create the Timer (Trigger "timer_callback" every 20ms)
  RCCHECK(rclc_timer_init_default(
    &timer,
    support,
    RCL_MS_TO_NS(20),
    timer_callback));

  RCCHECK(rclc_executor_add_timer(executor, &timer));

  // Allocate memory for the message structure
  if (!micro_ros_utilities_create_message_memory(
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        &imu_msg,
        (micro_ros_utilities_memory_conf_t) {})
    ) {
    error_loop();
  }

  // Set the "Frame ID" (The name of the sensor in the 3D model)
  imu_msg->header.frame_id = micro_ros_string_utilities_set(imu_msg->header.frame_id, "imu_link");

  // Set Covariances (Uncertainty values)
  // We use 0.01 to say "We are pretty sure this data is accurate"
  for (int i = 0; i < 9; i++) {
    imu_msg->orientation_covariance[i] = 0.0;
    imu_msg->angular_velocity_covariance[i] = 0.0;
    imu_msg->linear_acceleration_covariance[i] = 0.0;
  }
  imu_msg->orientation_covariance[0] = 0.01;
  imu_msg->orientation_covariance[4] = 0.01;
  imu_msg->orientation_covariance[8] = 0.01;
  imu_msg->angular_velocity_covariance[0] = 0.01;
  imu_msg->angular_velocity_covariance[4] = 0.01;
  imu_msg->angular_velocity_covariance[8] = 0.01;
  imu_msg->linear_acceleration_covariance[0] = 0.01;
  imu_msg->linear_acceleration_covariance[4] = 0.01;
  imu_msg->linear_acceleration_covariance[8] = 0.01;
}

// ==========================================
//          MAIN UPDATE LOOP
// ==========================================
// This is called inside loop() in the main file.
void imu_update() {
  // 1. Read any new data from the sensor wire
  read_imu();

  // 2. Check clock drift
  // Computer clocks drift over time. Every 60 seconds, we re-sync with the laptop
  // to make sure our timestamps stay accurate.
  if (millis() - last_sync_time > SYNC_INTERVAL) {
    last_sync_time = millis();
    rmw_uros_sync_session(100);
  }
}