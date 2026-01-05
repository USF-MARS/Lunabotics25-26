#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

// --- CONFIGURATION ---
#define HWT_SERIAL Serial1
#define HWT_BAUD 115200       
#define LED_PIN 13
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// --- ROS VARIABLES ---
rcl_publisher_t publisher;       
sensor_msgs__msg__Imu * imu_msg;
rclc_executor_t executor;        
rclc_support_t support;           
rcl_allocator_t allocator;        
rcl_node_t node;                  
rcl_timer_t timer;                

// --- SYNC VARIABLES ---
unsigned long last_sync_time = 0;
const unsigned long SYNC_INTERVAL = 60000; // Re-sync every 60 seconds

// --- RAW DATA STORAGE ---
volatile float raw_accel[3] = {0, 0, 0}; 
volatile float raw_gyro[3]  = {0, 0, 0}; 
volatile float raw_angle[3] = {0, 0, 0}; 

// Function headers
void error_loop();
void read_imu(); 

void error_loop(){
  while(1){
    digitalRead(LED_PIN) ? digitalWrite(LED_PIN, LOW) : digitalWrite(LED_PIN, HIGH);
    delay(100);
  }
}

// Helper: Euler to Quaternion
void euler_to_quat(float roll, float pitch, float yaw, double* q) {
    float c1 = cos((yaw * 3.14159 / 180.0) / 2);
    float c2 = cos((pitch * 3.14159 / 180.0) / 2);
    float c3 = cos((roll * 3.14159 / 180.0) / 2);

    float s1 = sin((yaw * 3.14159 / 180.0) / 2);
    float s2 = sin((pitch * 3.14159 / 180.0) / 2);
    float s3 = sin((roll * 3.14159 / 180.0) / 2);

    q[0] = c1 * c2 * c3 - s1 * s2 * s3; // W
    q[1] = s1 * s2 * c3 + c1 * c2 * s3; // X
    q[2] = s1 * c2 * c3 + c1 * s2 * s3; // Y
    q[3] = c1 * s2 * c3 - s1 * c2 * s3; // Z
}

// IMU Driver (With Checksum Fixes from previous review)
void read_imu() {
  while (HWT_SERIAL.available() >= 11) {
    if (HWT_SERIAL.peek() != 0x55) {
      HWT_SERIAL.read(); 
      continue;
    }
    
    uint8_t buffer[11];
    HWT_SERIAL.readBytes(buffer, 11);
    
    uint8_t sum = 0;
    for(int i=0; i<10; i++) sum += buffer[i];
    
    if(sum != buffer[10]) continue; 

    uint8_t type = buffer[1];
    if (type == 0x51) { // ACCEL
       raw_accel[0] = (short)(buffer[3] << 8 | buffer[2]) / 32768.0 * 16.0;
       raw_accel[1] = (short)(buffer[5] << 8 | buffer[4]) / 32768.0 * 16.0;
       raw_accel[2] = (short)(buffer[7] << 8 | buffer[6]) / 32768.0 * 16.0;
    } 
    else if (type == 0x52) { // GYRO
       raw_gyro[0] = (short)(buffer[3] << 8 | buffer[2]) / 32768.0 * 2000.0;
       raw_gyro[1] = (short)(buffer[5] << 8 | buffer[4]) / 32768.0 * 2000.0;
       raw_gyro[2] = (short)(buffer[7] << 8 | buffer[6]) / 32768.0 * 2000.0;
    } 
    else if (type == 0x53) { // ANGLE
       raw_angle[0] = (short)(buffer[3] << 8 | buffer[2]) / 32768.0 * 180.0;
       raw_angle[1] = (short)(buffer[5] << 8 | buffer[4]) / 32768.0 * 180.0;
       raw_angle[2] = (short)(buffer[7] << 8 | buffer[6]) / 32768.0 * 180.0;
    }
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  RCLC_UNUSED(timer);

  struct timespec tv = {0};
  
  // NEW: Use CLOCK_REALTIME to get the synchronized agent time
  // If sync hasn't happened yet, this falls back to boot time
  clock_gettime(CLOCK_REALTIME, &tv);

  imu_msg->header.stamp.sec = tv.tv_sec;
  imu_msg->header.stamp.nanosec = tv.tv_nsec;

  // --- POPULATE DATA ---
  double q[4];
  euler_to_quat(raw_angle[0], raw_angle[1], -1.0 * raw_angle[2], q);
  
  imu_msg->orientation.w = q[0];
  imu_msg->orientation.x = q[1];
  imu_msg->orientation.y = q[2];
  imu_msg->orientation.z = q[3];

  imu_msg->angular_velocity.x = raw_gyro[0] * (3.14159 / 180.0);
  imu_msg->angular_velocity.y = raw_gyro[1] * (3.14159 / 180.0);
  imu_msg->angular_velocity.z = -1.0 * raw_gyro[2] * (3.14159 / 180.0);

  imu_msg->linear_acceleration.x = raw_accel[0] * 9.80665;
  imu_msg->linear_acceleration.y = raw_accel[1] * 9.80665;
  imu_msg->linear_acceleration.z = raw_accel[2] * 9.80665;

  RCSOFTCHECK(rcl_publish(&publisher, imu_msg, NULL));
}

void setup() {
  set_microros_transports(); 
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); 
  
  HWT_SERIAL.begin(HWT_BAUD);
  delay(1000); 

  // --- NEW: INITIAL TIME SYNC ---
  // Block until we get a valid time from the Agent.
  // Crucial for Robot Localization to accept the first messages.
  const int timeout_ms = 1000;
  while(rmw_uros_sync_session(timeout_ms) != RMW_RET_OK) {
     // Flash LED to indicate we are waiting for Agent connection
     digitalWrite(LED_PIN, !digitalRead(LED_PIN));
     delay(100);
  }
  digitalWrite(LED_PIN, HIGH); // Solid light = Connected & Synced
  // -----------------------------

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "teensy_imu_node", "", &support));

  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "/imu/data"));

  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(20),
    timer_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  if(!micro_ros_utilities_create_message_memory(
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      &imu_msg,
      (micro_ros_utilities_memory_conf_t) {})
    )
  {
    error_loop();
  }

  imu_msg->header.frame_id = micro_ros_string_utilities_set(imu_msg->header.frame_id, "imu_link");
  
  // Set Covariances
  for(int i=0; i<9; i++) {
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

void loop() {
  // 1. Process Sensor Data
  read_imu(); 

  // 2. Periodic Time Sync
  // Crystals drift. We re-sync every 60 seconds.
  // We use the millis() timer so we don't block the loop unnecessarily.
  if (millis() - last_sync_time > SYNC_INTERVAL) {
      last_sync_time = millis();
      // Attempt to sync. If it fails, we keep going with the old offset.
      rmw_uros_sync_session(100); 
  }

  // 3. Handle ROS communication
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1))); 
}