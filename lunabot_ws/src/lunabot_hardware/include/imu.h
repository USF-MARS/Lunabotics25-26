#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/support.h>

void imu_init(rcl_node_t *node, rclc_executor_t *executor, rclc_support_t *support);
void imu_update();

#endif
