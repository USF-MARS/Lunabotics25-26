#ifndef ACTUATOR_H
#define ACTUATOR_H

#include <Arduino.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/support.h>

// 1. SETUP: Sets up the pins and ROS topics for BOTH actuators.
void actuator_init(rcl_node_t *node, rclc_executor_t *executor, rclc_support_t *support);

// 2. LOOP: Reads sensors and moves both motors to their targets.
void actuator_update();

#endif