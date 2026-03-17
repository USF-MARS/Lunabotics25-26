#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/support.h>

void encoder_init(rcl_node_t *node, rclc_executor_t *executor, rclc_support_t *support);
void encoder_update();

#endif
