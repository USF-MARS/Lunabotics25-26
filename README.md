# Lunabotics25-26

welcome to University of South Florida's Mechatronics & Robotic Systems' first NASA Lunabotics 2025 - 2026 Competition

This repo will be used for ROS2 Production
Drivetrain system
System Overview
Rover mass (target): ~55 kg
Drive: 4WD, independent wheel drives
Motors: 12 V CIM brushed DC
Gear reduction: ~100:1 per wheel
Wheel diameter: ~10 in
Motor drivers: Sabertooth 2x60
Encoders: CIMcoder


Motor and Torque Analysis
CIM motor (12 V):
Free speed: ~5310 rpm
Stall torque: ~2.4 N·M
Stall current: ~130 A
With 100:1 reduction:
Wheel max torque (theoretical): ~240 N·m per wheel
Total drivetrain torque: ~960 N·m
Realistic
We are limiting current to 80A max to protect components. Underload it will pull about 40-70A. This translates to, assuming 11 inch wheels:
65-110 Nm per wheel
260-440 Nm total


This torque far exceeds available traction, ensuring the system is traction-limited, not motor-limited. T

Traction Limit (Reality Check)
Assuming μ ≈ 0.6–08:
F_max ≈ 400–500 N of horizontal pushing total
Equivalent of pushing a 100lb crate on concreate
Wheel slip occurs before motor stall
Current draw is naturally limited
Motors and controllers are protected
Other notes
Motors are externally mounted for cooling
Uses 90 degree adapter.
Torque rating of 180 Nm
Motors might reach a max of 120 Nm w/ current limiting

Excavator system
It is a two actuator system. 
Tilt actuator, tilts the top of the bucket 
Lift actuator: this lifts up the the bucket arms and the tilt actuator 
vibrator
Autonomy
Autonomy is built around an NVIDIA Jetson Orin Nano 
2× Intel RealSense depth camera
Depth perception for obstacle detection and terrain mapping
HWT905 IMU
Provides orientation, angular rate, and acceleration data
Improves state estimation and SLAM stability, especially during slip and vibration
Control
We are using a Teensy 4.1 MCU for Low-level controls such as the motors and actuators. 
It connects via USB to the jetson and uses ros2 communication
Controlled via xbox controller plugged into the dev machine
Software Stack
ROS 2 as the system framework
Nav2 for navigation and path planning
SLAM for real-time mapping and localization


Other Notes
The jetson and sensors are powered by a separate battery source to prevent potential voltage sag from browning out components and potentially cycling our computer

The rover design we are taking inspiration from

