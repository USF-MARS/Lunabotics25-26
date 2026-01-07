# 🚦 Arbitration Node

**"The Traffic Cop of the Rover"**

### What is this?
The Arbitration Node is the safety center of the robot's software. It sits between the "Brains" (Autonomous Navigation, Foxglove Teleop) and the "Muscles" (The Teensy microcontroller that spins the wheels).

Its job is to decide **who gets to drive** at any given millisecond. Without this node, if the Autonomous code tried to drive forward while a human tried to drive backward, the robot would get confused, jitter, or crash.

### 🧠 The Logic (The 4 Rules)

This node follows a strict hierarchy of safety features. It checks these rules 10 times every second:

**1. 🛑 EMERGENCY STOP (Priority #1)**
* **Trigger:** The big red STOP button on the controller (or Foxglove).
* **Behavior:** The robot freezes immediately. All other commands are blocked.
* **Reset:** The robot stays frozen for at least 1 second after the stop command is released to ensure stability.

**2. 🎮 Manual Override (Priority #2)**
* **Trigger:** Moving the joystick on the controller.
* **Behavior:** If a human touches the controls, the robot ignores the autonomous code. "Human knows best."
* **Reset:** If you let go of the joystick for **1 second**, the robot allows the autonomous code to take over again.

**3. 🤖 Autonomous Mode (Priority #3)**
* **Trigger:** Flipping the "Auto Mode" switch on the dashboard.
* **Behavior:** The Nav2 stack or Digging Manager is allowed to drive the rover.
* **Condition:** This only works if **Rule 1** and **Rule 2** are not active.

**4. ☠️ Dead Man's Switch (Failsafe)**
* **Trigger:** If the software crashes or Wi-Fi disconnects (no commands received for 0.5 seconds).
* **Behavior:** The node automatically sends a "0 Velocity" command to the wheels. This prevents the rover from "ghost riding" if the laptop freezes.

### 📡 Topic Map

| Direction | Topic Name | Type | Description |
| :--- | :--- | :--- | :--- |
| **INPUT** | `/cmd_vel_teleop` | Twist | Joystick driving commands |
| **INPUT** | `/cmd_vel_nav` | Twist | Autonomous driving commands |
| **INPUT** | `/cmd_STOP_teleop` | Twist | Emergency Stop signal |
| **INPUT** | `/mode_switch` | Bool | Toggle Auto Mode (True = Auto) |
| **OUTPUT** | `/cmd_vel` | Twist | Final command sent to Teensy |
| **OUTPUT** | `/cmd_STOP` | Twist | E-Stop signal passed to Teensy |

### 🚀 How to Run

Make sure you have built the workspace first:

```bash
cd ~/lunabotics_ws
colcon build --packages-select arbitration_node
source install/setup.bash
