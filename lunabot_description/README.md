## URDF / Xacro Lunabot

### Installation
#### gazebo 
sudo apt-get install ros-${ROS_DISTRO}-ros-gz

#### joint_state_publisher
sudo apt install ros-jazzy-joint-state-publisher-gui
#### dynamic transforms
`sudo apt update
sudo apt install \
  ros-jazzy-tf2 \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-tools \
  ros-jazzy-robot-state-publisher
`


### What Is a URDF?
A URDF (Unified Robot Description Format) file is an XML-based description of a robot’s physical structure. ROS uses it to understand how the robot is connected and how data like transforms, sensors, and controllers relate to each other.

### 1. Conceptual Model Only
- This file exists to explain **structure**, not simulate real physics.
- Dimensions and masses are approximate and chosen for clarity and stability.
- Real dynamics are handled in controllers, not here.

### 2. `base_link` Is a Pure Reference Frame
- `base_link` has **no geometry and no mass**.
- It exists solely as the root of the TF tree.
- This follows standard ROS conventions and avoids frame ambiguity later.

### 3. 4WD Skid-Steer Assumption
- The rover uses **four independently driven wheels**.
- There are **no steering joints**.
- Turning is achieved by running left and right wheels at different speeds.
- Any differential behavior is handled in software, not in URDF.

### 4. Wheel Geometry Assumptions
- Wheel diameter is assumed to be ~10 inches.
- Wheel width is simplified to 0.1 m.
- All four wheels are identical for clarity and reuse.

### 5. Mass Values Are Not Physically Accurate
- Chassis mass is set to 20 kg.
- Each wheel is set to 2 kg.
- These values are placeholders to keep simulators numerically stable.

### 6. No Excavation or Sensor Geometry Yet
- Excavator arms, bucket, actuators, and vibration mechanisms are **not included**.
- Cameras, IMU, Jetson, and other electronics are also excluded.
- These will be added later as separate xacro files.

### 7. Coordinate Frame Conventions
- X axis points forward.
- Y axis points to the left.
- Z axis points upward.
- Wheels are rotated so they spin around the correct axis per ROS standards.

### 8. Single-File Teaching Focus
- This file is designed to be readable in one pass.
- The goal is onboarding and conceptual clarity.
- Future versions may split the model into modular xacro files.
