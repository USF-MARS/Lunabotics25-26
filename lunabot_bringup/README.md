# Lunabot bringup

## Step-by-step bringup

1. Launch the bringup file (robot state publisher + ros2_control + controllers):

   ```bash
   ros2 launch lunabot_bringup bringup.launch.py
   ```

2. Start RViz2 in a separate terminal:

   ```bash
   ros2 run rviz2 rviz2 -d "$(ros2 pkg prefix lunabot_description)/share/lunabot_description/config/default.rviz"
   ```

3. Verify the controller manager and controllers are active:

   ```bash
   ros2 control list_controllers
   ros2 control list_hardware_interfaces
   ```

## Teleop usage

Use keyboard teleop to drive the rover (publishes to `/cmd_vel`):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
```

### Expected topics

* `/cmd_vel`
* `/joint_states`
