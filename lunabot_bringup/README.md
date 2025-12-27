# Lunabot Bringup

## Robot State Publisher

```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro /home/ian/Lunabotics25-26/lunabot_description/urdf/lunabot.urdf.xacro)"
```

## Teleop

If you want to drive the robot manually, you can publish velocity commands with
`teleop_twist_keyboard`:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cmd_vel
```

## Diff Drive Controller Interfaces

The `diff_drive_controller` expects velocity commands on `/cmd_vel`.
It publishes odometry using the following frame IDs:

- `odom` (odom_frame_id)
- `base_link` (base_frame_id)

These frame names are configured in `lunabot_bringup/config/lunabot_controllers.yaml`.
