"""Rover bringup: Foxglove bridge, micro-ROS agent, teleop, joy -> /mode_switch.

Stack intent (see lunabot_arbitration and README there):
- Menu (joy_mode_switch) toggles /mode_switch so the operator can enter automation when arbitration
  and Nav2 are wired to /cmd_vel_nav and teleop uses *_teleop topics.
- Emergency stop and zeros: configure Foxglove (or teleop) to publish geometry_msgs/Twist on
  /cmd_STOP_teleop; arbitration forwards stop and zeros /cmd_vel.
- Connection loss: arbitration failsafe + Teensy cmd_vel watchdog; ensure arbitration is launched
  when using the full safety graph.

This launch does not yet start lunabot_arbitration; add that node when teleop is remapped to *_teleop.
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 1. Foxglove Bridge (Including the XML launch file)
    foxglove_bridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory('foxglove_bridge'), 
                         'launch', 'foxglove_bridge_launch.xml')
        )
    )

    # 2. Micro-ROS Agent (Running as a process)
    microros_agent = ExecuteProcess(
        cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial', '--dev', '/dev/ttyACM0'],
        output='screen'
    )

    # 3. Your Teleop Node
    teleop_node = Node(
        package='lunabot_teleop',
        executable='joy_to_cmd_vel',
        name='joy_to_cmd_vel',
        output='screen'
    )

    # 4. Menu button -> /mode_switch (for lunabot_arbitration)
    joy_mode_switch = Node(
        package='lunabot_teleop',
        executable='joy_mode_switch',
        name='joy_mode_switch',
        output='screen'
    )

    return LaunchDescription([
        foxglove_bridge,
        microros_agent,
        teleop_node,
        joy_mode_switch
    ])
