from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
#sdfa
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

    return LaunchDescription([
        foxglove_bridge,
        microros_agent,
        teleop_node
    ])
