import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description_share = get_package_share_directory('lunabot_description')
    bringup_share = get_package_share_directory('lunabot_bringup')

    default_model_path = os.path.join(description_share, 'urdf', 'lunabot.urdf.xacro')
    default_rviz_config = os.path.join(bringup_share, 'config', 'lunabot_bringup.rviz')
    controllers_config = os.path.join(bringup_share, 'config', 'lunabot_controllers.yaml')

    model = LaunchConfiguration('model')
    rviz_config = LaunchConfiguration('rviz_config')
    use_rviz = LaunchConfiguration('use_rviz')

    robot_description = Command(['xacro ', model])

    return LaunchDescription([
        DeclareLaunchArgument(
            name='model',
            default_value=default_model_path,
            description='Absolute path to robot urdf file',
        ),
        DeclareLaunchArgument(
            name='rviz_config',
            default_value=default_rviz_config,
            description='Absolute path to rviz config file',
        ),
        DeclareLaunchArgument(
            name='use_rviz',
            default_value='true',
            description='Whether to launch RViz2',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            output='screen',
            parameters=[
                {'robot_description': robot_description},
                controllers_config,
            ],
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller', '-c', '/controller_manager'],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(use_rviz),
        ),
    ])
