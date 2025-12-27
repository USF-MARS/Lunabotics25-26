import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description_share = get_package_share_directory('lunabot_description')
    bringup_share = get_package_share_directory('lunabot_bringup')

    default_model_path = os.path.join(
        description_share,
        'urdf',
        'lunabot.urdf.xacro',
    )
    default_controller_path = os.path.join(
        bringup_share,
        'config',
        'lunabot_controllers.yaml',
    )

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Absolute path to the robot URDF file',
    )
    controllers_arg = DeclareLaunchArgument(
        name='controllers',
        default_value=default_controller_path,
        description='Absolute path to the controller configuration file',
    )

    robot_description = Command(['xacro ', LaunchConfiguration('model')])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            LaunchConfiguration('controllers'),
        ],
        output='screen',
    )

    controller_spawners = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
            ),
        ],
    )

    return LaunchDescription([
        model_arg,
        controllers_arg,
        robot_state_publisher,
        controller_manager,
        controller_spawners,
    ])
