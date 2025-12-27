from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_share = FindPackageShare('lunabot_description')
    bringup_share = FindPackageShare('lunabot_bringup')

    default_model_path = PathJoinSubstitution(
        [description_share, 'urdf', 'lunabot_ros2_control.xacro']
    )
    controllers_yaml = PathJoinSubstitution(
        [bringup_share, 'config', 'lunabot_controllers.yaml']
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name='model',
                default_value=default_model_path,
                description='Absolute path to robot xacro file',
            ),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                parameters=[
                    {
                        'robot_description': Command(
                            ['xacro ', LaunchConfiguration('model')]
                        )
                    }
                ],
            ),
            Node(
                package='controller_manager',
                executable='ros2_control_node',
                parameters=[
                    {
                        'robot_description': Command(
                            ['xacro ', LaunchConfiguration('model')]
                        )
                    },
                    controllers_yaml,
                ],
                output='screen',
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                output='screen',
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
                output='screen',
            ),
        ]
    )
