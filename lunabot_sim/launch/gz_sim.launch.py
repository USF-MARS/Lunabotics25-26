import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_share = get_package_share_directory('lunabot_description')
    sim_share = get_package_share_directory('lunabot_sim')

    default_model_path = os.path.join(
        description_share,
        'urdf',
        'lunabot.urdf.xacro',
    )

    default_world = PathJoinSubstitution(
        [
            FindPackageShare('lunabot_sim'),
            'worlds',
            'low_resolution',
            'artemis',
            'artemis_arena.world',
        ]
    )

    gz_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_models_path = os.path.join(sim_share, 'models')
    if gz_resource_path:
        gz_resource_path = f'{gz_resource_path}:{gz_models_path}'
    else:
        gz_resource_path = gz_models_path

    return LaunchDescription(
        [
            SetEnvironmentVariable(
                name='GZ_SIM_RESOURCE_PATH',
                value=gz_resource_path,
            ),
            DeclareLaunchArgument(
                name='model',
                default_value=default_model_path,
                description='Absolute path to robot xacro file',
            ),
            DeclareLaunchArgument(
                name='world',
                default_value=default_world,
                description='Absolute path to Gazebo world file',
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']
                    )
                ),
                launch_arguments={
                    'gz_args': ['-r', LaunchConfiguration('world')]
                }.items(),
            ),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                parameters=[
                    {
                        'robot_description': Command(
                            ['xacro', LaunchConfiguration('model')]
                        ),
                        'use_sim_time': True,
                    }
                ],
            ),
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name',
                    'lunabot',
                    '-topic',
                    'robot_description',
                ],
                output='screen',
            ),
        ]
    )
