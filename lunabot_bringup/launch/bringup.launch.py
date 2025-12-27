from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_teleop = LaunchConfiguration("use_teleop")
    teleop_in_terminal = LaunchConfiguration("teleop_in_terminal")
    teleop_terminal = LaunchConfiguration("teleop_terminal")

    description_share = FindPackageShare("lunabot_description")
    bringup_share = FindPackageShare("lunabot_bringup")

    robot_description = Command(
        [
            "xacro ",
            PathJoinSubstitution([description_share, "urdf", "lunabot.urdf.xacro"]),
        ]
    )

    controller_config = PathJoinSubstitution(
        [bringup_share, "config", "ros2_controllers.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation time if true.",
            ),
            DeclareLaunchArgument(
                "use_teleop",
                default_value="true",
                description="Launch teleop_twist_keyboard if true.",
            ),
            DeclareLaunchArgument(
                "teleop_in_terminal",
                default_value="false",
                description="Launch teleop in a separate terminal if true.",
            ),
            DeclareLaunchArgument(
                "teleop_terminal",
                default_value="xterm -e",
                description="Terminal command prefix used when teleop_in_terminal is true.",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[
                    {"robot_description": robot_description, "use_sim_time": use_sim_time}
                ],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[
                    {"robot_description": robot_description, "use_sim_time": use_sim_time},
                    controller_config,
                ],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
                output="screen",
            ),
            Node(
                package="teleop_twist_keyboard",
                executable="teleop_twist_keyboard",
                name="teleop_twist_keyboard",
                emulate_tty=True,
                condition=IfCondition(
                    PythonExpression([use_teleop, " and not ", teleop_in_terminal])
                ),
                output="screen",
            ),
            Node(
                package="teleop_twist_keyboard",
                executable="teleop_twist_keyboard",
                name="teleop_twist_keyboard_terminal",
                emulate_tty=True,
                prefix=teleop_terminal,
                condition=IfCondition(
                    PythonExpression([use_teleop, " and ", teleop_in_terminal])
                ),
                output="screen",
            ),
        ]
    )
