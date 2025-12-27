ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro $(ros2 pkg prefix --share lunabot_description)/urdf/lunabot.urdf.xacro)"

Note: Make sure your workspace is built and sourced (e.g., `colcon build` then `source install/setup.bash`) so package discovery (`ros2 pkg prefix --share` or `get_package_share_directory`) can find `lunabot_description`.
