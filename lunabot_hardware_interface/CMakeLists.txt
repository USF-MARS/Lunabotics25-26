cmake_minimum_required(VERSION 3.8)
project(lunabot_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME}
  src/teensy_transport.cpp
  src/drivetrain_interface.cpp
  src/actuator_interface.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs
  nav_msgs
  diagnostic_msgs
  tf2
  tf2_ros
)

add_executable(lunabot_hardware_interface_node
  src/hardware_interface_node.cpp
)

target_compile_features(lunabot_hardware_interface_node PUBLIC cxx_std_17)

target_link_libraries(lunabot_hardware_interface_node ${PROJECT_NAME})

ament_target_dependencies(lunabot_hardware_interface_node
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs
  nav_msgs
  diagnostic_msgs
  tf2
  tf2_ros
)

install(TARGETS ${PROJECT_NAME} lunabot_hardware_interface_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
