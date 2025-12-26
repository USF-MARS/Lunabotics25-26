cmake_minimum_required(VERSION 3.8)
project(lunabot_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# This command tries to install all standard simulation folders.
# The 'OPTIONAL' flag prevents errors if the other team didn't make these specific folders.
install(
  DIRECTORY launch worlds models config maps
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
