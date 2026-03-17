# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/mars/Documents/Lunabotics25-26/lunabot_ws/build/micro_ros_agent/agent/src/xrceagent-build/spdlog/src/spdlog"
  "/home/mars/Documents/Lunabotics25-26/lunabot_ws/build/micro_ros_agent/agent/src/xrceagent-build/spdlog/src/spdlog-build"
  "/home/mars/Documents/Lunabotics25-26/lunabot_ws/build/micro_ros_agent/agent/src/xrceagent-build/temp_install/spdlog-1.9.2"
  "/home/mars/Documents/Lunabotics25-26/lunabot_ws/build/micro_ros_agent/agent/src/xrceagent-build/spdlog/tmp"
  "/home/mars/Documents/Lunabotics25-26/lunabot_ws/build/micro_ros_agent/agent/src/xrceagent-build/spdlog/src/spdlog-stamp"
  "/home/mars/Documents/Lunabotics25-26/lunabot_ws/build/micro_ros_agent/agent/src/xrceagent-build/spdlog/src"
  "/home/mars/Documents/Lunabotics25-26/lunabot_ws/build/micro_ros_agent/agent/src/xrceagent-build/spdlog/src/spdlog-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/mars/Documents/Lunabotics25-26/lunabot_ws/build/micro_ros_agent/agent/src/xrceagent-build/spdlog/src/spdlog-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/mars/Documents/Lunabotics25-26/lunabot_ws/build/micro_ros_agent/agent/src/xrceagent-build/spdlog/src/spdlog-stamp${cfgdir}") # cfgdir has leading slash
endif()
