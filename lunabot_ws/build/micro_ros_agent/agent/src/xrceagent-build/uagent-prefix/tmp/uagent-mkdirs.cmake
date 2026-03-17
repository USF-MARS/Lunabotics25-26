# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/mars/Documents/Lunabotics25-26/lunabot_ws/build/micro_ros_agent/agent/src/xrceagent"
  "/home/mars/Documents/Lunabotics25-26/lunabot_ws/build/micro_ros_agent/agent/src/xrceagent-build"
  "/home/mars/Documents/Lunabotics25-26/lunabot_ws/build/micro_ros_agent/agent/src/xrceagent-build/uagent-prefix"
  "/home/mars/Documents/Lunabotics25-26/lunabot_ws/build/micro_ros_agent/agent/src/xrceagent-build/uagent-prefix/tmp"
  "/home/mars/Documents/Lunabotics25-26/lunabot_ws/build/micro_ros_agent/agent/src/xrceagent-build/uagent-prefix/src/uagent-stamp"
  "/home/mars/Documents/Lunabotics25-26/lunabot_ws/build/micro_ros_agent/agent/src/xrceagent-build/uagent-prefix/src"
  "/home/mars/Documents/Lunabotics25-26/lunabot_ws/build/micro_ros_agent/agent/src/xrceagent-build/uagent-prefix/src/uagent-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/mars/Documents/Lunabotics25-26/lunabot_ws/build/micro_ros_agent/agent/src/xrceagent-build/uagent-prefix/src/uagent-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/mars/Documents/Lunabotics25-26/lunabot_ws/build/micro_ros_agent/agent/src/xrceagent-build/uagent-prefix/src/uagent-stamp${cfgdir}") # cfgdir has leading slash
endif()
