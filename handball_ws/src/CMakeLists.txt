cmake_minimum_required(VERSION 3.0.2)

if(DEFINED ENV{ROS_DISTRO})
  set(_catkin_toplevel "/opt/ros/$ENV{ROS_DISTRO}/share/catkin/cmake/toplevel.cmake")
else()
  set(_catkin_toplevel "")
endif()

if(NOT EXISTS "${_catkin_toplevel}")
  message(FATAL_ERROR "Set ROS_DISTRO so this workspace can include catkin's toplevel.cmake.")
endif()

include(${_catkin_toplevel})
