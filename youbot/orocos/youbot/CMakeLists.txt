#
# The find_package macro for Orocos-RTT works best with
# cmake >= 2.6.3
#
cmake_minimum_required(VERSION 2.6.3)

#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
# The CMake 'target' names are identical to the first argument of the
# macro, except for orocos_typekit, where the target is fully
# controlled by generated code of 'typegen'.
#
project(youbot)

#
# Do setup in case of ros package
#
set (ROS_ROOT $ENV{ROS_ROOT} )
if (ROS_ROOT)
  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
  rosbuild_init()
  rosbuild_find_ros_package(rtt )
endif()
find_package(Orocos-RTT PATHS ${rtt_PACKAGE_PATH}/install )

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

# Creates a component library libyoubot-<target>.so
# and installs in the directory lib/orocos/youbot/
#
orocos_component(youbot-api youbot-api-component.cpp ) # ...you may add multiple source files
orocos_component(cartesian cartesian-component.cpp youbot_chain.cpp) # ...you may add multiple source files
orocos_component(jointspace jointspace-component.cpp ) # ...you may add multiple source files
orocos_component(supervisor supervisor-component.cpp ) # ...you may add multiple source files
#
# You may add multiple orocos_component statements.

include_directories(/opt/ros/cturtle/stacks/geometry/kdl/include /usr/include/eigen2)
target_link_libraries(youbot-api memoryMapAndSemaphore /opt/ros/cturtle/stacks/geometry/kdl/lib/liborocos-kdl.so)
target_link_libraries(cartesian  /opt/ros/cturtle/stacks/geometry/kdl/lib/liborocos-kdl.so)
target_link_libraries(jointspace /opt/ros/cturtle/stacks/geometry/kdl/lib/liborocos-kdl.so)

#
# Building a typekit (recommended):
#
# Creates a typekit library libyoubot-types-<target>.so
# and installs in the directory lib/orocos/youbot/types/
#
orocos_typekit(youbot-types.hpp) # ...you may add multiple header files
#
# You may only have *ONE* orocos_typekit statement !


#
# Building a normal library (optional):
#
# Creates a library libsupport-<target>.so and installs it in
# lib/
#
#orocos_library(support support.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_library statements.


#
# Building a Plugin or Service (optional):
#
# Creates a plugin library libyoubot-service-<target>.so or libyoubot-plugin-<target>.so
# and installs in the directory lib/orocos/youbot/plugins/
#
orocos_plugin(youbot-service youbot-service.cpp) # ...you may add multiple source files
#orocos_plugin(youbot-plugin youbot-plugin.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_plugin statements.


#
# Additional headers (not in typekit):
#
# Installs in the include/orocos/youbot/ directory
#
orocos_install_headers( youbot-component.hpp ) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

#
# Generates and installs our orocos-youbot-<target>.pc and manifest.xml file
#
configure_file( ${CMAKE_CURRENT_SOURCE_DIR}/orocos-youbot.pc.in ${CMAKE_CURRENT_BINARY_DIR}/orocos-youbot-${OROCOS_TARGET}.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/orocos-youbot-${OROCOS_TARGET}.pc DESTINATION  lib/pkgconfig )
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/manifest.xml DESTINATION  lib/orocos/youbot )

#
# Provided 'uninstall' target
#
#orocos_uninstall_target()
