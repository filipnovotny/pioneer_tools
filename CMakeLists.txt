cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

rosbuild_add_library(joystick_teleop_common src/names.cpp)

#common commands for building c++ executables and libraries
#rosbuild_add_library(joystick_teleop src/example.cpp)
#target_link_libraries(joystick_teleop another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(joystick_teleop thread)

rosbuild_add_executable(joystick_teleop src/teleop_main.cpp src/teleop.cpp)
target_link_libraries(joystick_teleop joystick_teleop_common)

rosbuild_add_executable(odometry_frame_publisher src/odometry_frame_publisher_main.cpp src/odometry_frame_publisher.cpp)
target_link_libraries(odometry_frame_publisher joystick_teleop_common)
