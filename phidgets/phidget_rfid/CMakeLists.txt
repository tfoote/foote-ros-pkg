cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(phidget_rfid)
rospack_add_executable(demo_rfid rfid_demo.cc)
