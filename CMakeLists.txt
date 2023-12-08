cmake_minimum_required(VERSION 3.5)
project(armer_panda)

# Setup for ROS 2 based on ROS_VERSION environment variable
# if($ENV{ROS_VERSION} STREQUAL "2")
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
