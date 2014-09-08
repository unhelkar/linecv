cmake_minimum_required(VERSION 2.8.3)
project(linecv)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  message_generation
  belt_sensing
)

## Generate messages
add_message_files(FILES
  nline_xy.msg
 )

generate_messages(
  DEPENDENCIES std_msgs
 )

## Declare a catkin package
catkin_package() 

## Adding directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Adding executables
#add_executable(simple_canny src/simple_canny.cpp)
#target_link_libraries(simple_canny ${catkin_LIBRARIES})
