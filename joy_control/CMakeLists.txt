cmake_minimum_required(VERSION 3.0.2)
project(joy_control)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES joy_control
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
CATKIN_DEPENDS roscpp sensor_msgs std_msgs
) 


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(joy_control src/joy_control.cpp)
target_link_libraries(joy_control ${catkin_LIBRARIES})
