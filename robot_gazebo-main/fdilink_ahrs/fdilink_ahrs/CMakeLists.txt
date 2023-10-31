cmake_minimum_required(VERSION 2.8)

project(fdilink_ahrs)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")  

find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
find_package(catkin REQUIRED COMPONENTS
   message_generation
  # genmsg
  # geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  # std_srvs
  nav_msgs
  serial
)
include_directories(
  include 
  src
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  
)

catkin_package(
  CATKIN_DEPENDS message_runtime
#  INCLUDE_DIRS include
#  LIBRARIES xf_mic_api
#  CATKIN_DEPENDS roscpp rospy senor_msgs std_msgs
#  DEPENDS system_lib
)

# set(SERIAL_LIB /opt/ros/kinetic/lib/libserial.so)
# set(SERIAL_LIB_PATH /opt/ros/kinetic/lib)
# link_directories(${SERIAL_LIB_PATH})

## ahrs_driver
add_executable(ahrs_driver src/ahrs_driver.cpp)
target_link_libraries(ahrs_driver crc_table ${catkin_LIBRARIES})# ${SERIAL_LIB}

add_executable(imu_tf src/imu_tf.cpp)
target_link_libraries( imu_tf ${catkin_LIBRARIES})# ${SERIAL_LIB}

## crc_table
add_library(crc_table src/crc_table.cpp)
