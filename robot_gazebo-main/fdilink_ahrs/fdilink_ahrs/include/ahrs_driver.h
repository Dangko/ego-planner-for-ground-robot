#ifndef BASE_DRIVER_H_
#define BASE_DRIVER_H_

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <serial/serial.h> //ROS的串口包 http://wjwwood.io/serial/doc/1.1.0/index.html
#include <math.h>
#include <fstream>
#include <fdilink_data_struct.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Pose2D.h>
#include <boost/thread.hpp>
#include <string>
#include <ros/package.h>
#include <crc_table.h>

using namespace std;
namespace FDILink
{
#define FRAME_HEAD 0xfc
#define FRAME_END 0xfd
#define TYPE_IMU 0x40
#define TYPE_AHRS 0x41
#define TYPE_INSGPS 0x42
#define TYPE_GEODETIC_POS 0x5c
#define TYPE_GROUND 0xf0

#define IMU_LEN  0x38   //56
#define AHRS_LEN 0x30   //48
#define INSGPS_LEN 0x48 //80
#define GEODETIC_POS_LEN 0x20 //32
#define PI 3.141592653589793
#define DEG_TO_RAD 0.017453292519943295


class ahrsBringup
{
public:
  ahrsBringup();
  ~ahrsBringup();
  void processLoop();
  bool checkCS8(int len);
  bool checkCS16(int len);
  void checkSN(int type);
  void magCalculateYaw(double roll, double pitch, double &magyaw, double magx, double magy, double magz);
  ros::NodeHandle nh_;

private:
  bool if_debug_;
  //sum info
  int sn_lost_ = 0;
  int crc_error_ = 0;
  uint8_t read_sn_ = 0;
  bool frist_sn_;
  int device_type_ = 1;

  //serial
  serial::Serial serial_; //声明串口对象
  std::string serial_port_;
  int serial_baud_;
  int serial_timeout_;
  //data
  FDILink::imu_frame_read  imu_frame_;
  FDILink::ahrs_frame_read ahrs_frame_;
  FDILink::insgps_frame_read insgps_frame_;
  //FDILink::lanlon_frame_read latlon_frame_;
  FDILink::Geodetic_Position_frame_read Geodetic_Position_frame_;
  //frame name
  string imu_frame_id_;
  string insgps_frame_id_;
  string latlon_frame_id_;
  //topic
  string imu_topic_, mag_pose_2d_topic_;
  string latlon_topic_;
  string Euler_angles_topic_,Magnetic_topic_;

  //Publisher
  ros::Publisher imu_pub_;
  ros::Publisher gps_pub_;
  ros::Publisher mag_pose_pub_;
  ros::Publisher Euler_angles_pub_;
  ros::Publisher Magnetic_pub_;
  ros::Publisher gnss_pub_;

}; //ahrsBringup
} // namespace FDILink

#endif
