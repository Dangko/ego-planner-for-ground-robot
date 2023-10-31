#ifndef FDILINK_DATA_STRUCT_H_
#define FDILINK_DATA_STRUCT_H_

#include <iostream>
namespace FDILink{
#pragma pack(1)
struct fdilink_header
{
	uint8_t  header_start;
	uint8_t  data_type;
	uint8_t  data_size;
	uint8_t  serial_num;
	uint8_t  header_crc8;
    uint8_t  header_crc16_h;
	uint8_t  header_crc16_l;
};
#pragma pack()

#pragma pack(1)
struct IMUData_Packet_t
{
	float gyroscope_x;          //unit: rad/s
	float gyroscope_y;          //unit: rad/s
	float gyroscope_z;          //unit: rad/s
	float accelerometer_x;      //m/s^2
	float accelerometer_y;      //m/s^2
	float accelerometer_z;      //m/s^2
	float magnetometer_x;       //mG
	float magnetometer_y;       //mG
	float magnetometer_z;       //mG
	float imu_temperature;      //C
	float Pressure;             //Pa
	float pressure_temperature; //C
	int64_t Timestamp;          //us
};
#pragma pack()

struct AHRSData_Packet_t
{
	float RollSpeed;   //unit: rad/s
	float PitchSpeed;  //unit: rad/s
	float HeadingSpeed;//unit: rad/s
	float Roll;        //unit: rad
	float Pitch;       //unit: rad
	float Heading;     //unit: rad
	float Qw;//w          //Quaternion
	float Qx;//x
	float Qy;//y
	float Qz;//z
	int64_t Timestamp; //unit: us
};
#pragma pack(1)
struct INSGPSData_Packet_t
{
	float BodyVelocity_X;       
	float BodyVelocity_Y;       
	float BodyVelocity_Z;       
	float BodyAcceleration_X;   
	float BodyAcceleration_Y;   
	float BodyAcceleration_Z;   
	float Location_North;      
	float Location_East;       
	float Location_Down;
	float Velocity_North;
	float Velocity_East;
	float Velocity_Down;
	float Acceleration_North;
	float Acceleration_East;
	float Acceleration_Down;
	float Pressure_Altitude;
	int64_t Timestamp;
};
#pragma pack()

#pragma pack(1)
struct Geodetic_Position_Packet_t
{
	double Latitude;       
	double Longitude;       
	double Height;
	float hAcc;
	float vAcc;      
};
#pragma pack()

//for IMU=========================
#pragma pack(1)
struct read_imu_struct{
  fdilink_header     header;    //7                
  union data
  {
	IMUData_Packet_t   data_pack; //56
	uint8_t            data_buff[56]; //56
  }data;
  uint8_t            frame_end; //1                  
};            

struct read_imu_tmp{
  uint8_t frame_header[7];
  uint8_t read_msg[57];
};                           

union imu_frame_read{
  struct read_imu_struct frame;
  read_imu_tmp read_buf;
  uint8_t read_tmp[64];
};
#pragma pack()
//for IMU------------------------

//for AHRS=========================
#pragma pack(1)
struct read_ahrs_struct{
  fdilink_header     header;    //7                
  union data
  {
	AHRSData_Packet_t  data_pack; //48
	uint8_t            data_buff[48]; //48
  }data;
  uint8_t            frame_end; //1                  
};       

 
struct read_ahrs_tmp{
  uint8_t frame_header[7];
  uint8_t read_msg[49];
};                           

union ahrs_frame_read{
  struct read_ahrs_struct frame;
  read_ahrs_tmp read_buf;
  uint8_t read_tmp[56];
};
#pragma pack()
//for AHRS------------------------

//for INSGPS=========================
#pragma pack(1)
struct read_insgps_struct{
  fdilink_header     header;    //7                
  union data
  {
	INSGPSData_Packet_t  data_pack; //72
	uint8_t              data_buff[72]; //72
  }data;
  uint8_t            frame_end; //1                  
};   

                           
struct read_insgps_tmp{
  uint8_t frame_header[7];
  uint8_t read_msg[73];
};                          

union insgps_frame_read{
  struct read_insgps_struct frame;
  read_insgps_tmp read_buf;
  uint8_t read_tmp[80];
};
#pragma pack()
//for INSGPS------------------------

//for Geodetic_Position=========================
#pragma pack(1)
struct read_Geodetic_Position_struct{
  fdilink_header     header;    //7                
  union data
  {
	Geodetic_Position_Packet_t  data_pack; //40
	uint8_t                     data_buff[32]; //40
  }data;
  uint8_t            frame_end; //1                  
};   
                           
struct read_Geodetic_Position_tmp{
  uint8_t frame_header[7];
  uint8_t read_msg[33];
};                          

union Geodetic_Position_frame_read{
  struct read_Geodetic_Position_struct frame;
  read_Geodetic_Position_tmp read_buf;
  uint8_t read_tmp[40];
};
 
#pragma pack()

}//namespace FDILink
#endif//FDILINK_DATA_STRUCT_H_
