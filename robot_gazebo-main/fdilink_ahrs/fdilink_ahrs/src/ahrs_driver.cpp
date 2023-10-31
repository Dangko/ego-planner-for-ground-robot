#include <ahrs_driver.h>
#include <Eigen/Eigen>
namespace FDILink
{

ahrsBringup::ahrsBringup() :frist_sn_(false), serial_timeout_(20)
{
  ros::NodeHandle pravite_nh("~");
  //topic_name & frame_id
  pravite_nh.param("debug",     if_debug_,  false);
  pravite_nh.param("device_type", device_type_, 1); // default: single imu
  pravite_nh.param("imu_topic", imu_topic_, std::string("/imu"));
  pravite_nh.param("imu_frame", imu_frame_id_, std::string("imu"));
  pravite_nh.param("mag_pose_2d_topic", mag_pose_2d_topic_, std::string("/mag_pose_2d"));
  pravite_nh.param("Euler_angles_pub_", Euler_angles_topic_, std::string("/euler_angles"));
  pravite_nh.param("Magnetic_pub_", Magnetic_topic_, std::string("/magnetic"));
 
  //serial                                                 
  pravite_nh.param("port", serial_port_, std::string("/dev/ttyTHS1")); 
  pravite_nh.param("baud", serial_baud_, 115200);
  //publisher
  imu_pub_ = nh_.advertise<sensor_msgs::Imu>(imu_topic_.c_str(), 10);
  gps_pub_ = nh_.advertise<sensor_msgs::NavSatFix>("/gps/fix", 10);
  gnss_pub_ = nh_.advertise<sensor_msgs::NavSatFix>("/gnss_data", 10);

  mag_pose_pub_ = nh_.advertise<geometry_msgs::Pose2D>(mag_pose_2d_topic_.c_str(), 10);

  Euler_angles_pub_ = nh_.advertise<geometry_msgs::Vector3>(Euler_angles_topic_.c_str(), 10);
  Magnetic_pub_ = nh_.advertise<geometry_msgs::Vector3>(Magnetic_topic_.c_str(), 10);
  //setp up serial
  try
  {
    serial_.setPort(serial_port_);
    serial_.setBaudrate(serial_baud_);
    serial_.setFlowcontrol(serial::flowcontrol_none);
    serial_.setParity(serial::parity_none); //default is parity_none
    serial_.setStopbits(serial::stopbits_one);
    serial_.setBytesize(serial::eightbits);
    serial::Timeout time_out = serial::Timeout::simpleTimeout(serial_timeout_);
    serial_.setTimeout(time_out);
    serial_.open();
  }
  catch (serial::IOException &e)
  {
    ROS_ERROR_STREAM("Unable to open port ");
    exit(0);
  }
  if (serial_.isOpen())
  {
    ROS_INFO_STREAM("Serial Port initialized");
  }
  else
  {
    ROS_ERROR_STREAM("Unable to initial Serial port ");
    exit(0);
  }
  processLoop();
}

ahrsBringup::~ahrsBringup()
{
  if (serial_.isOpen())
    serial_.close();
}

void ahrsBringup::processLoop()
{
  ROS_INFO("ahrsBringup::processLoop: start");
  while (ros::ok())
  {
    if (!serial_.isOpen())
    {
      ROS_WARN("serial unopen");
    }
    //check head start
    uint8_t check_head[1] = {0xff};
    size_t head_s = serial_.read(check_head, 1);
    if (if_debug_){
      if (head_s != 1)
      {
        ROS_ERROR("Read serial port time out! can't read pack head.");
      }
      std::cout << std::endl;
      std::cout << "check_head: " << std::hex << (int)check_head[0] << std::dec << std::endl;
    }
    if (check_head[0] != FRAME_HEAD)
    {
      continue;
    }
    //check head type
    uint8_t head_type[1] = {0xff};
    size_t type_s = serial_.read(head_type, 1);
    if (if_debug_){
      std::cout << "head_type:  " << std::hex << (int)head_type[0] << std::dec << std::endl;
    }
    if (head_type[0] != TYPE_IMU && head_type[0] != TYPE_AHRS && head_type[0] != TYPE_INSGPS && head_type[0] != TYPE_GEODETIC_POS && head_type[0] != 0x50 && head_type[0] != TYPE_GROUND)
    {
      //ROS_WARN("head_type error: %02X",head_type[0]);
      continue;
    }
    //check head length
    uint8_t check_len[1] = {0xff};
    size_t len_s = serial_.read(check_len, 1);
    if (if_debug_){
      std::cout << "check_len: "<< std::dec << (int)check_len[0]  << std::endl;
    }
    if (head_type[0] == TYPE_IMU && check_len[0] != IMU_LEN)
    {
      ROS_WARN("head_len error (imu)");
      continue;
    }else if (head_type[0] == TYPE_AHRS && check_len[0] != AHRS_LEN)
    {
      ROS_WARN("head_len error (ahrs)");
      continue;
    }else if (head_type[0] == TYPE_INSGPS && check_len[0] != INSGPS_LEN)
    {
      ROS_WARN("head_len error (insgps)");
      continue;
    }else if (head_type[0] == TYPE_GEODETIC_POS && check_len[0] != GEODETIC_POS_LEN)
    {
      ROS_WARN("head_len error (GEODETIC_POS)");
      continue;
    }
    else if (head_type[0] == TYPE_GROUND || head_type[0] == 0x50) // 未知数据，防止记录失败
    {
      uint8_t ground_sn[1];
      size_t ground_sn_s = serial_.read(ground_sn, 1);
      if (++read_sn_ != ground_sn[0])
      {
        if ( ground_sn[0] < read_sn_)
        {
          if(if_debug_){
            ROS_WARN("detected sn lost.");
          }
          sn_lost_ += 256 - (int)(read_sn_ - ground_sn[0]);
          read_sn_ = ground_sn[0];
          // continue;
        }
        else
        {
          if(if_debug_){
            ROS_WARN("detected sn lost.");
          }
          sn_lost_ += (int)(ground_sn[0] - read_sn_);
          read_sn_ = ground_sn[0];
          // continue;
        }
      }
      uint8_t ground_ignore[500];
      size_t ground_ignore_s = serial_.read(ground_ignore, (check_len[0]+4));
      continue;
    }
    //read head sn 
    uint8_t check_sn[1] = {0xff};
    size_t sn_s = serial_.read(check_sn, 1);
    uint8_t head_crc8[1] = {0xff};
    size_t crc8_s = serial_.read(head_crc8, 1);
    uint8_t head_crc16_H[1] = {0xff};
    uint8_t head_crc16_L[1] = {0xff};
    size_t crc16_H_s = serial_.read(head_crc16_H, 1);
    size_t crc16_L_s = serial_.read(head_crc16_L, 1);
    if (if_debug_){
      std::cout << "check_sn: "     << std::hex << (int)check_sn[0]     << std::dec << std::endl;
      std::cout << "head_crc8: "    << std::hex << (int)head_crc8[0]    << std::dec << std::endl;
      std::cout << "head_crc16_H: " << std::hex << (int)head_crc16_H[0] << std::dec << std::endl;
      std::cout << "head_crc16_L: " << std::hex << (int)head_crc16_L[0] << std::dec << std::endl;
    }
    // put header & check crc8 & count sn lost
    if (head_type[0] == TYPE_IMU)
    {
      imu_frame_.frame.header.header_start   = check_head[0];
      imu_frame_.frame.header.data_type      = head_type[0];
      imu_frame_.frame.header.data_size      = check_len[0];
      imu_frame_.frame.header.serial_num     = check_sn[0];
      imu_frame_.frame.header.header_crc8    = head_crc8[0];
      imu_frame_.frame.header.header_crc16_h = head_crc16_H[0];
      imu_frame_.frame.header.header_crc16_l = head_crc16_L[0];
      uint8_t CRC8 = CRC8_Table(imu_frame_.read_buf.frame_header, 4);
      if (CRC8 != imu_frame_.frame.header.header_crc8)
      {
        ROS_WARN("header_crc8 error");
        continue;
      }
      if(!frist_sn_){
        read_sn_  = imu_frame_.frame.header.serial_num - 1;
        frist_sn_ = true;
      }
      //check sn 
      ahrsBringup::checkSN(TYPE_IMU);
    }
    else if (head_type[0] == TYPE_AHRS)
    {
      ahrs_frame_.frame.header.header_start   = check_head[0];
      ahrs_frame_.frame.header.data_type      = head_type[0];
      ahrs_frame_.frame.header.data_size      = check_len[0];
      ahrs_frame_.frame.header.serial_num     = check_sn[0];
      ahrs_frame_.frame.header.header_crc8    = head_crc8[0];
      ahrs_frame_.frame.header.header_crc16_h = head_crc16_H[0];
      ahrs_frame_.frame.header.header_crc16_l = head_crc16_L[0];
      uint8_t CRC8 = CRC8_Table(ahrs_frame_.read_buf.frame_header, 4);
      if (CRC8 != ahrs_frame_.frame.header.header_crc8)
      {
        ROS_WARN("header_crc8 error");
        continue;
      }
      if(!frist_sn_){
        read_sn_  = ahrs_frame_.frame.header.serial_num - 1;
        frist_sn_ = true;
      }
      //check sn 
      ahrsBringup::checkSN(TYPE_AHRS);
    }
    else if (head_type[0] == TYPE_INSGPS)
    {
      insgps_frame_.frame.header.header_start   = check_head[0];
      insgps_frame_.frame.header.data_type      = head_type[0];
      insgps_frame_.frame.header.data_size      = check_len[0];
      insgps_frame_.frame.header.serial_num     = check_sn[0];
      insgps_frame_.frame.header.header_crc8    = head_crc8[0];
      insgps_frame_.frame.header.header_crc16_h = head_crc16_H[0];
      insgps_frame_.frame.header.header_crc16_l = head_crc16_L[0];
      uint8_t CRC8 = CRC8_Table(insgps_frame_.read_buf.frame_header, 4);
      if (CRC8 != insgps_frame_.frame.header.header_crc8)
      {
        ROS_WARN("header_crc8 error");
        continue;
      }
      else if(if_debug_)
      {
        std::cout << "header_crc8 matched." << std::endl;
      }
      
      ahrsBringup::checkSN(TYPE_INSGPS);
    }
    else if (head_type[0] == TYPE_GEODETIC_POS)
    {
      Geodetic_Position_frame_.frame.header.header_start   = check_head[0];
      Geodetic_Position_frame_.frame.header.data_type      = head_type[0];
      Geodetic_Position_frame_.frame.header.data_size      = check_len[0];
      Geodetic_Position_frame_.frame.header.serial_num     = check_sn[0];
      Geodetic_Position_frame_.frame.header.header_crc8    = head_crc8[0];
      Geodetic_Position_frame_.frame.header.header_crc16_h = head_crc16_H[0];
      Geodetic_Position_frame_.frame.header.header_crc16_l = head_crc16_L[0];
      uint8_t CRC8 = CRC8_Table(Geodetic_Position_frame_.read_buf.frame_header, 4);
      if (CRC8 != Geodetic_Position_frame_.frame.header.header_crc8)
      {
        ROS_WARN("header_crc8 error");
        continue;
      }
      if(!frist_sn_){
        read_sn_  = Geodetic_Position_frame_.frame.header.serial_num - 1;
        frist_sn_ = true;
      }
      
      ahrsBringup::checkSN(TYPE_GEODETIC_POS);
    }
    if (head_type[0] == TYPE_IMU)
    {
      uint16_t head_crc16_l = imu_frame_.frame.header.header_crc16_l;
      uint16_t head_crc16_h = imu_frame_.frame.header.header_crc16_h;
      uint16_t head_crc16 = head_crc16_l + (head_crc16_h << 8);
      size_t data_s = serial_.read(imu_frame_.read_buf.read_msg, (IMU_LEN + 1)); //48+1
      // if (if_debug_){
      //   for (size_t i = 0; i < (IMU_LEN + 1); i++)
      //   {
      //     std::cout << std::hex << (int)imu_frame_.read_buf.read_msg[i] << " ";
      //   }
      //   std::cout << std::dec << std::endl;
      // }
      uint16_t CRC16 = CRC16_Table(imu_frame_.frame.data.data_buff, IMU_LEN);
      if (if_debug_)
      {          
        std::cout << "CRC16:        " << std::hex << (int)CRC16 << std::dec << std::endl;
        std::cout << "head_crc16:   " << std::hex << (int)head_crc16 << std::dec << std::endl;
        std::cout << "head_crc16_h: " << std::hex << (int)head_crc16_h << std::dec << std::endl;
        std::cout << "head_crc16_l: " << std::hex << (int)head_crc16_l << std::dec << std::endl;
        bool if_right = ((int)head_crc16 == (int)CRC16);
        std::cout << "if_right: " << if_right << std::endl;
      }
      
      if (head_crc16 != CRC16)
      {
        ROS_WARN("check crc16 faild(imu).");
        continue;
      }
      else if(imu_frame_.frame.frame_end != FRAME_END)
      {
        ROS_WARN("check frame end.");
        continue;
      }
      
    }
    else if (head_type[0] == TYPE_AHRS)
    {
      uint16_t head_crc16_l = ahrs_frame_.frame.header.header_crc16_l;
      uint16_t head_crc16_h = ahrs_frame_.frame.header.header_crc16_h;
      uint16_t head_crc16 = head_crc16_l + (head_crc16_h << 8);
      size_t data_s = serial_.read(ahrs_frame_.read_buf.read_msg, (AHRS_LEN + 1)); //48+1
      // if (if_debug_){
      //   for (size_t i = 0; i < (AHRS_LEN + 1); i++)
      //   {
      //     std::cout << std::hex << (int)ahrs_frame_.read_buf.read_msg[i] << " ";
      //   }
      //   std::cout << std::dec << std::endl;
      // }
      uint16_t CRC16 = CRC16_Table(ahrs_frame_.frame.data.data_buff, AHRS_LEN);
      if (if_debug_){          
        std::cout << "CRC16:        " << std::hex << (int)CRC16 << std::dec << std::endl;
        std::cout << "head_crc16:   " << std::hex << (int)head_crc16 << std::dec << std::endl;
        std::cout << "head_crc16_h: " << std::hex << (int)head_crc16_h << std::dec << std::endl;
        std::cout << "head_crc16_l: " << std::hex << (int)head_crc16_l << std::dec << std::endl;
        bool if_right = ((int)head_crc16 == (int)CRC16);
        std::cout << "if_right: " << if_right << std::endl;
      }
      
      if (head_crc16 != CRC16)
      {
        ROS_WARN("check crc16 faild(ahrs).");
        continue;
      }
      else if(ahrs_frame_.frame.frame_end != FRAME_END)
      {
        ROS_WARN("check frame end.");
        continue;
      }
    }
    else if (head_type[0] == TYPE_INSGPS)
    {
      uint16_t head_crc16_l = insgps_frame_.frame.header.header_crc16_l;
      uint16_t head_crc16_h = insgps_frame_.frame.header.header_crc16_h;
      uint16_t head_crc16 = head_crc16_l + (head_crc16_h << 8);
      size_t data_s = serial_.read(insgps_frame_.read_buf.read_msg, (INSGPS_LEN + 1)); //48+1
      // if (if_debug_){
      //   for (size_t i = 0; i < (AHRS_LEN + 1); i++)
      //   {
      //     std::cout << std::hex << (int)ahrs_frame_.read_buf.read_msg[i] << " ";
      //   }
      //   std::cout << std::dec << std::endl;
      // }
      uint16_t CRC16 = CRC16_Table(insgps_frame_.frame.data.data_buff, INSGPS_LEN);
      if (if_debug_){          
        std::cout << "CRC16:        " << std::hex << (int)CRC16 << std::dec << std::endl;
        std::cout << "head_crc16:   " << std::hex << (int)head_crc16 << std::dec << std::endl;
        std::cout << "head_crc16_h: " << std::hex << (int)head_crc16_h << std::dec << std::endl;
        std::cout << "head_crc16_l: " << std::hex << (int)head_crc16_l << std::dec << std::endl;
        bool if_right = ((int)head_crc16 == (int)CRC16);
        std::cout << "if_right: " << if_right << std::endl;
      }
      
      if (head_crc16 != CRC16)
      {
        ROS_WARN("check crc16 faild(ahrs).");
        continue;
      }
      else if(insgps_frame_.frame.frame_end != FRAME_END)
      {
        ROS_WARN("check frame end.");
        continue;
      } 
    }
    else if (head_type[0] == TYPE_GEODETIC_POS)
    {
      uint16_t head_crc16_l = Geodetic_Position_frame_.frame.header.header_crc16_l;
      uint16_t head_crc16_h = Geodetic_Position_frame_.frame.header.header_crc16_h;
      uint16_t head_crc16 = head_crc16_l + (head_crc16_h << 8);
      size_t data_s = serial_.read(Geodetic_Position_frame_.read_buf.read_msg, (GEODETIC_POS_LEN + 1)); //24+1
      // if (if_debug_){
      //   for (size_t i = 0; i < (AHRS_LEN + 1); i++)
      //   {
      //     std::cout << std::hex << (int)ahrs_frame_.read_buf.read_msg[i] << " ";
      //   }
      //   std::cout << std::dec << std::endl;
      // }
      uint16_t CRC16 = CRC16_Table(Geodetic_Position_frame_.frame.data.data_buff, GEODETIC_POS_LEN);
      if (if_debug_){          
        std::cout << "CRC16:        " << std::hex << (int)CRC16 << std::dec << std::endl;
        std::cout << "head_crc16:   " << std::hex << (int)head_crc16 << std::dec << std::endl;
        std::cout << "head_crc16_h: " << std::hex << (int)head_crc16_h << std::dec << std::endl;
        std::cout << "head_crc16_l: " << std::hex << (int)head_crc16_l << std::dec << std::endl;
        bool if_right = ((int)head_crc16 == (int)CRC16);
        std::cout << "if_right: " << if_right << std::endl;
      }
      
      if (head_crc16 != CRC16)
      {
        ROS_WARN("check crc16 faild(gps).");
        continue;
      }
      else if(Geodetic_Position_frame_.frame.frame_end != FRAME_END)
      {
        ROS_WARN("check frame end.");
        continue;
      }
    }
   // publish magyaw topic

    if (head_type[0] == TYPE_IMU)
    {
      // publish imu topic
      sensor_msgs::Imu imu_data;
      imu_data.header.stamp = ros::Time::now();
      imu_data.header.frame_id = imu_frame_id_.c_str();
      Eigen::Quaterniond q_ahrs(ahrs_frame_.frame.data.data_pack.Qw,
                                ahrs_frame_.frame.data.data_pack.Qx,
                                ahrs_frame_.frame.data.data_pack.Qy,
                                ahrs_frame_.frame.data.data_pack.Qz);
      Eigen::Quaterniond q_r =                          
          Eigen::AngleAxisd( 3.14159, Eigen::Vector3d::UnitZ()) * 
          Eigen::AngleAxisd( 3.14159, Eigen::Vector3d::UnitY()) * 
          Eigen::AngleAxisd( 0.00000, Eigen::Vector3d::UnitX());
      Eigen::Quaterniond q_rr =                          
          Eigen::AngleAxisd( 0.00000, Eigen::Vector3d::UnitZ()) * 
          Eigen::AngleAxisd( 0.00000, Eigen::Vector3d::UnitY()) * 
          Eigen::AngleAxisd( 3.14159, Eigen::Vector3d::UnitX());
      Eigen::Quaterniond q_xiao_rr =
          Eigen::AngleAxisd( 3.14159/2, Eigen::Vector3d::UnitZ()) * 
          Eigen::AngleAxisd( 0.00000, Eigen::Vector3d::UnitY()) * 
          Eigen::AngleAxisd( 3.14159, Eigen::Vector3d::UnitX());
      if (device_type_ == 0)         //未经变换的原始数据
      {
        imu_data.orientation.w = ahrs_frame_.frame.data.data_pack.Qw;
        imu_data.orientation.x = ahrs_frame_.frame.data.data_pack.Qx;
        imu_data.orientation.y = ahrs_frame_.frame.data.data_pack.Qy;
        imu_data.orientation.z = ahrs_frame_.frame.data.data_pack.Qz;
        imu_data.angular_velocity.x = ahrs_frame_.frame.data.data_pack.RollSpeed;
        imu_data.angular_velocity.y = ahrs_frame_.frame.data.data_pack.PitchSpeed;
        imu_data.angular_velocity.z = ahrs_frame_.frame.data.data_pack.HeadingSpeed;
        imu_data.linear_acceleration.x = imu_frame_.frame.data.data_pack.accelerometer_x;
        imu_data.linear_acceleration.y = imu_frame_.frame.data.data_pack.accelerometer_y;
        imu_data.linear_acceleration.z = imu_frame_.frame.data.data_pack.accelerometer_z;
      }
      else if (device_type_ == 1)    //imu单品ROS标准下的坐标变换
      {
        
        Eigen::Quaterniond q_out =  q_r * q_ahrs * q_rr;
        imu_data.orientation.w = q_out.w();
        imu_data.orientation.x = q_out.x();
        imu_data.orientation.y = q_out.y();
        imu_data.orientation.z = q_out.z();
        imu_data.angular_velocity.x = ahrs_frame_.frame.data.data_pack.RollSpeed;
        imu_data.angular_velocity.y = -ahrs_frame_.frame.data.data_pack.PitchSpeed;
        imu_data.angular_velocity.z = -ahrs_frame_.frame.data.data_pack.HeadingSpeed;
        imu_data.linear_acceleration.x = imu_frame_.frame.data.data_pack.accelerometer_x;
        imu_data.linear_acceleration.y = -imu_frame_.frame.data.data_pack.accelerometer_y;
        imu_data.linear_acceleration.z = -imu_frame_.frame.data.data_pack.accelerometer_z;
      }
      imu_pub_.publish(imu_data);
}
    else if (head_type[0] == TYPE_AHRS)
    {
      geometry_msgs::Pose2D pose_2d;
      pose_2d.theta = ahrs_frame_.frame.data.data_pack.Heading;
      mag_pose_pub_.publish(pose_2d);
      //std::cout << "YAW: " << pose_2d.theta << std::endl;
      geometry_msgs::Vector3 Euler_angles_2d,Magnetic;  
      Euler_angles_2d.x = ahrs_frame_.frame.data.data_pack.Roll;
      Euler_angles_2d.y = ahrs_frame_.frame.data.data_pack.Pitch;
      Euler_angles_2d.z = ahrs_frame_.frame.data.data_pack.Heading;
      Magnetic.x = imu_frame_.frame.data.data_pack.magnetometer_x;
      Magnetic.y = imu_frame_.frame.data.data_pack.magnetometer_y;
      Magnetic.z = imu_frame_.frame.data.data_pack.magnetometer_z;

      Euler_angles_pub_.publish(Euler_angles_2d);
      Magnetic_pub_.publish(Magnetic);

    }
    //将解析到的gps数据赋值并publish
    else if (head_type[0] == TYPE_GEODETIC_POS)
    {
      sensor_msgs::NavSatFix gps_data;
      gps_data.header.stamp = ros::Time::now();
      gps_data.header.frame_id = "navsat_link";
      gps_data.latitude = Geodetic_Position_frame_.frame.data.data_pack.Latitude / DEG_TO_RAD;
      gps_data.longitude = Geodetic_Position_frame_.frame.data.data_pack.Longitude / DEG_TO_RAD;
      gps_data.altitude = Geodetic_Position_frame_.frame.data.data_pack.Height;

      //std::cout << "lat: " << Geodetic_Position_frame_.frame.data.data_pack.Latitude << std::endl;
      //std::cout << "lon: " << Geodetic_Position_frame_.frame.data.data_pack.Longitude << std::endl;
      //std::cout << "h: " << Geodetic_Position_frame_.frame.data.data_pack.Height << std::endl;

      gps_pub_.publish(gps_data);
    } 
    else if (head_type[0] == TYPE_INSGPS)
    {
		
    //  std::cout << "N: " << insgps_frame_.frame.data.data_pack.Location_North << std::endl;
    //  std::cout << "E: " << insgps_frame_.frame.data.data_pack.Location_East << std::endl;
    //  std::cout << "D: " << insgps_frame_.frame.data.data_pack.Location_Down << std::endl;

    }   
  }
}

void ahrsBringup::magCalculateYaw(double roll, double pitch, double &magyaw, double magx, double magy, double magz)
{
  double temp1 = magy * cos(roll) + magz * sin(roll);
  double temp2 = magx * cos(pitch) + magy * sin(pitch) * sin(roll) - magz * sin(pitch) * cos(roll);
  magyaw = atan2(-temp1, temp2);
  if(magyaw < 0)
  {
    magyaw = magyaw + 2 * PI;
  }
  // return magyaw;
}

void ahrsBringup::checkSN(int type)
{
  switch (type)
  {
  case TYPE_IMU:
    if (++read_sn_ != imu_frame_.frame.header.serial_num)
    {
      if ( imu_frame_.frame.header.serial_num < read_sn_)
      {
        sn_lost_ += 256 - (int)(read_sn_ - imu_frame_.frame.header.serial_num);
        if(if_debug_){
          ROS_WARN("detected sn lost.");
        }
      }
      else
      {
        sn_lost_ += (int)(imu_frame_.frame.header.serial_num - read_sn_);
        if(if_debug_){
          ROS_WARN("detected sn lost.");
        }
      }
    }
    read_sn_ = imu_frame_.frame.header.serial_num;
    break;

  case TYPE_AHRS:
    if (++read_sn_ != ahrs_frame_.frame.header.serial_num)
    {
      if ( ahrs_frame_.frame.header.serial_num < read_sn_)
      {
        sn_lost_ += 256 - (int)(read_sn_ - ahrs_frame_.frame.header.serial_num);
        if(if_debug_){
          ROS_WARN("detected sn lost.");
        }
      }
      else
      {
        sn_lost_ += (int)(ahrs_frame_.frame.header.serial_num - read_sn_);
        if(if_debug_){
          ROS_WARN("detected sn lost.");
        }
      }
    }
    read_sn_ = ahrs_frame_.frame.header.serial_num;
    break;

  case TYPE_INSGPS:
    if (++read_sn_ != insgps_frame_.frame.header.serial_num)
    {
      if ( insgps_frame_.frame.header.serial_num < read_sn_)
      {
        sn_lost_ += 256 - (int)(read_sn_ - insgps_frame_.frame.header.serial_num);
        if(if_debug_){
          ROS_WARN("detected sn lost.");
        }
      }
      else
      {
        sn_lost_ += (int)(insgps_frame_.frame.header.serial_num - read_sn_);
        if(if_debug_){
          ROS_WARN("detected sn lost.");
        }
      }
    }
    read_sn_ = insgps_frame_.frame.header.serial_num;
    break;

  case TYPE_GEODETIC_POS:
    if (++read_sn_ != Geodetic_Position_frame_.frame.header.serial_num)
    {
      if ( Geodetic_Position_frame_.frame.header.serial_num < read_sn_)
      {
        sn_lost_ += 256 - (int)(read_sn_ - Geodetic_Position_frame_.frame.header.serial_num);
        if(if_debug_){
          ROS_WARN("detected sn lost.");
        }
      }
      else
      {
        sn_lost_ += (int)(Geodetic_Position_frame_.frame.header.serial_num - read_sn_);
        if(if_debug_){
          ROS_WARN("detected sn lost.");
        }
      }
    }
    read_sn_ = Geodetic_Position_frame_.frame.header.serial_num;
    break;

  default:
    break;
  }
}

} //namespace FDILink

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ahrs_bringup");
  FDILink::ahrsBringup bp;

  return 0;
}
