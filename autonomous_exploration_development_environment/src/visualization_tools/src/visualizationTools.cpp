#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

const double PI = 3.1415926;

string metricFile;
string trajFile;
string mapFile;
double overallMapVoxelSize = 0.5;
double exploredAreaVoxelSize = 0.3;
double exploredVolumeVoxelSize = 0.5;
double transInterval = 0.2;
double yawInterval = 10.0;
int overallMapDisplayInterval = 2;
int overallMapDisplayCount = 0;
int exploredAreaDisplayInterval = 1;
int exploredAreaDisplayCount = 0;

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZ>::Ptr overallMapCloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr overallMapCloudDwz(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZI>::Ptr exploredAreaCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr exploredAreaCloud2(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr exploredVolumeCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr exploredVolumeCloud2(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr trajectory(new pcl::PointCloud<pcl::PointXYZI>());

const int systemDelay = 5;
int systemDelayCount = 0;
bool systemDelayInited = false;
double systemTime = 0;
double systemInitTime = 0;
bool systemInited = false;

float vehicleYaw = 0;
float vehicleX = 0, vehicleY = 0, vehicleZ = 0;
float exploredVolume = 0, travelingDis = 0, runtime = 0, timeDuration = 0;

pcl::VoxelGrid<pcl::PointXYZ> overallMapDwzFilter;
pcl::VoxelGrid<pcl::PointXYZI> exploredAreaDwzFilter;
pcl::VoxelGrid<pcl::PointXYZI> exploredVolumeDwzFilter;

sensor_msgs::PointCloud2 overallMap2;

ros::Publisher *pubExploredAreaPtr = NULL;
ros::Publisher *pubTrajectoryPtr = NULL;
ros::Publisher *pubExploredVolumePtr = NULL;
ros::Publisher *pubTravelingDisPtr = NULL;
ros::Publisher *pubTimeDurationPtr = NULL;

FILE *metricFilePtr = NULL;
FILE *trajFilePtr = NULL;

void odometryHandler(const nav_msgs::Odometry::ConstPtr& odom)
{
  systemTime = odom->header.stamp.toSec();

  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  float dYaw = fabs(yaw - vehicleYaw);
  if (dYaw > PI) dYaw = 2 * PI  - dYaw;

  float dx = odom->pose.pose.position.x - vehicleX;
  float dy = odom->pose.pose.position.y - vehicleY;
  float dz = odom->pose.pose.position.z - vehicleZ;
  float dis = sqrt(dx * dx + dy * dy + dz * dz);

  if (!systemDelayInited) {
    vehicleYaw = yaw;
    vehicleX = odom->pose.pose.position.x;
    vehicleY = odom->pose.pose.position.y;
    vehicleZ = odom->pose.pose.position.z;
    return;
  }

  if (systemInited) {
    timeDuration = systemTime - systemInitTime;
    
    std_msgs::Float32 timeDurationMsg;
    timeDurationMsg.data = timeDuration;
    pubTimeDurationPtr->publish(timeDurationMsg);
  }

  if (dis < transInterval && dYaw < yawInterval) {
    return;
  }

  if (!systemInited) {
    dis = 0;
    systemInitTime = systemTime;
    systemInited = true;
  }

  travelingDis += dis;

  vehicleYaw = yaw;
  vehicleX = odom->pose.pose.position.x;
  vehicleY = odom->pose.pose.position.y;
  vehicleZ = odom->pose.pose.position.z;

  fprintf(trajFilePtr, "%f %f %f %f %f %f %f\n", vehicleX, vehicleY, vehicleZ, roll, pitch, yaw, timeDuration);

  pcl::PointXYZI point;
  point.x = vehicleX;
  point.y = vehicleY;
  point.z = vehicleZ;
  point.intensity = travelingDis;
  trajectory->push_back(point);

  sensor_msgs::PointCloud2 trajectory2;
  pcl::toROSMsg(*trajectory, trajectory2);
  trajectory2.header.stamp = odom->header.stamp;
  trajectory2.header.frame_id = "map";
  pubTrajectoryPtr->publish(trajectory2);
}

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudIn)
{
  if (!systemDelayInited) {
    systemDelayCount++;
    if (systemDelayCount > systemDelay) {
      systemDelayInited = true;
    }
  }

  if (!systemInited) {
    return;
  }

  laserCloud->clear();
  pcl::fromROSMsg(*laserCloudIn, *laserCloud);

  *exploredVolumeCloud += *laserCloud;

  exploredVolumeCloud2->clear();
  exploredVolumeDwzFilter.setInputCloud(exploredVolumeCloud);
  exploredVolumeDwzFilter.filter(*exploredVolumeCloud2);

  pcl::PointCloud<pcl::PointXYZI>::Ptr tempCloud = exploredVolumeCloud;
  exploredVolumeCloud = exploredVolumeCloud2;
  exploredVolumeCloud2 = tempCloud;

  exploredVolume = exploredVolumeVoxelSize * exploredVolumeVoxelSize * 
                   exploredVolumeVoxelSize * exploredVolumeCloud->points.size();

  *exploredAreaCloud += *laserCloud;

  exploredAreaDisplayCount++;
  if (exploredAreaDisplayCount >= 5 * exploredAreaDisplayInterval) {
    exploredAreaCloud2->clear();
    exploredAreaDwzFilter.setInputCloud(exploredAreaCloud);
    exploredAreaDwzFilter.filter(*exploredAreaCloud2);

    tempCloud = exploredAreaCloud;
    exploredAreaCloud = exploredAreaCloud2;
    exploredAreaCloud2 = tempCloud;

    sensor_msgs::PointCloud2 exploredArea2;
    pcl::toROSMsg(*exploredAreaCloud, exploredArea2);
    exploredArea2.header.stamp = laserCloudIn->header.stamp;
    exploredArea2.header.frame_id = "map";
    pubExploredAreaPtr->publish(exploredArea2);

    exploredAreaDisplayCount = 0;
  }

  fprintf(metricFilePtr, "%f %f %f %f\n", exploredVolume, travelingDis, runtime, timeDuration);

  std_msgs::Float32 exploredVolumeMsg;
  exploredVolumeMsg.data = exploredVolume;
  pubExploredVolumePtr->publish(exploredVolumeMsg);
  
  std_msgs::Float32 travelingDisMsg;
  travelingDisMsg.data = travelingDis;
  pubTravelingDisPtr->publish(travelingDisMsg);
}

void runtimeHandler(const std_msgs::Float32::ConstPtr& runtimeIn)
{
  runtime = runtimeIn->data;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visualizationTools");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("metricFile", metricFile);
  nhPrivate.getParam("trajFile", trajFile);
  nhPrivate.getParam("mapFile", mapFile);
  nhPrivate.getParam("overallMapVoxelSize", overallMapVoxelSize);
  nhPrivate.getParam("exploredAreaVoxelSize", exploredAreaVoxelSize);
  nhPrivate.getParam("exploredVolumeVoxelSize", exploredVolumeVoxelSize);
  nhPrivate.getParam("transInterval", transInterval);
  nhPrivate.getParam("yawInterval", yawInterval);
  nhPrivate.getParam("overallMapDisplayInterval", overallMapDisplayInterval);
  nhPrivate.getParam("exploredAreaDisplayInterval", exploredAreaDisplayInterval);

  ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry> ("/state_estimation", 5, odometryHandler);

  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2> ("/registered_scan", 5, laserCloudHandler);

  ros::Subscriber subRuntime = nh.subscribe<std_msgs::Float32> ("/runtime", 5, runtimeHandler);

  ros::Publisher pubOverallMap = nh.advertise<sensor_msgs::PointCloud2> ("/overall_map", 5);

  ros::Publisher pubExploredArea = nh.advertise<sensor_msgs::PointCloud2> ("/explored_areas", 5);
  pubExploredAreaPtr = &pubExploredArea;

  ros::Publisher pubTrajectory = nh.advertise<sensor_msgs::PointCloud2> ("/trajectory", 5);
  pubTrajectoryPtr = &pubTrajectory;

  ros::Publisher pubExploredVolume = nh.advertise<std_msgs::Float32> ("/explored_volume", 5);
  pubExploredVolumePtr = &pubExploredVolume;

  ros::Publisher pubTravelingDis = nh.advertise<std_msgs::Float32> ("/traveling_distance", 5);
  pubTravelingDisPtr = &pubTravelingDis;

  ros::Publisher pubTimeDuration = nh.advertise<std_msgs::Float32> ("/time_duration", 5);
  pubTimeDurationPtr = &pubTimeDuration;

  //ros::Publisher pubRuntime = nh.advertise<std_msgs::Float32> ("/runtime", 5);

  overallMapDwzFilter.setLeafSize(overallMapVoxelSize, overallMapVoxelSize, overallMapVoxelSize);
  exploredAreaDwzFilter.setLeafSize(exploredAreaVoxelSize, exploredAreaVoxelSize, exploredAreaVoxelSize);
  exploredVolumeDwzFilter.setLeafSize(exploredVolumeVoxelSize, exploredVolumeVoxelSize, exploredVolumeVoxelSize);

  pcl::PLYReader ply_reader;
  if (ply_reader.read(mapFile, *overallMapCloud) == -1) {
    printf("\nCouldn't read pointcloud.ply file.\n\n");
  }

  overallMapCloudDwz->clear();
  overallMapDwzFilter.setInputCloud(overallMapCloud);
  overallMapDwzFilter.filter(*overallMapCloudDwz);
  overallMapCloud->clear();

  pcl::toROSMsg(*overallMapCloudDwz, overallMap2);

  time_t logTime = time(0);
  tm *ltm = localtime(&logTime);
  string timeString = to_string(1900 + ltm->tm_year) + "-" + to_string(1 + ltm->tm_mon) + "-" + to_string(ltm->tm_mday) + "-" +
                      to_string(ltm->tm_hour) + "-" + to_string(ltm->tm_min) + "-" + to_string(ltm->tm_sec);

  metricFile += "_" + timeString + ".txt";
  trajFile += "_" + timeString + ".txt";
  metricFilePtr = fopen(metricFile.c_str(), "w");
  trajFilePtr = fopen(trajFile.c_str(), "w");

  ros::Rate rate(100);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();

    overallMapDisplayCount++;
    if (overallMapDisplayCount >= 100 * overallMapDisplayInterval) {
      overallMap2.header.stamp = ros::Time().fromSec(systemTime);
      overallMap2.header.frame_id = "map";
      pubOverallMap.publish(overallMap2);

      overallMapDisplayCount = 0;
    }

    status = ros::ok();
    rate.sleep();
  }

  fclose(metricFilePtr);
  fclose(trajFilePtr);

  printf("\nExploration metrics and vehicle trajectory are saved in 'src/vehicle_simulator/log'.\n\n");

  return 0;
}
