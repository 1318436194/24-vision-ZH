#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <gazebo_msgs/ModelState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Joy.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

const double PI = 3.1415926;

bool use_gazebo_time = false;
double cameraOffsetZ = 0;
double sensorOffsetX = 0;
double sensorOffsetY = 0;
double vehicleHeight = 0.75;
double terrainVoxelSize = 0.05;
double groundHeightThre = 0.1;
bool adjustZ = true;
double terrainRadiusZ = 0.5;
int minTerrainPointNumZ = 10;
double smoothRateZ = 0.2;
bool adjustIncl = true;
double terrainRadiusIncl = 1.5;
int minTerrainPointNumIncl = 500;
double smoothRateIncl = 0.2;
double InclFittingThre = 0.2;
double maxIncl = 30.0;

const int systemDelay = 5;
int systemInitCount = 0;
bool systemInited = false;

//创建点云变量，float x, y, z, intensity; 表示XYZ信息加上强度信息的类型
pcl::PointCloud<pcl::PointXYZI>::Ptr scanData(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudIncl(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudDwz(new pcl::PointCloud<pcl::PointXYZI>());

//???
std::vector<int> scanInd;

//里程计坐标系时间
ros::Time odomTime;

float vehicleX = 0;
float vehicleY = 0;
float vehicleZ = 0;
float vehicleRoll = 0;
float vehiclePitch = 0;
float vehicleYaw = 0;

float vehicleYawRate = 0;
float vehicleSpeed = 0;

float terrainZ = 0;
float terrainRoll = 0;
float terrainPitch = 0;

//创建堆栈长度常量
const int stackNum = 400;
//用于存放连续时间内位置和姿态的数组
float vehicleXStack[stackNum];
float vehicleYStack[stackNum];
float vehicleZStack[stackNum];
float vehicleRollStack[stackNum];
float vehiclePitchStack[stackNum];
float vehicleYawStack[stackNum];
float terrainRollStack[stackNum];
float terrainPitchStack[stackNum];
double odomTimeStack[stackNum];
int odomSendIDPointer = -1;
int odomRecIDPointer = 0;

/*VoxelGrid 体素滤波器
　pcl库中的VoxelGrid对点云进行体素化，主要就是创建一个三维体素栅格(就是每个比较小的立方体组成的体素栅格)。
 在每个体素(三维立方体)里面，求取该立方体内的所有点云重心点来代表这个立方体的表示，以此达到下采样的目的
 将点云中的每一个点与其周围的每领域比较，将其分成成分相似的点集，减少点云中的噪声和离群值*/
pcl::VoxelGrid<pcl::PointXYZI> terrainDwzFilter;

//创建雷达点云发布者
ros::Publisher *pubScanPointer = NULL;

//雷达信息回调处理函数
void scanHandler(const sensor_msgs::PointCloud2::ConstPtr &scanIn)
{

  //第一次调用回调函数时给个延时？
  if (!systemInited)
  {
    systemInitCount++;
    if (systemInitCount > systemDelay)
    {
      systemInited = true;
    }
    return;
  }

  //获取传入的雷达数据扫描时间戳
  double scanTime = scanIn->header.stamp.toSec();

  //时间同步？
  if (odomSendIDPointer < 0)
  {
    return;
  }
  while (odomTimeStack[(odomRecIDPointer + 1) % stackNum] < scanTime &&
         odomRecIDPointer != (odomSendIDPointer + 1) % stackNum)
  {
    odomRecIDPointer = (odomRecIDPointer + 1) % stackNum;
  }

  //获取位姿信息
  double odomRecTime = odomTime.toSec();
  float vehicleRecX = vehicleX;
  float vehicleRecY = vehicleY;
  float vehicleRecZ = vehicleZ;
  float vehicleRecRoll = vehicleRoll;
  float vehicleRecPitch = vehiclePitch;
  float vehicleRecYaw = vehicleYaw;
  float terrainRecRoll = terrainRoll;
  float terrainRecPitch = terrainPitch;

  if (use_gazebo_time)
  {
    odomRecTime = odomTimeStack[odomRecIDPointer];
    vehicleRecX = vehicleXStack[odomRecIDPointer];
    vehicleRecY = vehicleYStack[odomRecIDPointer];
    vehicleRecZ = vehicleZStack[odomRecIDPointer];
    vehicleRecRoll = vehicleRollStack[odomRecIDPointer];
    vehicleRecPitch = vehiclePitchStack[odomRecIDPointer];
    vehicleRecYaw = vehicleYawStack[odomRecIDPointer];
    terrainRecRoll = terrainRollStack[odomRecIDPointer];
    terrainRecPitch = terrainPitchStack[odomRecIDPointer];
  }

  float sinTerrainRecRoll = sin(terrainRecRoll);
  float cosTerrainRecRoll = cos(terrainRecRoll);
  float sinTerrainRecPitch = sin(terrainRecPitch);
  float cosTerrainRecPitch = cos(terrainRecPitch);

  //每次处理雷达信息前，先清空原有的消息
  scanData->clear();

  //ROS中定义的点云与PCL定义的点云数据转换
  pcl::fromROSMsg(*scanIn, *scanData);

  /*从传感器获得的点云可能包含几种测量误差和/或不准确。其中之一是在一些点的坐标中存在NaN（不是数）值
   *点云对象的成员函数有称为“is_dense()”，如果所有的点都有效的返回true是为有限值。
   * 一个NaNs表明测量传感器距离到该点的距离值是有问题的，可能是因为传感器太近或太远，或者因为表面反射。
   * 当存在无效点云的NaNs值作为算法的输入的时候，可能会引起很多问题,所以需要去除 */
  pcl::removeNaNFromPointCloud(*scanData, *scanData, scanInd);

  //通过接收到的Terrain和vehicle的位姿计算点云的世界坐标？？？
  int scanDataSize = scanData->points.size();
  for (int i = 0; i < scanDataSize; i++)
  {
    float pointX1 = scanData->points[i].x;
    float pointY1 = scanData->points[i].y * cosTerrainRecRoll - scanData->points[i].z * sinTerrainRecRoll;
    float pointZ1 = scanData->points[i].y * sinTerrainRecRoll + scanData->points[i].z * cosTerrainRecRoll;

    float pointX2 = pointX1 * cosTerrainRecPitch + pointZ1 * sinTerrainRecPitch;
    float pointY2 = pointY1;
    float pointZ2 = -pointX1 * sinTerrainRecPitch + pointZ1 * cosTerrainRecPitch;

    float pointX3 = pointX2 + vehicleRecX;
    float pointY3 = pointY2 + vehicleRecY;
    float pointZ3 = pointZ2 + vehicleRecZ;

    scanData->points[i].x = pointX3;
    scanData->points[i].y = pointY3;
    scanData->points[i].z = pointZ3;
  }

  // publish 5Hz registered scan messages
  sensor_msgs::PointCloud2 scanData2;

  //PCL中定义的点云与ROS定义的点云数据转换
  pcl::toROSMsg(*scanData, scanData2);

  //发布地图坐标系点云信息，以接收到的里程计时间为时间戳
  scanData2.header.stamp = ros::Time().fromSec(odomRecTime);
  scanData2.header.frame_id = "map";
  pubScanPointer->publish(scanData2);
}

//地图坐标系点云信息回调处理函数
void terrainCloudHandler(const sensor_msgs::PointCloud2ConstPtr &terrainCloud2)
{
  if (!adjustZ && !adjustIncl)
  {
    return;
  }

  terrainCloud->clear();
  pcl::fromROSMsg(*terrainCloud2, *terrainCloud);

  pcl::PointXYZI point;
  terrainCloudIncl->clear();
  int terrainCloudSize = terrainCloud->points.size();
  double elevMean = 0;
  int elevCount = 0;
  bool terrainValid = true;

  //
  for (int i = 0; i < terrainCloudSize; i++)
  {
    point = terrainCloud->points[i];

    float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) + (point.y - vehicleY) * (point.y - vehicleY));

    if (dis < terrainRadiusZ)
    {
      //intensity是什么？？？
      if (point.intensity < groundHeightThre)
      {
        elevMean += point.z;
        elevCount++;
      }
      else
      {
        terrainValid = false;
      }
    }

    if (dis < terrainRadiusIncl && point.intensity < groundHeightThre)
    {
      terrainCloudIncl->push_back(point);
    }
  }

  if (elevCount >= minTerrainPointNumZ)
    elevMean /= elevCount;
  else
    terrainValid = false;

  //看不懂，矫正z坐标？？？
  if (terrainValid && adjustZ)
  {
    terrainZ = (1.0 - smoothRateZ) * terrainZ + smoothRateZ * elevMean;
  }

  //体素滤波
  terrainCloudDwz->clear();
  terrainDwzFilter.setInputCloud(terrainCloudIncl);
  terrainDwzFilter.filter(*terrainCloudDwz);
  int terrainCloudDwzSize = terrainCloudDwz->points.size();

  //若合法点云数量太少，直接返回
  if (terrainCloudDwzSize < minTerrainPointNumIncl || !terrainValid)
  {
    return;
  }

  cv::Mat matA(terrainCloudDwzSize, 2, CV_32F, cv::Scalar::all(0));
  cv::Mat matAt(2, terrainCloudDwzSize, CV_32F, cv::Scalar::all(0));
  cv::Mat matAtA(2, 2, CV_32F, cv::Scalar::all(0));
  cv::Mat matB(terrainCloudDwzSize, 1, CV_32F, cv::Scalar::all(0));
  cv::Mat matAtB(2, 1, CV_32F, cv::Scalar::all(0));
  cv::Mat matX(2, 1, CV_32F, cv::Scalar::all(0));

  int inlierNum = 0;
  matX.at<float>(0, 0) = terrainPitch;
  matX.at<float>(1, 0) = terrainRoll;
  for (int iterCount = 0; iterCount < 5; iterCount++)
  {
    int outlierCount = 0;
    for (int i = 0; i < terrainCloudDwzSize; i++)
    {
      point = terrainCloudDwz->points[i];

      matA.at<float>(i, 0) = -point.x + vehicleX;
      matA.at<float>(i, 1) = point.y - vehicleY;
      matB.at<float>(i, 0) = point.z - elevMean;

      if (fabs(matA.at<float>(i, 0) * matX.at<float>(0, 0) + matA.at<float>(i, 1) * matX.at<float>(1, 0) -
               matB.at<float>(i, 0)) > InclFittingThre &&
          iterCount > 0)
      {
        matA.at<float>(i, 0) = 0;
        matA.at<float>(i, 1) = 0;
        matB.at<float>(i, 0) = 0;
        outlierCount++;
      }
    }

    cv::transpose(matA, matAt);
    matAtA = matAt * matA;
    matAtB = matAt * matB;
    cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

    if (inlierNum == terrainCloudDwzSize - outlierCount)
      break;
    inlierNum = terrainCloudDwzSize - outlierCount;
  }

  if (inlierNum < minTerrainPointNumIncl || fabs(matX.at<float>(0, 0)) > maxIncl * PI / 180.0 ||
      fabs(matX.at<float>(1, 0)) > maxIncl * PI / 180.0)
  {
    terrainValid = false;
  }

  if (terrainValid && adjustIncl)
  {
    terrainPitch = (1.0 - smoothRateIncl) * terrainPitch + smoothRateIncl * matX.at<float>(0, 0);
    terrainRoll = (1.0 - smoothRateIncl) * terrainRoll + smoothRateIncl * matX.at<float>(1, 0);
  }
}

void speedHandler(const geometry_msgs::TwistStamped::ConstPtr &speedIn)
{
  vehicleSpeed = speedIn->twist.linear.x;
  vehicleYawRate = speedIn->twist.angular.z;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vehicleSimulator");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  //从参数服务器获取参数
  nhPrivate.getParam("use_gazebo_time", use_gazebo_time);
  nhPrivate.getParam("cameraOffsetZ", cameraOffsetZ);
  nhPrivate.getParam("sensorOffsetX", sensorOffsetX);
  nhPrivate.getParam("sensorOffsetY", sensorOffsetY);
  nhPrivate.getParam("vehicleHeight", vehicleHeight);
  nhPrivate.getParam("vehicleX", vehicleX);
  nhPrivate.getParam("vehicleY", vehicleY);
  nhPrivate.getParam("vehicleZ", vehicleZ);
  nhPrivate.getParam("terrainZ", terrainZ);
  nhPrivate.getParam("vehicleYaw", vehicleYaw);
  nhPrivate.getParam("terrainVoxelSize", terrainVoxelSize);
  nhPrivate.getParam("groundHeightThre", groundHeightThre);
  nhPrivate.getParam("adjustZ", adjustZ);
  nhPrivate.getParam("terrainRadiusZ", terrainRadiusZ);
  nhPrivate.getParam("minTerrainPointNumZ", minTerrainPointNumZ);
  nhPrivate.getParam("adjustIncl", adjustIncl);
  nhPrivate.getParam("terrainRadiusIncl", terrainRadiusIncl);
  nhPrivate.getParam("minTerrainPointNumIncl", minTerrainPointNumIncl);
  nhPrivate.getParam("InclFittingThre", InclFittingThre);
  nhPrivate.getParam("maxIncl", maxIncl);

  //订阅雷达信息
  ros::Subscriber subScan = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 2, scanHandler);

  //订阅地形图
  ros::Subscriber subTerrainCloud = nh.subscribe<sensor_msgs::PointCloud2>("/terrain_map", 2, terrainCloudHandler);

  //订阅速度信息
  ros::Subscriber subSpeed = nh.subscribe<geometry_msgs::TwistStamped>("/cmd_vel", 5, speedHandler);

  //创建里程计发布者
  ros::Publisher pubVehicleOdom = nh.advertise<nav_msgs::Odometry>("/odom", 5);

  //里程计消息odom->base_link
  nav_msgs::Odometry odomData;
  odomData.header.frame_id = "odom";
  odomData.child_frame_id = "base_link";

  //odom->base_link的坐标变换
  tf::TransformBroadcaster tfBroadcaster;
  tf::StampedTransform odomTrans;
  odomTrans.frame_id_ = "odom";
  odomTrans.child_frame_id_ = "base_link";

  //机器人状态发布
  ros::Publisher pubModelState = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 5);
  gazebo_msgs::ModelState cameraState;
  cameraState.model_name = "camera";
  gazebo_msgs::ModelState lidarState;
  lidarState.model_name = "lidar";
  gazebo_msgs::ModelState robotState;
  robotState.model_name = "robot";

  ros::Publisher pubScan = nh.advertise<sensor_msgs::PointCloud2>("/registered_scan", 2);
  pubScanPointer = &pubScan;

  /*设置点云滤波器叶子大小，控制滤波器的精度和效率
   * 小叶子适用于高表面密度的点云，反之亦然*/
  terrainDwzFilter.setLeafSize(terrainVoxelSize, terrainVoxelSize, terrainVoxelSize);

  printf("\nSimulation started.\n\n");

  //发布频率
  ros::Rate rate(200);
  bool status = ros::ok();
  while (status)
  {
    ros::spinOnce();

    float vehicleRecRoll = vehicleRoll;
    float vehicleRecPitch = vehiclePitch;
    float vehicleRecZ = vehicleZ;

    vehicleRoll = terrainRoll * cos(vehicleYaw) + terrainPitch * sin(vehicleYaw);
    vehiclePitch = -terrainRoll * sin(vehicleYaw) + terrainPitch * cos(vehicleYaw);
    vehicleYaw += 0.005 * vehicleYawRate;
    if (vehicleYaw > PI)
      vehicleYaw -= 2 * PI;
    else if (vehicleYaw < -PI)
      vehicleYaw += 2 * PI;

    vehicleX += 0.005 * cos(vehicleYaw) * vehicleSpeed +
                0.005 * vehicleYawRate * (-sin(vehicleYaw) * sensorOffsetX - cos(vehicleYaw) * sensorOffsetY);
    vehicleY += 0.005 * sin(vehicleYaw) * vehicleSpeed +
                0.005 * vehicleYawRate * (cos(vehicleYaw) * sensorOffsetX - sin(vehicleYaw) * sensorOffsetY);
    vehicleZ = terrainZ + vehicleHeight;

    ros::Time odomTimeRec = odomTime;
    odomTime = ros::Time::now();
    if (odomTime == odomTimeRec)
      odomTime += ros::Duration(0.005);

    odomSendIDPointer = (odomSendIDPointer + 1) % stackNum;
    odomTimeStack[odomSendIDPointer] = odomTime.toSec();
    vehicleXStack[odomSendIDPointer] = vehicleX;
    vehicleYStack[odomSendIDPointer] = vehicleY;
    vehicleZStack[odomSendIDPointer] = vehicleZ;
    vehicleRollStack[odomSendIDPointer] = vehicleRoll;
    vehiclePitchStack[odomSendIDPointer] = vehiclePitch;
    vehicleYawStack[odomSendIDPointer] = vehicleYaw;
    terrainRollStack[odomSendIDPointer] = terrainRoll;
    terrainPitchStack[odomSendIDPointer] = terrainPitch;

    // publish 200Hz odometry messages
    //!!!
    geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(vehicleRoll, vehiclePitch, vehicleYaw);

    odomData.header.stamp = odomTime;
    odomData.pose.pose.orientation = geoQuat;
    odomData.pose.pose.position.x = vehicleX;
    odomData.pose.pose.position.y = vehicleY;
    odomData.pose.pose.position.z = vehicleZ;
    odomData.twist.twist.angular.x = 200.0 * (vehicleRoll - vehicleRecRoll);
    odomData.twist.twist.angular.y = 200.0 * (vehiclePitch - vehicleRecPitch);
    odomData.twist.twist.angular.z = vehicleYawRate;
    odomData.twist.twist.linear.x = vehicleSpeed;
    odomData.twist.twist.linear.z = 200.0 * (vehicleZ - vehicleRecZ);
    pubVehicleOdom.publish(odomData);

    // publish 200Hz tf messages
    odomTrans.stamp_ = odomTime;
    odomTrans.setRotation(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w));

    //vehicleZ->0.1
    odomTrans.setOrigin(tf::Vector3(vehicleX, vehicleY, 0.1));
    tfBroadcaster.sendTransform(odomTrans);

    // publish 200Hz Gazebo model state messages (this is for Gazebo simulation)
    cameraState.pose.orientation = geoQuat;
    cameraState.pose.position.x = vehicleX;
    cameraState.pose.position.y = vehicleY;
    cameraState.pose.position.z = vehicleZ + cameraOffsetZ;
    pubModelState.publish(cameraState);

    robotState.pose.orientation = geoQuat;
    robotState.pose.position.x = vehicleX;
    robotState.pose.position.y = vehicleY;
    robotState.pose.position.z = vehicleZ;
    pubModelState.publish(robotState);

    //0->vehicleYaw
    geoQuat = tf::createQuaternionMsgFromRollPitchYaw(terrainRoll, terrainPitch, 0);

    lidarState.pose.orientation = geoQuat;
    lidarState.pose.position.x = vehicleX;
    lidarState.pose.position.y = vehicleY;
    lidarState.pose.position.z = vehicleZ;
    pubModelState.publish(lidarState);

    status = ros::ok();
    rate.sleep();
  }

  return 0;
}