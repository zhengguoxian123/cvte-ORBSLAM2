
/**

@mainpage slam_gmapping

@htmlinclude manifest.html

@b slam_gmapping is a wrapper around the GMapping SLAM library. It reads laser
scans and odometry and computes a map. This map can be
written to a file using e.g.

  "rosrun map_server map_saver static_map:=dynamic_map"

<hr>

@section topic ROS topics

Subscribes to (name/type):
- @b "scan"/<a href="../../sensor_msgs/html/classstd__msgs_1_1LaserScan.html">sensor_msgs/LaserScan</a> : data from a laser range scanner 
- @b "/tf": odometry from the robot


Publishes to (name/type):
- @b "/tf"/tf/tfMessage: position relative to the map


@section services
 - @b "~dynamic_map" : returns the map


@section parameters ROS parameters

Reads the following parameters from the parameter server

Parameters used by our GMapping wrapper:

- @b "~throttle_scans": @b [int] throw away every nth laser scan
- @b "~base_frame": @b [string] the tf frame_id to use for the robot base pose
- @b "~map_frame": @b [string] the tf frame_id where the robot pose on the map is published
- @b "~odom_frame": @b [string] the tf frame_id from which odometry is read
- @b "~map_update_interval": @b [double] time in seconds between two recalculations of the map


Parameters used by GMapping itself:

Laser Parameters:
- @b "~/maxRange" @b [double] maximum range of the laser scans. Rays beyond this range get discarded completely. (default: maximum laser range minus 1 cm, as received in the the first LaserScan message)
- @b "~/maxUrange" @b [double] maximum range of the laser scanner that is used for map building (default: same as maxRange)
Initial map dimensions and resolution:
- @b "~/xmin" @b [double] minimum x position in the map [m]
- @b "~/ymin" @b [double] minimum y position in the map [m]
- @b "~/xmax" @b [double] maximum x position in the map [m]
- @b "~/ymax" @b [double] maximum y position in the map [m]
- @b "~/delta" @b [double] size of one pixel [m]

*/

#include "Mapping2d.h"

#include <iostream>

#include <time.h>

#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/MapMetaData.h"
#include "Converter.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))
namespace ORB_SLAM2
{
  Mapping2d::Mapping2d():
  map_to_odom_(tf::Transform(tf::createQuaternionFromRPY( 0, 0, 0 ), tf::Point(0, 0, 0 ))),private_nh_("~")
{
  seed_ = time(NULL);
  init();
}
 
 
void Mapping2d::MainMapping(vector<KeyFrame*> vpKFs)
{
  sensor_msgs::LaserScan scan = *(vpKFs[0]->mscan);
  
  // 上一次更新地图的时间
  static ros::Time last_map_update(0,0); //静态局部变量
  //std::cout<<" last_map_update"<<last_map_update.toSec()<<std::endl;
  
  // We can't initialize the mapper until we've got the first scan
  if(!got_first_scan_)
  {
    //步骤1：地图初始化
    if(!initMapper(scan)) 
      return;
    got_first_scan_ = true;
  }
  sensor_msgs::LaserScan LastScan = *(vpKFs[vpKFs.size()-1]->mscan);
  
  // 如果地图为空 或者 当前帧中的激光对应的时间 离 上一次更新地图 大于 map_update_interval_(5)秒， 则更新地图
    if(!got_map_ || (LastScan.header.stamp - last_map_update) > map_update_interval_)
    {
      // 步骤3： 更新并publish 2d地图，以及publish 估计位姿的信息熵(方差)
      
      updateMap(vpKFs);
      last_map_update = LastScan.header.stamp;
      ROS_DEBUG("Updated the map");
    }
    
  
}
void Mapping2d::init()
{
  got_first_scan_ = false;
  got_map_ = false;
  

  
  // Parameters used by our GMapping wrapper
  if(!private_nh_.getParam("base_frame", base_frame_))
    base_frame_ = "base_footprint";
  if(!private_nh_.getParam("map_frame", map_frame_))
    map_frame_ = "map";
  if(!private_nh_.getParam("odom_frame", odom_frame_))
    odom_frame_ = "odom";

  double tmp;
  if(!private_nh_.getParam("map_update_interval", tmp))
    tmp = 5.0;
  map_update_interval_.fromSec(tmp);
  
  // Parameters used by GMapping itself
  maxUrange_ = 0.0;  maxRange_ = 0.0; // preliminary default, will be set in initMapper()

  if(!private_nh_.getParam("xmin", xmin_))
    xmin_ = -1.0;
  if(!private_nh_.getParam("ymin", ymin_))
    ymin_ = -1.0;
  if(!private_nh_.getParam("xmax", xmax_))
    xmax_ = 1.0;
  if(!private_nh_.getParam("ymax", ymax_))
    ymax_ = 1.0;
  if(!private_nh_.getParam("delta", delta_))
    delta_ = 0.05;
  if(!private_nh_.getParam("occ_thresh", occ_thresh_))
    occ_thresh_ = 0.25;
  
  sst_ = node_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  sstm_ = node_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
}

Mapping2d::~Mapping2d()
{

}

//地图初始化
//1. 判断lidar是否安装在平面，以及其是否正前、正面向上安装
//2. 
bool
Mapping2d::initMapper(const sensor_msgs::LaserScan& scan)
{
  
  tf::StampedTransform laser_pose;
  tf_.waitForTransform("base_footprint", "laser", ros::Time(), ros::Duration(1.0));
  tf_.lookupTransform("base_footprint", "laser",ros::Time(), laser_pose);
  std::cout<<" get the tf_basefoot_laser"<<std::endl;

  
  // T_base_laser
  double yaw,pitch,roll;
  laser_pose.getBasis().getRPY(roll,pitch,yaw);//弧度
  tf::Vector3 v3 = laser_pose.getOrigin();
  m_laser_pose = GMapping::OrientedPoint(v3.getX(),v3.getY(),yaw);;
  ROS_INFO("m laser pose(T_base_laser)  is: x %.3f ,y %.3f, theta %.3f,", m_laser_pose.x, m_laser_pose.y, m_laser_pose.theta);
  
  /*
  // create a point 1m above the laser position and transform it into the laser-frame
  tf::Vector3 v;
  v.setValue(0, 0, 1 + laser_pose.getOrigin().z());
  tf::Stamped<tf::Vector3> up(v, scan.header.stamp,
                                      base_frame_);
  try
  {
    tf_.transformPoint(laser_frame_, up, up);
    ROS_DEBUG("Z-Axis in sensor frame: %.3f", up.z());
  }
  catch(tf::TransformException& e)
  {
    ROS_WARN("Unable to determine orientation of laser: %s",
             e.what());
    return false;
  }
  //由urdf参数决定
  // gmapping doesnt take roll or pitch into account. So check for correct sensor alignment.
  if (fabs(fabs(up.z()) - 1) > 0.001)
  {
    ROS_WARN("Laser has to be mounted planar! Z-coordinate has to be 1 or -1, but gave: %.5f",
                 up.z());
    return false;
  }
*/

  double angle_center = (scan.angle_min + scan.angle_max)/2;
  ROS_INFO("angle center of laser is %.3f", angle_center);
  ROS_INFO("scan.angle_min is  %.3f", scan.angle_min);
  ROS_INFO("scan.angle_max is  %.3f", scan.angle_max);
 /* // rplidar 正放
  if (up.z() > 0)
  {
    do_reverse_range_ = scan.angle_min > scan.angle_max;
    centered_laser_pose_ = tf::Stamped<tf::Pose>(tf::Transform(tf::createQuaternionFromRPY(0,0,angle_center),
                                                               tf::Vector3(0,0,0)), ros::Time::now(), laser_frame_); //The frame_id associated this data. 
    ROS_INFO("Laser is mounted upwards.");
  }
  //rplidar 倒放
  else
  {
    do_reverse_range_ = scan.angle_min < scan.angle_max;
    centered_laser_pose_ = tf::Stamped<tf::Pose>(tf::Transform(tf::createQuaternionFromRPY(M_PI,0,-angle_center),
                                                               tf::Vector3(0,0,0)), ros::Time::now(), laser_frame_);
    ROS_INFO("Laser is mounted upside down.");
  }
  */
  
    do_reverse_range_ = scan.angle_min > scan.angle_max;
    centered_laser_pose_ = tf::Stamped<tf::Pose>(tf::Transform(tf::createQuaternionFromRPY(0,0,angle_center),
                                                               tf::Vector3(0,0,0)), ros::Time::now(), laser_frame_); //The frame_id associated this data. 
    ROS_INFO("Laser is mounted upwards.");
  // Compute the angles of the laser from -x to x, basically symmetric and in increasing order
  laser_angles_.resize(scan.ranges.size());
  // Make sure angles are started so that they are centered
  double theta = - std::fabs(scan.angle_min - scan.angle_max)/2;
  for(unsigned int i=0; i<scan.ranges.size(); ++i)
  {
    laser_angles_[i]=theta;
    theta += std::fabs(scan.angle_increment);
  }

  ROS_DEBUG("Laser angles in laser-frame: min: %.3f max: %.3f inc: %.3f", scan.angle_min, scan.angle_max,
            scan.angle_increment);
  ROS_DEBUG("Laser angles in top-down centered laser-frame: min: %.3f max: %.3f inc: %.3f", laser_angles_.front(),
            laser_angles_.back(), std::fabs(scan.angle_increment));

  // setting maxRange and maxUrange here so we can set a reasonable default
  ros::NodeHandle private_nh_("~");
  if(!private_nh_.getParam("maxRange", maxRange_))
    maxRange_ = 6.0;
  if(!private_nh_.getParam("maxUrange", maxUrange_))
    maxUrange_ = maxRange_ - 0.5;

  ROS_INFO("Initialization complete");

  return true;
}


//更新并publish 2d地图
void
Mapping2d::updateMap(vector<KeyFrame*> vpKFs)
{
  ROS_DEBUG("Update map");
  ROS_INFO("Using ORB-SLAM2 to update map");
  // 步骤1. 构建ScanMatcher 对象，用来获得2d概率地图
  GMapping::ScanMatcher matcher;
  
  //T_base_laser
  ROS_INFO("laser pose (T_base_laser)  is: x %.3f ,y %.3f, theta %.3f,", m_laser_pose.x, m_laser_pose.y, m_laser_pose.theta);
  
  sensor_msgs::LaserScan LastScan = *(vpKFs[vpKFs.size()-1]->mscan);
  matcher.setLaserParameters(LastScan.ranges.size(), &(laser_angles_[0]),
                             m_laser_pose);//gsp_laser_->getPose(): T_base_laser

  matcher.setlaserMaxRange(maxRange_);
  matcher.setusableRange(maxUrange_);
  matcher.setgenerateMap(true);

  if(!got_map_) {
    map_.map.info.resolution = delta_;
    map_.map.info.origin.position.x = 0.0;
    map_.map.info.origin.position.y = 0.0;
    map_.map.info.origin.position.z = 0.0;
    map_.map.info.origin.orientation.x = 0.0;
    map_.map.info.origin.orientation.y = 0.0;
    map_.map.info.origin.orientation.z = 0.0;
    map_.map.info.origin.orientation.w = 1.0;
  } 
  
   GMapping::Point center;
   center.x=(xmin_ + xmax_) / 2.0;
   center.y=(ymin_ + ymax_) / 2.0;
   //空白地图
   GMapping::ScanMatcherMap m_smap(center, xmin_, ymin_, xmax_, ymax_, 
                                delta_);// 配置地图相关参数，如分辨率
  
   for(size_t i=0; i<vpKFs.size();i++)
   {
     KeyFrame* pKF = vpKFs[i];
     
     //[1] 计算T_camera0_camera : [Rwc | twc]
     cv::Mat Rwc = pKF->GetRotation().t();// R_camera0_camera
     cv::Mat twc = pKF->GetCameraCenter();// t_camera0_camera
     vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);
     tf::Transform T_camera0_camera(tf::Quaternion(q[0], q[1], q[2],q[3]),tf::Vector3(twc.at<float>(0),twc.at<float>(1),twc.at<float>(2)) ); //T_camera0_camera
   
     //[2] 给定 T_base-footprint_camera-rgb-optical
     tf::Transform T_base_camera(tf::Quaternion(-0.5, 0.5, -0.5,0.5),tf::Vector3(0.075,-0.099,0.783) );
     
     //[3] 计算T_map_base-footprint, T_map_camera0 = T_base-footprint_camera-rgb-optical
     //T_map_base-footprint =T_map_camera0 * T_camera0_camera * tf_basefoot_camera.inverse()
     tf::Transform T_map_base = T_base_camera * T_camera0_camera *T_base_camera.inverse();
     
     //[4] 类型转换： tf -> OrientedPoint
     double yaw,pitch,roll;
     T_map_base.getBasis().getRPY(roll,pitch,yaw);//弧度
     tf::Vector3 v = T_map_base.getOrigin();
     GMapping::OrientedPoint robot_pose(v.getX(),v.getY(),yaw);
     
     
     // 对激光数据进行预处理 
     // GMapping wants an array of doubles...
     sensor_msgs::LaserScan scan = *(pKF->mscan);
     double* ranges_double = new double[scan.ranges.size()];
     // If the angle increment is negative, we have to invert the order of the readings.
     // 在地图初始化initMapper中，为do_reverse_range_赋值
     if (do_reverse_range_)
    {
       ROS_DEBUG("Inverting scan");
       int num_ranges = scan.ranges.size();
       for(int i=0; i < num_ranges; i++)
      {
        // Must filter out short readings, because the mapper won't
        if(scan.ranges[num_ranges - i - 1] < scan.range_min)
          ranges_double[i] = (double)scan.range_max;
        else
          ranges_double[i] = (double)scan.ranges[num_ranges - i - 1];
      }
    } 
    else 
   {
      for(unsigned int i=0; i < scan.ranges.size(); i++)
      {
        // Must filter out short readings, because the mapper won't
        if(scan.ranges[i] < scan.range_min)
          ranges_double[i] = (double)scan.range_max;
        else
          ranges_double[i] = (double)scan.ranges[i];
      }
   }
      // 构建2d概率地图
    matcher.invalidateActiveArea();
    matcher.computeActiveArea(m_smap, robot_pose, ranges_double); //设置map.m_activeArea 中的可活动区域
    matcher.registerScan(m_smap, robot_pose, ranges_double);
    delete[] ranges_double;
   }

  // the map may have expanded, so resize ros message as well
  //步骤2： 拓展地图，重新计算长宽
  if(map_.map.info.width != (unsigned int) m_smap.getMapSizeX() || map_.map.info.height != (unsigned int) m_smap.getMapSizeY()) {

    // NOTE: The results of ScanMatcherMap::getSize() are different from the parameters given to the constructor
    //       so we must obtain the bounding box in a different way
    GMapping::Point wmin = m_smap.map2world(GMapping::IntPoint(0, 0));
    GMapping::Point wmax = m_smap.map2world(GMapping::IntPoint(m_smap.getMapSizeX(), m_smap.getMapSizeY()));
    xmin_ = wmin.x; ymin_ = wmin.y;
    xmax_ = wmax.x; ymax_ = wmax.y;
    
    ROS_DEBUG("map size is now %dx%d pixels (%f,%f)-(%f, %f)", m_smap.getMapSizeX(), m_smap.getMapSizeY(),
              xmin_, ymin_, xmax_, ymax_);

    map_.map.info.width = m_smap.getMapSizeX();
    map_.map.info.height = m_smap.getMapSizeY();
    map_.map.info.origin.position.x = xmin_;
    map_.map.info.origin.position.y = ymin_;
    map_.map.data.resize(map_.map.info.width * map_.map.info.height);

    ROS_DEBUG("map origin: (%f, %f)", map_.map.info.origin.position.x, map_.map.info.origin.position.y);
  }
  //步骤3： 根据概率地图中概率为 map地图中每个点赋值
  // occ < 0, -1;
  // occ > occ_thresh, 100;
  // else 0
  for(int x=0; x < m_smap.getMapSizeX(); x++)
  {
    for(int y=0; y < m_smap.getMapSizeY(); y++)
    {
      /// @todo Sort out the unknown vs. free vs. obstacle thresholding
      GMapping::IntPoint p(x, y);
      double occ=m_smap.cell(p); //由operator double()计算
      /*
       if(occ >0.0)
      {
      std::cout<< "double occ :"<< occ << std::endl;
      std::cout << "operator double():" << (double)(smap.cell(p))<<std::endl; 
      }
      */
      assert(occ <= 1.0);
      if(occ < 0)
	// data[width *y + x ] 对应于地图上(x,y)点
	//未知区域
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = -1;
      else if(occ > occ_thresh_)
      {
	//障碍区域
        //map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = (int)round(occ*100.0);
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 100;
      }
      else
	//空的区域
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 0;
    }
  }
  got_map_ = true;
  
  //make sure to set the header information on the map
  // 步骤4： publish map和map.info
  // 实际上rosrun map_server map_saver -f /tmp/my_map 只是通过订阅map来构建地图
  
  map_.map.header.stamp = ros::Time::now();
  map_.map.header.frame_id = tf_.resolve( map_frame_ );

  sst_.publish(map_.map);
  sstm_.publish(map_.map.info);
}

}