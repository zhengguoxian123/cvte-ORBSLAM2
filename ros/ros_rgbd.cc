#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <linux/input.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
//vikit
#include <vikit/pinhole_camera.h>
#include <vikit/math_utils.h>
#include <vikit/file_reader.h>
#include <vikit/timer.h>
#include <vikit/blender_utils.h>
// RGBD-DSLAM
#include "Thirdparty/RGBD-DSLAM/include/feature_detection.h"
#include "Thirdparty/RGBD-DSLAM/include/sparse_img_align.h"
#include "Thirdparty/RGBD-DSLAM/include/frame.h"
#include "Thirdparty/RGBD-DSLAM/include/point.h"
#include "Thirdparty/RGBD-DSLAM/include/feature.h"
#include "Thirdparty/RGBD-DSLAM/include/config.h"
#include "Thirdparty/RGBD-DSLAM/include/test_utils.h"

#include <boost/thread.hpp>
#include <opencv2/core/core.hpp>
#include "Converter.h"
#include "System.h"

using namespace std;
using namespace Eigen;

struct CAMERA_INTRINSIC_PARAMETERS
{
    float cx, cy, fx, fy, k1,k2,p1,p2,k3,scale,width,height;
};

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM,ros::NodeHandle &nh,const string &strSettingsFile,bool buse_odom):mpSLAM(pSLAM),mnh(nh),bpublishtf(false),bfirst_listent(true)
    {
      modom_sub = mnh.subscribe("odom",1,&ImageGrabber::odomCallback,this);
      mvel_pub = mnh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 10);
      mimage_pub = mnh.advertise<sensor_msgs::Image>("/Direct_Image", 10);
      
      mptRecovery = new boost::thread(boost::bind(&ImageGrabber::RotateRecovery, this));
      
      mOdom.pose.pose.position.x = 0.0;
      mOdom.pose.pose.position.y = 0.0;
      mOdom.pose.pose.position.z = 0.0;
      mOdom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
      mLastOdom = mOdom;
      mCurrentOdom = mOdom;
      maxlostcounts = 250;
      
      buseodom = buse_odom;
      busedirect = !buse_odom;
      
      //读取相机内参--Use for direct method
      cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
           if(!fsSettings.isOpened())
           {
              cerr << "Failed to open settings file at: " <<strSettingsFile<< endl;
              exit(-1);
           }
     
      mc.cx = fsSettings["Camera.cx"];
      mc.cy = fsSettings["Camera.cy"];
      mc.fx = fsSettings["Camera.fx"];
      mc.fy = fsSettings["Camera.fy"];
      mc.k1= fsSettings["Camera.k1"];
      mc.k2= fsSettings["Camera.k2"];
      mc.p1= fsSettings["Camera.p1"];
      mc.p2= fsSettings["Camera.p2"];
      mc.k3= fsSettings["Camera.k3"];
      mc.width =fsSettings["Camera.width"];
      mc.height = fsSettings["Camera.height"];
      mc.scale = fsSettings["DepthMapFactor"];
      
      mcam  = new vk::PinholeCamera(mc.width, mc.height, mc.fx, mc.fy, mc.cx, mc.cy,
                                                         mc.k1,mc.k2,mc.p1,mc.p2,mc.k3 );
      // 构造fast特征提取器
      RGBD_DSLAM::Config::triangMinCornerScore() = 20;
      RGBD_DSLAM::Config::kltMinLevel() = 0;
      mfeature_detector = new RGBD_DSLAM::feature_detection::FastDetector(mcam->width(), mcam->height(), RGBD_DSLAM::Config::gridSize(), RGBD_DSLAM::Config::nPyrLevels());
    
      //cvNamedWindow("image",CV_WINDOW_AUTOSIZE); //运行出错，与ORB-SLAM有冲突
      mcolor_cur = cv::Mat();
      mdepth_cur = cv::Mat();
    }
    ~ImageGrabber()
    {
        mptRecovery->interrupt();
        mptRecovery->join();

        delete mptRecovery;
    }
    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);
    void PublishPose(cv::Mat Tcw);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);
    void CreateTfFromXYtheta(double x,double y,double theta, tf::Transform& t);
    void RotateRecovery();//连续一定帧数跟丢后，执行原地旋转
    
    ros::Subscriber modom_sub;//订阅odom 话题
    ORB_SLAM2::System* mpSLAM;
    ros::NodeHandle mnh;
    ros::Publisher* pPosPub;//发布位姿态
    tf::TransformBroadcaster* pbroadcaster;
    ros::Publisher mvel_pub;
    boost::thread* mptRecovery;
    
    bool bpublishtf;
    bool bfirst_listent;
    bool btracked;//ORB_SLAM2跟踪状态
    int  mlostcounts;//统计连续跟丢帧数
    int  maxlostcounts;//最大跟丢数，超过则执行RotateRecovery()
    
    tf::StampedTransform T_basefoot_camera;
    tf::TransformListener mtf;// 订阅tf
    double mlasttime, mcurrenttime;
    // 解决跟丢：里程计odom 
    bool buseodom;
    nav_msgs::Odometry mOdom;
    nav_msgs::Odometry mLastOdom;
    nav_msgs::Odometry mCurrentOdom;
    tf::Transform mT_map_last;// 保存ORBSLAM上一次成功跟踪时的T_map_bastfoot(2d)
    
    //解决跟丢：RGBD-DSLAM
    bool busedirect;
    cv::Mat mcolor_last, mdepth_last, mcolor_cur, mdepth_cur;
    vk::PinholeCamera* mcam;
    CAMERA_INTRINSIC_PARAMETERS mc;
    RGBD_DSLAM::feature_detection::FastDetector* mfeature_detector;
    RGBD_DSLAM::FramePtr mframe_ref;
    RGBD_DSLAM::FramePtr mframe_cur;
    ros::Publisher mimage_pub;
};

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // 保存odom数据
    mLastOdom = mCurrentOdom;
    mCurrentOdom = mOdom;
    
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    // Use for Direct Method
    //mcolor_last = mcolor_cur;
    //mdepth_last = mdepth_cur;
   // mcolor_cur = cv_ptrRGB->image;
    //mdepth_cur = cv_ptrD->image;
    mcolor_cur.copyTo(mcolor_last);
    mdepth_cur.copyTo(mdepth_last);
    cv_ptrRGB->image.copyTo(mcolor_cur);
    cv_ptrD->image.copyTo(mdepth_cur);
    // Important: mcolor_last不可为空，否则在RGBD-DSLAM 构造Frame出错
    if(mcolor_last.empty() || mdepth_last.empty())
    {
      std::cout<<"At the first time the last image is empty!!"<<std::endl;
      mcolor_cur.copyTo(mcolor_last);
      mdepth_cur.copyTo(mdepth_last);
    }
    
    cv::Mat Tcw = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
    if(bpublishtf)
    {
      PublishPose(Tcw);
    }
    
}
void ImageGrabber::PublishPose(cv::Mat Tcw)
{
    
    if(!Tcw.empty())
    {
        btracked = true;
	mlostcounts = 0;
        //[1] 计算T_camera0_camera : [Rwc | twc]
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);
        vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);
        tf::Transform T_camera0_camera(tf::Quaternion(q[0], q[1], q[2],q[3]),tf::Vector3(twc.at<float>(0),twc.at<float>(1),twc.at<float>(2)) );

        //[2] 订阅T_base-footprint_camera-rgb-optical
       
         if(bfirst_listent)
        {
          
          mtf.waitForTransform("base_footprint", "camera_rgb_optical_frame", ros::Time(), ros::Duration(1.0));
          mtf.lookupTransform("base_footprint", "camera_rgb_optical_frame",ros::Time(), T_basefoot_camera);
          std::cout<<" get the tf_basefoot_camera"<<std::endl;
          bfirst_listent =false;
          
        }
        //[3] 计算T_map_base-footprint, T_map_camera0 = T_base-footprint_camera-rgb-optical
        //T_map_base-footprint =T_map_camera0 * T_camera0_camera * tf_basefoot_camera.inverse()

        tf::Transform T_map_basefoot = T_basefoot_camera * T_camera0_camera *T_basefoot_camera.inverse();

        //[4] 只取T_map_base-footprint的2D信息
        double yaw,pitch,roll;
        T_map_basefoot.getBasis().getRPY(roll,pitch,yaw);//弧度
        //std::cout<<roll*180.0/M_PI<<" "<<pitch*180.0/M_PI<<" "<<yaw*180.0/M_PI<<std::endl;

        tf::Quaternion q_map_basefoot;
        q_map_basefoot.setRPY(0,0,yaw);
        
        tf::Vector3 v = T_map_basefoot.getOrigin();
	
        //保存mT_map_last
	mT_map_last.setOrigin( tf::Vector3(v.getX(),v.getY(),0.0) );
	mT_map_last.setRotation(q_map_basefoot);
	
        /*
        cv::Mat Rmw=(cv::Mat_<float>(3,3) << 0, 0, 1, -1, 0, 0, 0, -1, 0);// 从格栅地图的map 到 orbslam的world
        //Rwc = Rmw*Rwc;
        twc = Rmw*twc;
        
        
      
        tf::Transform tfwc(tf::Quaternion(q[0], q[1], q[2],q[3]));
        double yaw,pitch,roll;
        tfwc.getBasis().getRPY(roll,pitch,yaw);//弧度
        std::cout<<roll*180.0/M_PI<<" "<<pitch*180.0/M_PI<<" "<<yaw*180.0/M_PI<<std::endl;
        tf::Quaternion qwc;
        qwc.setRPY(0,0,-pitch);
        //std::cout<<"转动角度："<<-yaw*180.0/M_PI<<std::endl;
        tf::Transform tf_map_basefoot = tf::Transform(qwc, tf::Vector3(twc.at<float>(0),twc.at<float>(1), 0.0));// T_map_base
        */
        /*tf::TransformListener mtf;// 订阅tf
        if(bfirst_listent)
        {
          
          mtf.waitForTransform("odom", "base_footprint", ros::Time(), ros::Duration(1.0));
          bfirst_listent =false;
          std::cout<<" wait for firstly"<<std::endl;
        }

        tf::StampedTransform tf_odom_basefoot;
        mtf.lookupTransform("odom", "base_footprint", ros::Time(), tf_odom_basefoot);//获得T_odom_base
        //terminate called after throwing an instance of 'tf2::LookupException'
        //what():  "odom" passed to lookupTransform argument target_frame does not exist. 

        tf::Transform tf_map_odom = tf_map_basefoot* tf_odom_basefoot.inverse();//T_map_odom = T_map_base * T_odom_base.inverse()*/
        
        pbroadcaster->sendTransform(tf::StampedTransform(tf::Transform(tf::createIdentityQuaternion(),
                                           tf::Vector3(0,0,0)),
                                           ros::Time::now(),
                                           "map","odom")); //T_world_odom

        pbroadcaster->sendTransform(tf::StampedTransform(tf::Transform(q_map_basefoot,
                                    tf::Vector3(v.getX(),v.getY(),0.0) ),
                                    ros::Time::now(),
                                    "odom","base_footprint"));//T_world_base
        mcurrenttime =ros::Time::now().toSec();
        double costtime = mcurrenttime -mlasttime;
        mlasttime = mcurrenttime;
        //std::cout<<"cost: "<< costtime<<" seconds"<<std::endl;
        //(pPosPub)->publish(poseMSG);

        //mlbLost.push_back(mState==LOST);
    }
    else // lost: 利用odom里程计信息发布tf
    {
      btracked = false;
      mlostcounts++;
      if(mlostcounts <= maxlostcounts)
      {
	  if(buseodom)//当连续跟丢帧数小于minlostcounts时，用里程计信息发布tf
	{
	  //计算里程计增量 T_last_cur
	  tf::Transform T_map_last;
	  CreateTfFromXYtheta(mLastOdom.pose.pose.position.x,
	                  mLastOdom.pose.pose.position.y,
			    tf::getYaw(mLastOdom.pose.pose.orientation), 
			  T_map_last);
      
	  tf::Transform T_map_cur; 
	  CreateTfFromXYtheta(mCurrentOdom.pose.pose.position.x,
	                  mCurrentOdom.pose.pose.position.y,
			  tf::getYaw(mCurrentOdom.pose.pose.orientation),
			  T_map_cur);
      
	  tf::Transform T_last_cur = T_map_last.inverse()*T_map_cur;
     
	  // 计算T_map_current = T_map_last * T_last_cur
	  tf::Transform T_map_base = mT_map_last *T_last_cur;
      
	  //更新mT_map_last
	  mT_map_last = T_map_base;//
	  //发布tf
	  pbroadcaster->sendTransform(tf::StampedTransform(tf::Transform(tf::createIdentityQuaternion(),
                                           tf::Vector3(0,0,0)),
                                           ros::Time::now(),
                                           "map","odom")); //T_world_odom

	  pbroadcaster->sendTransform(tf::StampedTransform(T_map_base,
                                    ros::Time::now(),
                                    "odom","base_footprint"));//T_world_base
	  std::cout<<"Using Odom to navigation"<<std::endl;
	}
	else //direct method 估计里程--效果较差2017-6-6
	{
	  if(mlostcounts == 1) //第一次跟丢
	  {
	    //设置参考帧
	   
	     if(mcolor_last.channels()==3)
              {
                cvtColor(mcolor_last,mcolor_last,CV_RGB2GRAY);   
              }
	    mframe_ref.reset(new RGBD_DSLAM::Frame(mcam, mcolor_last,mdepth_last,mc.scale ,0.0));
	    Quaterniond q(1,0,0,0);
            Vector3d t(0,0,0);
	    mframe_ref->T_f_w_ = Sophus::SE3(q,t);
	    // 提取FAST特征，并生成3D点
	    mfeature_detector->detect(mframe_ref.get(), mframe_ref->img_pyr_, RGBD_DSLAM::Config::triangMinCornerScore(), mframe_ref->fts_);
            printf("Added %zu 3d pts to the reference frame.\n", mframe_ref->nObs());
	  }
	  
	   if(mcolor_cur.channels()==3)
            {
                cvtColor(mcolor_cur,mcolor_cur,CV_RGB2GRAY);   
            }
	  mframe_cur.reset(new RGBD_DSLAM::Frame(mcam, mcolor_cur, mdepth_cur,mc.scale,0.0));
          mframe_cur->T_f_w_ = mframe_ref->T_f_w_; // start at reference frame
          //frame_cur->T_f_w_ = T_prev_w; // start at last frame
	  Sophus::SE3 T_cur_ref;
	  RGBD_DSLAM::SparseImgAlign img_align(RGBD_DSLAM::Config::kltMaxLevel(), RGBD_DSLAM::Config::kltMinLevel(),
                                         30, RGBD_DSLAM::SparseImgAlign::GaussNewton, false, false);
	  img_align.run(mframe_ref, mframe_cur,T_cur_ref);
          mframe_cur->T_f_w_ = T_cur_ref * mframe_ref->T_f_w_;
	  
	  //更新 mframe_ref 和 mframe_cur
	  mfeature_detector->detect(mframe_cur.get(), mframe_cur->img_pyr_, RGBD_DSLAM::Config::triangMinCornerScore(), mframe_cur->fts_);
	  mframe_ref = mframe_cur;
	  std::cout<<"Using direct method to navigation ..."<<std::endl;
	  
	  //获得两图的匹配情况，并发布图
	  cv::Mat img_show = RGBD_DSLAM::test_utils::match_points(mframe_ref,mframe_cur,T_cur_ref,"vo result");
	 
	  sensor_msgs::ImagePtr msgImage;
	  msgImage = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_show).toImageMsg();
	  mimage_pub.publish(*msgImage);
	  
	  // 利用 相邻两帧间的运动T_cur_ref ，发布tf
	  //类型转换： 从Sophus::SE3到Eigen,然后到tf::Transform
	  Quaterniond q = T_cur_ref.inverse().unit_quaternion();//q_ref_cur
	  Vector3d t = T_cur_ref.inverse().translation();//t_ref_cur
	  
	  tf::Transform T_last_cur( tf::Quaternion(q.x(), q.y(), q.z(),q.w()),tf::Vector3( t(0),t(1),t(2)) );
	  double yaw,pitch,roll;
          T_last_cur.getBasis().getRPY(roll,pitch,yaw);//弧度
          CreateTfFromXYtheta(t(0),t(1),yaw,T_last_cur); //只取2d方向
	  
	  // 计算T_map_current = T_map_lastbsase * T_lastbase_cameralast * T_cameralast_cameracur *T_cameracur_basecur
	  tf::Transform T_map_base = mT_map_last *T_basefoot_camera*T_last_cur*T_basefoot_camera.inverse();
      
	  //更新mT_map_last
	  mT_map_last = T_map_base;//
	  //发布tf
	  pbroadcaster->sendTransform(tf::StampedTransform(tf::Transform(tf::createIdentityQuaternion(),
                                           tf::Vector3(0,0,0)),
                                           ros::Time::now(),
                                           "map","odom")); //T_world_odom

	  pbroadcaster->sendTransform(tf::StampedTransform(T_map_base,
                                    ros::Time::now(),
                                    "odom","base_footprint"));//T_world_base
	}
      }
      
      
    }
}
void ImageGrabber::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    mOdom = *odom_msg;
}
void ImageGrabber::CreateTfFromXYtheta(double x, double y, double theta, tf::Transform& t)
{
    t.setOrigin(tf::Vector3(x, y, 0.0));
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, theta);
    t.setRotation(q);

}

void ImageGrabber::RotateRecovery()
{
  ros::Rate r(20);
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  double theta = 0.0;
  while(mnh.ok())
  {
    //if(mlostcounts>25 && (theta < 2*M_PI))
    if(mlostcounts> maxlostcounts )
    {
      current_time = ros::Time::now();
    
      geometry_msgs::Twist cmd_vel;
      double vth = 0.25;
      cmd_vel.linear.x = 0.0;
      cmd_vel.linear.y = 0.0;
      cmd_vel.angular.z = vth;
      mvel_pub.publish(cmd_vel);
      theta = vth * (current_time.toSec() - last_time.toSec());
      last_time = current_time;
      std::cout<<" Rotete Recovery ....."<<std::endl;
      if(theta > 2*M_PI)
      {
	mlostcounts = 0; //退出原地旋转行为，继续用里程计发布tf
      }
    }
    else
    {
      current_time = ros::Time::now();
      last_time = ros::Time::now();
    }
    
    r.sleep();
  }
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::NodeHandle nh;
    
    ros::start();
    bool bReuseMap = false;
    bool buse_odom = true;
    if(argc != 6)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings if_reuse_map if_publish_tf if_use_odom" << endl;        
        ros::shutdown();
        return 1;
    }    
    
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    if (!strcmp(argv[3], "true"))
    {
		bReuseMap = true;
    }

     if (!strcmp(argv[5], "false"))
    {
		buse_odom = false;
    }
    
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true,bReuseMap);// LoadMap("Slam_Map.bin");

    ImageGrabber igb(&SLAM,nh,argv[2],buse_odom);

     if (!strcmp(argv[4], "true"))
    {
		igb.bpublishtf = true;
    }
    
 
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/kinect2/qhd/image_color", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/kinect2/qhd/image_depth_rect", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    //ros::Publisher PosPub = nh.advertise<geometry_msgs::PoseStamped>("ORB_SLAM/pose", 5);//发布topic 和 信息的队列长度
    //igb.pPosPub = &(PosPub);
    tf::TransformBroadcaster broadcaster;
    igb.pbroadcaster = &(broadcaster);
    igb.mlasttime =ros::Time::now().toSec();
    
    /**********非阻塞键盘输入**********
     int keys_fd;
     struct input_event input;
     keys_fd=open("/dev/input/event3",O_RDONLY|O_NONBLOCK );//定义为非阻塞
     if(keys_fd<0)
      {
              printf("error\n");
              return -1;
       }
    **********************/
    //ros::spin();
    //while(ros::ok() && !(input.code == KEY_ESC))
    while(ros::ok())
    {
  
       ros::spinOnce();//call back once
    }
    // Stop all threads
    SLAM.Shutdown();

    // Save map
    SLAM.SaveMap("Slam_latest_Map.bin");

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}




