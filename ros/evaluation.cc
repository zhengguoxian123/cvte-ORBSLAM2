#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <stdlib.h>
#include <string>
#include <stdio.h>

#include <linux/input.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>
#include "Converter.h"
#include"System.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM),bpublishtf(false),bfirst_listent(true){}
     void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);
    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD, const sensor_msgs::LaserScanConstPtr& msgScan);
    void PublishPose(cv::Mat Tcw);
    void PublishScan();
    
    ORB_SLAM2::System* mpSLAM;
    ros::Publisher* pPosPub;//发布位姿态
    ros::Publisher* pScanPub;//发布激光数据
    tf::TransformBroadcaster* pbroadcaster;
    bool bpublishtf;
    bool bfirst_listent;
    tf::StampedTransform mT_basefoot_camera;
    tf::Transform mT_map_basefoot;
    tf::TransformListener mtf;// 订阅tf
    double mlasttime, mcurrenttime;
};
void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
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

    cv::Mat Tcw = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
    if(bpublishtf)
    {
      PublishPose(Tcw);
    }
    
}
void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD, const sensor_msgs::LaserScanConstPtr& msgScan)
{
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

    //cv::Mat Tcw = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
    cv::Mat Tcw = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec(),msgScan);
    if(bpublishtf)
    {
      PublishPose(Tcw);
    }
     PublishScan();
     mpSLAM->PublishMap();
    
}

void ImageGrabber::PublishPose(cv::Mat Tcw)
{
    
    if(!Tcw.empty())
    {
        //[1] 计算T_camera0_camera : [Rwc | twc]
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);
        vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);
        tf::Transform T_camera0_camera(tf::Quaternion(q[0], q[1], q[2],q[3]),tf::Vector3(twc.at<float>(0),twc.at<float>(1),twc.at<float>(2)) );

        //[2] 订阅T_base-footprint_camera-rgb-optical
       
         if(bfirst_listent)
        {
          
          mtf.waitForTransform("base_footprint", "camera_rgb_optical_frame", ros::Time(), ros::Duration(1.0));
          mtf.lookupTransform("base_footprint", "camera_rgb_optical_frame",ros::Time(), mT_basefoot_camera);
          std::cout<<" get the tf_basefoot_camera"<<std::endl;
          bfirst_listent =false;
          
        }
        //[3] 计算T_map_base-footprint, T_map_camera0 = T_base-footprint_camera-rgb-optical
        //T_map_base-footprint =T_map_camera0 * T_camera0_camera * tf_basefoot_camera.inverse()

        mT_map_basefoot = mT_basefoot_camera * T_camera0_camera *mT_basefoot_camera.inverse();

        //[4] 只取T_map_base-footprint的2D信息
        double yaw,pitch,roll;
        mT_map_basefoot.getBasis().getRPY(roll,pitch,yaw);//弧度
       // std::cout<<roll*180.0/M_PI<<" "<<pitch*180.0/M_PI<<" "<<yaw*180.0/M_PI<<std::endl;

        tf::Quaternion q_map_basefoot;
        q_map_basefoot.setRPY(0,0,yaw);
        
        tf::Vector3 v = mT_map_basefoot.getOrigin();
        //std::cout<<"v.z :"<< v.getZ()<<std::endl;
        pbroadcaster->sendTransform(tf::StampedTransform(tf::Transform(tf::createIdentityQuaternion(),
                                           tf::Vector3(0,0,0)),
                                           ros::Time::now(),
                                           "map","odom")); //T_world_odom

        pbroadcaster->sendTransform(tf::StampedTransform(tf::Transform(q_map_basefoot,
                                    tf::Vector3(v.getX(),v.getY(),v.getZ()) ),
                                    ros::Time::now(),
                                    "odom","base_footprint"));//T_world_base
        mcurrenttime =ros::Time::now().toSec();
        double costtime = mcurrenttime -mlasttime;
        mlasttime = mcurrenttime;
        //std::cout<<"cost: "<< costtime<<" seconds"<<std::endl;
        //(pPosPub)->publish(poseMSG);

        //mlbLost.push_back(mState==LOST);
    }
}

void ImageGrabber::PublishScan()
{
   pScanPub->publish(mpSLAM->GetLastKeyframeScan());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();
    bool bReuseMap = false;
    if(argc != 5)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings if_reuse_map if_publish_tf" << endl;        
        ros::shutdown();
        return 1;
    }    
    
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    if (!strcmp(argv[3], "true"))
    {
		bReuseMap = true;
    }
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true,bReuseMap);// LoadMap("Slam_Map.bin");

    ImageGrabber igb(&SLAM);

     if (!strcmp(argv[4], "true"))
    {
		igb.bpublishtf = true;
    }
    ros::NodeHandle nh;
 
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/kinect2/qhd/image_color", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/kinect2/qhd/image_depth_rect", 1);
   // message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub(nh, "/scan", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    ros::Publisher ScanPub = nh.advertise<sensor_msgs::LaserScan>("orbslam2/scan", 5);//发布topic 和 信息的队列长度
    igb.pScanPub = &(ScanPub);
    tf::TransformBroadcaster broadcaster;
    igb.pbroadcaster = &(broadcaster);
    igb.mlasttime =ros::Time::now().toSec();
    
    /**********非阻塞键盘输入**********/
     int keys_fd;
     struct input_event input;
     keys_fd=open("/dev/input/event3",O_RDONLY|O_NONBLOCK );//定义为非阻塞
     if(keys_fd<0)
      {
              printf("error\n");
              return -1;
       }
    /**********************/
    char Savecampose[256];
    int measure_point_th = 1; //测量点序号
    int PerPoint  = 5;        //每个点测量总次数
    int i = 0;                //统计每个点测量次数
    //while(ros::ok() && !(input.code == KEY_ESC))
    while(ros::ok() )  
    {
      //TODO
      /*
       read(keys_fd,&input,sizeof(struct input_event));
       if(input.code == KEY_ENTER)
       {
	 i++;
	 if(i == PerPoint)
	 {
	   measure_point_th++;
	   std::cout<< " please measure next point!!!"<<std::endl;
	}
	 //保存测量点数据
	 sprintf(Savecampose,"/home/bobo/code/orbslam2_3D/data/measure_point/%01d.txt",measure_point_th);
	 ofstream fout(Savecampose);
	 tf::Quaternion q_map_basefoot;
	 q_map_basefoot = igb.mT_map_basefoot.getRotation();
	 double yaw,pitch,roll;
         igb.mT_map_basefoot.getBasis().getRPY(roll,pitch,yaw);//弧度
	 tf::Vector3 v = igb.mT_map_basefoot.getOrigin();
	 //fout <<measure_point_th << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
         // << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
	 
	 std::cout<<" The key points nunber of current frame is :"<< SLAM.GetCFKeypointNum()<<std::endl;
      }
      */	 
       ros::spinOnce();//call back once
    }
    // Stop all threads
    SLAM.Shutdown();

    // Save map
    //SLAM.SaveMap("Slam_latest_Map.bin");

    // Save camera trajectory
    //SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}




