#include <vikit/pinhole_camera.h>
#include <vikit/math_utils.h>
#include <vikit/file_reader.h>
#include <vikit/timer.h>
#include <vikit/blender_utils.h>
#include <iostream>
#include "feature_detection.h"
#include "sparse_img_align.h"
#include "frame.h"
#include "point.h"
#include "feature.h"
#include "config.h"
#include "test_utils.h"
#include <iomanip>
using namespace std;
using namespace Eigen;
// 相机内参结构
struct CAMERA_INTRINSIC_PARAMETERS
{
    float cx, cy, fx, fy, scale,width,height;
};
//(x,y), (cols,rows)
Eigen::Vector3d imageTo3d(int x, int y, ushort& d, const CAMERA_INTRINSIC_PARAMETERS &camera)
{
   float z_c = float(d)/camera.scale;
   float x_c = z_c*(x - camera.cx)/camera.fx;
   float y_c = z_c*(y - camera.cy)/camera.fy;
   return Eigen::Vector3d(x_c, y_c, z_c);
}

Eigen::Vector2d C_3dToimage(Vector3d point,const CAMERA_INTRINSIC_PARAMETERS &camera)
{
  float u = point[0]*camera.fx/point[2] + camera.cx;
  float v = point[1]*camera.fy/point[2] + camera.cy;
  return Vector2d(u,v);
}

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if(argc != 5)
    {
        cerr << endl << "Usage: ./sparse_direct  path_to_settings path_to_sequence path_to_association path_to_result" << endl;
        return 1;
    }
    // Retrieve paths to images
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    string strAssociationFilename = string(argv[3]);
    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

    // Check consistency in the number of images and depthmaps
    int nImages = vstrImageFilenamesRGB.size();
    if(vstrImageFilenamesRGB.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }

    vk::PinholeCamera* cam;
    RGBD_DSLAM::FramePtr frame_ref;
    RGBD_DSLAM::FramePtr frame_cur;
    RGBD_DSLAM::Config::triangMinCornerScore() = 20;
    RGBD_DSLAM::Config::kltMinLevel() = 0;

    string directSettingsFile = string(argv[1]);
    cv::FileStorage fsSettings(directSettingsFile, cv::FileStorage::READ);
           if(!fsSettings.isOpened())
           {
              cerr << "Failed to open settings file at: " <<directSettingsFile<< endl;
              exit(-1);
           }
    CAMERA_INTRINSIC_PARAMETERS camera;
    camera.cx = fsSettings["Camera.cx"];
    camera.cy = fsSettings["Camera.cy"];
    camera.fx = fsSettings["Camera.fx"];
    camera.fy = fsSettings["Camera.fy"];
    camera.width =fsSettings["Camera.width"];
    camera.height = fsSettings["Camera.height"];
    camera.scale = fsSettings["DepthMapFactor"];

    cam = new vk::PinholeCamera(camera.width, camera.height, camera.fx, camera.fy, camera.cx, camera.cy );
    RGBD_DSLAM::feature_detection::FastDetector feature_detector(cam->width(), cam->height(), RGBD_DSLAM::Config::gridSize(), RGBD_DSLAM::Config::nPyrLevels());

    //int image_num = 2;
    cv::Mat color_image, depth_image;
    Sophus::SE3 T_prev_w,T_cur_ref;
     //char color_path[256] , depth_path[256];
    string result_path = string(argv[4]);
    string Trajectory_name(result_path+"/cameraTrajectory.txt");
    std::ofstream ofs(Trajectory_name.c_str());
    int input_key;
     for(int index = 0; index < nImages ; index++)
    {
         /*sprintf(color_path,"/home/v5lab/bobo/code/rpg_svo/svo/Datasets/data/rgb_png/%01d.png",index);
         sprintf(depth_path,"/home/v5lab/bobo/code/rpg_svo/svo/Datasets/data/depth_png/%01d.png",index);
         color_image = cv::imread(color_path,0);
         depth_image= cv::imread(depth_path,0);*/
         color_image=cv::imread(string(argv[2])+"/"+vstrImageFilenamesRGB[index],0);
         depth_image = cv::imread(string(argv[2])+"/"+vstrImageFilenamesD[index],0);

         assert(color_image.type() == CV_8UC1 && !color_image.empty());

         if(index==0)
         {
           // set reference frame
           frame_ref.reset(new RGBD_DSLAM::Frame(cam, color_image,depth_image,camera.scale ,0.0));
           //Quaterniond q = Quaternion::Identity();Sophus::SE3(q,t).inverse()
           Quaterniond q(1,0,0,0);
           Vector3d t(0,0,0);
           frame_ref->T_f_w_ = Sophus::SE3(q,t).inverse();

           //save the camera trajectory
            Quaterniond q2 = frame_ref->T_f_w_.inverse().unit_quaternion();
           ofs << std::fixed<<setprecision(6) << vTimestamps[index]<<" "<<setprecision(9) << frame_ref->T_f_w_.inverse().translation().transpose() << " " << q2.x()<<" "<< q2.y()<<" "<<q2.z()<<" "<<q2.w()<< std::endl;

           // extract features, generate features with 3D points
           feature_detector.detect(
               frame_ref.get(), frame_ref->img_pyr_, RGBD_DSLAM::Config::triangMinCornerScore(), frame_ref->fts_);
           std::for_each(frame_ref->fts_.begin(), frame_ref->fts_.end(), [&](RGBD_DSLAM::Feature* i)
           {
             ushort d = depth_image.ptr<ushort>(int(i->px[1]))[int(i->px[0])];
             if( (0.4*camera.scale) < d && d < (5*camera.scale)) //so important
              {
             Eigen::Vector3d pt_pos_cur = imageTo3d(int(i->px[0]),int(i->px[1]),d,camera);
             //cout<<"i->f="<<i->f<<endl;
             Eigen::Vector3d pt_pos_w = frame_ref->T_f_w_.inverse()*pt_pos_cur;
             RGBD_DSLAM::Point* pt = new RGBD_DSLAM::Point(pt_pos_w, i);
             i->point = pt;
             }
           });

           printf("Added %zu 3d pts to the reference frame.\n", frame_ref->nObs());
           T_prev_w = frame_ref->T_f_w_;
           continue;
         }

         frame_cur.reset(new RGBD_DSLAM::Frame(cam, color_image, depth_image,camera.scale,0.0));
         //frame_cur_->T_f_w_ = frame_ref_->T_f_w_; // start at reference frame
         frame_cur->T_f_w_ = T_prev_w; // start at last frame

         // run image align
         vk::Timer t;

         RGBD_DSLAM::SparseImgAlign img_align(RGBD_DSLAM::Config::kltMaxLevel(), RGBD_DSLAM::Config::kltMinLevel(),
                                         30, RGBD_DSLAM::SparseImgAlign::GaussNewton, false, false);
         img_align.run(frame_ref, frame_cur,T_cur_ref);
	 frame_cur->T_f_w_ = T_cur_ref * frame_ref->T_f_w_;

         printf("[%3.i] time = %f ms\n",
                index, t.stop()*1000);

         //save the camera trajectory
         Quaterniond q2 = frame_cur->T_f_w_.inverse().unit_quaternion();
         ofs <<std::fixed<< setprecision(6)<<vTimestamps[index]<<" "<<setprecision(9) << frame_cur->T_f_w_.inverse().translation().transpose() << " "
             << q2.x()<<" "<< q2.y()<<" "<<q2.z()<<" "<<q2.w()<< std::endl;

         // save old pose for next iteration
         T_prev_w = frame_cur->T_f_w_;

         //plot the feature points
              cv::Mat img_show(color_image.rows*2,color_image.cols,CV_8UC3);
              cv::Mat img_ref = frame_ref->img_pyr_[0];
              cv::Mat img_cur = frame_cur->img_pyr_[0];
              cv::Mat img_ref_rgb(img_ref.size(),CV_8UC3);
              cv::Mat img_cur_rgb(img_ref.size(),CV_8UC3);
              cv::cvtColor(img_ref, img_ref_rgb, CV_GRAY2RGB);
              cv::cvtColor(img_cur, img_cur_rgb, CV_GRAY2RGB);
              img_ref_rgb.copyTo(img_show(cv::Rect(0,0,color_image.cols,color_image.rows)));
              img_cur_rgb.copyTo(img_show(cv::Rect(0,color_image.rows,color_image.cols,color_image.rows)));

              for(auto it=frame_ref->fts_.begin(),  ite=frame_ref->fts_.end(); it!=ite; ++it)
              {
                  //if ( rand() > RAND_MAX/5 )
                       //continue;
                 if((*it)->point ==NULL)
                      continue;
                 Vector3d p =(*it)->point->pos_;
                 Vector2d pixel_pre = (*it)->px;

                Vector3d p2 =frame_cur->T_f_w_ *p;
               // Vector3d p2 =Tcw.rotation()*p + Tcw.translation();
                Vector2d pixel_current = C_3dToimage(p2,camera);
                 if(pixel_current[0] <0||pixel_current[0]>color_image.cols ||pixel_current[1]<0||pixel_current[1] > color_image.rows)
                   continue;
                 int b = rand() % 255;
                 int g = rand() % 255;
                 int r = rand() % 255;
                 cv::circle(img_show,cv::Point2f(pixel_pre[0],pixel_pre[1]), 8, cv::Scalar(b,g,r), 2);
                 cv::circle(img_show,cv::Point2f(pixel_current[0],pixel_current[1]+color_image.rows), 8, cv::Scalar(b,g,r), 2);
                 cv::line(img_show,cv::Point2f(pixel_pre[0],pixel_pre[1]),cv::Point2f(pixel_current[0],pixel_current[1]+color_image.rows),cv::Scalar(b,g,r),1);
              }
              cv::imshow("result", img_show);
              input_key=cv::waitKey();
              if(input_key == 27)
                break;
              // end plot the feature points

              //update reference frame; how to update it ??
              frame_ref= frame_cur;
              //frame_ref_->T_f_w_ = T_gt_w;

              // extract features, generate features with 3D points
              //vk::Timer t1;
              feature_detector.detect(
                  frame_ref.get(), frame_ref->img_pyr_, RGBD_DSLAM::Config::triangMinCornerScore(), frame_ref->fts_);
             // printf("Fast corner detection took %f ms, %zu corners detected (ref i7-W520: 7.166360ms, 40000)\n", t1.stop()*1000, frame_ref_->fts_.size());

              std::for_each(frame_ref->fts_.begin(), frame_ref->fts_.end(), [&](RGBD_DSLAM::Feature* i)
              {
                  ushort d = depth_image.ptr<ushort>(int(i->px[1]))[int(i->px[0])];
                  if((0.4*camera.scale) < d && d < (5*camera.scale))
                  {
                  Eigen::Vector3d pt_pos_cur = imageTo3d(int(i->px[0]),int(i->px[1]),d,camera);
                  Eigen::Vector3d  pt_pos_w = frame_ref->T_f_w_.inverse()*pt_pos_cur;
                  RGBD_DSLAM::Point* pt1 = new RGBD_DSLAM::Point(pt_pos_w, i);
                  i->point = pt1;
                  }
              });
              // update frame_ref_  end
     }
     ofs.close();
     return 0;
}
void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);

        }
    }
}

