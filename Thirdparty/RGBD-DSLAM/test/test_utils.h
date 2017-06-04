#ifndef TEST_UTILS_H_
#define TEST_UTILS_H_
//STL
#include <string.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <functional>

#include <cstdlib> // for getenv
#include <opencv2/opencv.hpp>

#include "frame.h"
#include "point.h"
#include "feature.h"
using namespace std;
using namespace Eigen;

namespace RGBD_DSLAM {
 //索引排序 
  template <typename T>
vector<size_t> sort_indexes(const vector<T> &v) 
{
  // initialize original index locations
  vector<size_t> idx(v.size());
  iota(idx.begin(), idx.end(), 0);

  // sort indexes based on comparing values in v
  sort(idx.begin(), idx.end(),
       [&v](size_t i1, size_t i2) {return v[i1] < v[i2];});//升序

  return idx;
}
  
namespace test_utils {
  bool plot_points(RGBD_DSLAM::FramePtr frame_ref, RGBD_DSLAM::FramePtr frame_cur, Sophus::SE3 & T_cur_ref, const string & windowname)
   {
              cv::Mat img_show(frame_ref->img_pyr_[0].rows*2,frame_ref->img_pyr_[0].cols,CV_8UC3);
              cv::Mat img_ref = frame_ref->img_pyr_[0];
              cv::Mat img_cur = frame_cur->img_pyr_[0];
              cv::Mat img_ref_rgb(img_ref.size(),CV_8UC3);
              cv::Mat img_cur_rgb(img_ref.size(),CV_8UC3);
              cv::cvtColor(img_ref, img_ref_rgb, CV_GRAY2RGB);
              cv::cvtColor(img_cur, img_cur_rgb, CV_GRAY2RGB);
              img_ref_rgb.copyTo(img_show(cv::Rect(0,0,img_ref.cols,img_ref.rows)));
              img_cur_rgb.copyTo(img_show(cv::Rect(0,img_ref.rows,img_ref.cols,img_ref.rows)));

              for(auto it=frame_ref->fts_.begin(),  ite=frame_ref->fts_.end(); it!=ite; ++it)
              {
                  //if ( rand() > RAND_MAX/5 )
                       //continue;
                 if((*it)->f3d ==Vector3d(0.0,0.0,0.0))
                      continue;
                //Vector3d p =frame_ref->T_f_w_.inverse()*(*it)->f3d;
                 Vector2d pixel_pre = (*it)->px;

                Vector3d p2 =T_cur_ref*(*it)->f3d;
               // Vector3d p2 =Tcw.rotation()*p + Tcw.translation();
                Vector2d pixel_current = frame_cur->cam_->world2cam(p2);
                 if(pixel_current[0] <0||pixel_current[0]>img_cur.cols ||pixel_current[1]<0||pixel_current[1] > img_cur.rows)
                   continue;
                 int b = rand() % 255;
                 int g = rand() % 255;
                 int r = rand() % 255;
                 cv::circle(img_show,cv::Point2f(pixel_pre[0],pixel_pre[1]), 8, cv::Scalar(b,g,r), 2);
                 cv::circle(img_show,cv::Point2f(pixel_current[0],pixel_current[1]+img_cur.rows), 8, cv::Scalar(b,g,r), 2);
                 cv::line(img_show,cv::Point2f(pixel_pre[0],pixel_pre[1]),cv::Point2f(pixel_current[0],pixel_current[1]+img_cur.rows),cv::Scalar(b,g,r),1);
              }
              cv::imshow(windowname, img_show);
              int input_key=cv::waitKey(10);
              if( input_key== 27)
                return false;
                else
	      return true;
  } 
  bool plot_points(RGBD_DSLAM::FramePtr frame_ref, RGBD_DSLAM::FramePtr frame_cur,const string & windowname)
   {
             Sophus::SE3  T_cur_ref = frame_cur->T_f_w_*frame_ref->T_f_w_.inverse();
	     return plot_points(frame_ref,frame_cur,T_cur_ref,windowname);
  }
} // namespace test_utils
} // namespace RGBD_DSLAM


#endif // TEST_UTILS_H_
