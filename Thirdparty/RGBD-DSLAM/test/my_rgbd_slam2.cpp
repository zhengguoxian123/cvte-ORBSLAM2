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

namespace {
/// SparseImgAlign Test-Fixture
class SparseImgAlignTest {
 public:

  SparseImgAlignTest();
  virtual ~SparseImgAlignTest();
  void testSequence(
      const std::string& dataset_dir,
      const std::string& experiment_name,
      svo::feature_detection::AbstractDetector* feature_detector);

  vk::PinholeCamera* cam_;
  svo::FramePtr frame_ref_;
  svo::FramePtr frame_cur_;
};

SparseImgAlignTest::SparseImgAlignTest() :
    cam_(new vk::PinholeCamera(752, 480, 315.5, 315.5, 376.0, 240.0))
{}

SparseImgAlignTest::~SparseImgAlignTest()
{}

void SparseImgAlignTest::testSequence(
    const std::string& dataset_dir,
    const std::string& experiment_name,
    svo::feature_detection::AbstractDetector* feature_detector)
{
  vk::FileReader<vk::blender_utils::file_format::ImageNameAndPose> sequence_file_reader(dataset_dir+"/trajectory.txt");
  std::vector<vk::blender_utils::file_format::ImageNameAndPose> sequence;
  sequence_file_reader.skipComments();
  if(!sequence_file_reader.next())
    std::runtime_error("Failed to open sequence file");
  sequence_file_reader.readAllEntries(sequence);
  printf("RUN EXPERIMENT: read %zu dataset entries.\n", sequence.size());
  std::vector<vk::blender_utils::file_format::ImageNameAndPose>::iterator iter = sequence.begin();
  std::list<double> translation_error;

  Sophus::SE3 T_prev_w, T_prevgt_w;
  std::string trace_dir(svo::test_utils::getTraceDir());
  std::string trace_name(trace_dir + "/sparse_img_align_" + experiment_name + "_trans_estimate.txt");
  std::ofstream ofs(trace_name.c_str());
  for(int i=0; iter != sequence.end() && i<180; ++iter, ++i)
  {
    // load img
    // IMPORTANT: We need to flip the image because the Povray dataset has a
    // negative focal length in y direction which we didn't set.
    std::string img_name(dataset_dir+"/img/" + iter->image_name_ + "_0.png");
    cv::Mat img(cv::imread(img_name, 0));
    assert(!img.empty());

    // load pose
    Sophus::SE3 T_w_gt(iter->q_, iter->t_);
    Sophus::SE3 T_gt_w = T_w_gt.inverse(); // ground-truth

    if(i==0)
    {
      // set reference frame
      frame_ref_.reset(new svo::Frame(cam_, img, 0.0));
      frame_ref_->T_f_w_ = T_gt_w;

      // load ground-truth depth
      cv::Mat depthmap;
      vk::blender_utils::loadBlenderDepthmap(
          dataset_dir+"/depth/"+iter->image_name_+"_0.depth", *cam_, depthmap);//unit: m

      // extract features, generate features with 3D points
      feature_detector->detect(
          frame_ref_.get(), frame_ref_->img_pyr_, svo::Config::triangMinCornerScore(), frame_ref_->fts_);
      std::for_each(frame_ref_->fts_.begin(), frame_ref_->fts_.end(), [&](svo::Feature* i) {
        Eigen::Vector3d pt_pos_cur = i->f*depthmap.at<float>(i->px[1], i->px[0]);
        Eigen::Vector3d pt_pos_w = frame_ref_->T_f_w_.inverse()*pt_pos_cur;
        svo::Point* pt = new svo::Point(pt_pos_w, i);
        i->point = pt;
      });

      printf("Added %zu 3d pts to the reference frame.\n", frame_ref_->nObs());
      T_prev_w = frame_ref_->T_f_w_;
      T_prevgt_w = T_gt_w;
      continue;
    }

    frame_cur_.reset(new svo::Frame(cam_, img, 0.0));
    //frame_cur_->T_f_w_ = frame_ref_->T_f_w_; // start at reference frame
    frame_cur_->T_f_w_ = T_prev_w; // start at last frame

    // run image align
    vk::Timer t;
    svo::SparseImgAlign img_align(svo::Config::kltMaxLevel(), svo::Config::kltMinLevel(),
                                    30, svo::SparseImgAlign::GaussNewton, false, false);
    img_align.run(frame_ref_, frame_cur_);

    // compute error
    Sophus::SE3 T_f_gt = frame_cur_->T_f_w_ * T_gt_w.inverse();
    translation_error.push_back(T_f_gt.translation().norm());
    printf("[%3.i] time = %f ms \t |t| = %f \t translation error = %f\n",
           i, t.stop()*1000, (frame_ref_->T_f_w_*T_gt_w.inverse()).translation().norm(),
           translation_error.back());

    // save old pose for next iteration
    T_prev_w = frame_cur_->T_f_w_;
    T_prevgt_w = T_gt_w;

    ofs << frame_cur_->T_f_w_.inverse().translation().transpose() << " "
        << T_gt_w.inverse().translation().transpose() << std::endl;

    //plot the feature points
         cv::Mat img_show(img.rows*2,img.cols,CV_8UC3);
         cv::Mat img_ref = frame_ref_->img_pyr_[0];
         cv::Mat img_cur = frame_cur_->img_pyr_[0];
         cv::Mat img_ref_rgb(img_ref.size(),CV_8UC3);
         cv::Mat img_cur_rgb(img_ref.size(),CV_8UC3);
         cv::cvtColor(img_ref, img_ref_rgb, CV_GRAY2RGB);
         cv::cvtColor(img_cur, img_cur_rgb, CV_GRAY2RGB);
         img_ref_rgb.copyTo(img_show(cv::Rect(0,0,img.cols,img.rows)));
         img_cur_rgb.copyTo(img_show(cv::Rect(0,img.rows,img.cols,img.rows)));

         for(auto it=frame_ref_->fts_.begin(),  ite=frame_ref_->fts_.end(); it!=ite; ++it)
         {
             //if ( rand() > RAND_MAX/5 )
                  //continue;
           Eigen:: Vector3d p =(*it)->point->pos_;
           Eigen::Vector2d pixel_pre = (*it)->px;

          Eigen::Vector3d p2 =frame_cur_->T_f_w_ *p;
          // Vector3d p2 =Tcw.rotation()*p + Tcw.translation();
           Eigen::Vector2d pixel_current = cam_->world2cam(p2);
            if(pixel_current[0] <0||pixel_current[0]>img.cols ||pixel_current[1]<0||pixel_current[1] >img.rows)
              continue;
            int b = rand() % 255;
            int g = rand() % 255;
            int r = rand() % 255;
            cv::circle(img_show,cv::Point2f(pixel_pre[0],pixel_pre[1]), 8, cv::Scalar(b,g,r), 2);
            cv::circle(img_show,cv::Point2f(pixel_current[0],pixel_current[1]+img.rows), 8, cv::Scalar(b,g,r), 2);
            cv::line(img_show,cv::Point2f(pixel_pre[0],pixel_pre[1]),cv::Point2f(pixel_current[0],pixel_current[1]+img.rows),cv::Scalar(b,g,r),1);
         }
         cv::imshow("result", img_show);
         cv::waitKey();
         // end plot the feature points

    //update reference frame; how to update it ??
    frame_ref_ = frame_cur_;
    //frame_ref_->T_f_w_ = T_gt_w;

    // load ground-truth depth; *****cost much time*****
    cv::Mat depthmap;
    vk::blender_utils::loadBlenderDepthmap(
        dataset_dir+"/depth/"+iter->image_name_+"_0.depth", *cam_, depthmap);

    // extract features, generate features with 3D points
    //vk::Timer t1;
    feature_detector->detect(
        frame_ref_.get(), frame_ref_->img_pyr_, svo::Config::triangMinCornerScore(), frame_ref_->fts_);
   // printf("Fast corner detection took %f ms, %zu corners detected (ref i7-W520: 7.166360ms, 40000)\n", t1.stop()*1000, frame_ref_->fts_.size());

    std::for_each(frame_ref_->fts_.begin(), frame_ref_->fts_.end(), [&](svo::Feature* i)
    {
      Eigen::Vector3d  pt_pos_cur = i->f*depthmap.at<float>(i->px[1], i->px[0]);
      Eigen::Vector3d  pt_pos_w = frame_ref_->T_f_w_.inverse()*pt_pos_cur;
      svo::Point* pt1 = new svo::Point(pt_pos_w, i);
      i->point = pt1;
    });
    // update frame_ref_  end
  }
  ofs.close();

  // trace error
  trace_name = trace_dir + "/sparse_img_align_" + experiment_name + "_trans_error.txt";
  ofs.open(trace_name.c_str());
  for(std::list<double>::iterator it=translation_error.begin(); it!=translation_error.end(); ++it)
    ofs << *it << std::endl;
  ofs.close();
}

}  // namespace


int main(int argc, char** argv)
{
  std::string experiment_name("flying_room_1_rig_1_fast_minlev0");
  std::string dataset_dir(svo::test_utils::getDatasetDir() + "/sin2_tex2_h1_v8_d");
  svo::Config::triangMinCornerScore() = 20;
  svo::Config::kltMinLevel() = 0;
  SparseImgAlignTest test;
  svo::feature_detection::FastDetector detector(
      test.cam_->width(), test.cam_->height(), svo::Config::gridSize(), svo::Config::nPyrLevels());
  test.testSequence(dataset_dir, experiment_name, &detector);
  return 0;
}
