#include <iostream>
#include <iomanip>
#include <boost/concept_check.hpp>
//vikit
#include <vikit/pinhole_camera.h>
#include <vikit/math_utils.h>
#include <vikit/file_reader.h>
#include <vikit/timer.h>
#include <vikit/blender_utils.h>
//RGBD_DSLAM
#include "feature_detection.h"
#include "sparse_img_align.h"
#include "frame.h"
#include "point.h"
#include "feature.h"
#include "config.h"
#include "test_utils.h"
//g2o
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm_levenberg.h>


using namespace std;
using namespace Eigen;
// 相机内参结构
struct CAMERA_INTRINSIC_PARAMETERS
{
    float cx, cy, fx, fy, k1,k2,p1,p2,k3,scale,width,height;
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
//相关数据读取
//读取图片文件
void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);
//读取groundtruth
void LoadGroudtruth(const string& GroudtruthFilename, vector<Sophus::SE3>& gt);
//关键帧检测
bool isKeyFrame(RGBD_DSLAM::FramePtr frame1, RGBD_DSLAM::FramePtr frame2,const double& fai_max,const double &fai_min);
//给定两关键帧，增加位姿图的边
void add_edge(RGBD_DSLAM::FramePtr frame1, RGBD_DSLAM::FramePtr frame2,Sophus::SE3  T_f1_f2, g2o::SparseOptimizer& opt);
//回环检测
/******在当前帧附近nearbyloop(5)关键帧搜索回环，chi2_threshold为chi2阈值**********/
void NearbyLoopDet(vector<RGBD_DSLAM::FramePtr> & keyframes, RGBD_DSLAM::FramePtr frame_cur, g2o::SparseOptimizer& opt,
		                    const int &  nearby_loop,const double& chi2_threshold);

/******在当前帧附近2关键帧作为回环**********/
void NearbyLoopDet(vector<RGBD_DSLAM::FramePtr> & keyframes, RGBD_DSLAM::FramePtr frame_cur, g2o::SparseOptimizer& opt);

bool loopdetect(vector<RGBD_DSLAM::FramePtr> & keyframes, RGBD_DSLAM::FramePtr frame_cur, g2o::SparseOptimizer& opt,const double& radius);

//一些类型转换函数
cv::Mat toCvMat(const Eigen::Matrix<double,4,4>& m);
Eigen::Isometry3d toEigenIsometry(const Sophus::SE3 & m);

//声明g2o相关别名
typedef g2o::BlockSolver_6_3 SlamBlockSolver; 
typedef g2o::LinearSolverCSparse< SlamBlockSolver::PoseMatrixType > SlamLinearSolver; 

int main(int argc, char **argv)
{
    if(argc != 6)
    {
        cerr << endl << "Usage: ./sparse_direct  path_to_settings path_to_sequence path_to_association path_to_result path_to_truthtrans" << endl;
        return 1;
    }
    // Retrieve paths to images
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    string strAssociationFilename = string(argv[3]);
    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);
    //读取真实位姿
    string GroudtruthFilename = string(argv[5]);
    vector<Sophus::SE3> gt;
    LoadGroudtruth(GroudtruthFilename, gt);
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
    CAMERA_INTRINSIC_PARAMETERS c;
    c.cx = fsSettings["Camera.cx"];
    c.cy = fsSettings["Camera.cy"];
    c.fx = fsSettings["Camera.fx"];
    c.fy = fsSettings["Camera.fy"];
    c.k1= fsSettings["Camera.k1"];
    c.k2= fsSettings["Camera.k2"];
    c.p1= fsSettings["Camera.p1"];
    c.p2= fsSettings["Camera.p2"];
    c.k3= fsSettings["Camera.k3"];
    c.width =fsSettings["Camera.width"];
    c.height = fsSettings["Camera.height"];
    c.scale = fsSettings["DepthMapFactor"];
    double fai_min = fsSettings["fai_min"];
    double fai_max = fsSettings["fai_max"];
    int nearby_loop= fsSettings["nearby_loop"];
    double chi2_threshold= fsSettings["chi2_threshold"];
    
    cam  = new vk::PinholeCamera(c.width, c.height, c.fx, c.fy, c.cx, c.cy,
                                                         c.k1,c.k2,c.p1,c.p2,c.k3 );
    RGBD_DSLAM::feature_detection::FastDetector feature_detector(cam->width(), cam->height(), RGBD_DSLAM::Config::gridSize(), RGBD_DSLAM::Config::nPyrLevels());
  
    cv::Mat color_image, depth_image;
    Sophus::SE3 T_prev_w,T_cur_ref;
    string result_path = string(argv[4]);
    string Trajectory_name(result_path+"/CameraTrajectory.txt");
    std::ofstream ofs(Trajectory_name.c_str());
    //判断是否主动退出
    bool run =true;
    //保存关键帧
    vector<RGBD_DSLAM::FramePtr > keyframes;
    // 保存位移矢量误差
    vector<double> trans_error;
    //保存相邻帧光度误差平方和均值
    vector<double> intensity_error;
    //保存李代数协方差行列式的自然对数ln(det(covariance))
    vector<double> ln_cov;
    /******************************* 
    //有关g2o的初始化
    *******************************/
    // 初始化求解器
    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering( false );
    SlamBlockSolver* blockSolver = new SlamBlockSolver( linearSolver );
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( blockSolver );

    g2o::SparseOptimizer optimizer;  // 最后用的就是这个东东
    optimizer.setAlgorithm( solver ); 
    optimizer.setVerbose( false );// 不要输出调试信息
    //定义核函数
   
    //************************//
    
     for(int index = 0; index < nImages ; index++)
    {
         color_image=cv::imread(string(argv[2])+"/"+vstrImageFilenamesRGB[index],0);
         depth_image = cv::imread(string(argv[2])+"/"+vstrImageFilenamesD[index],0);

         assert(color_image.type() == CV_8UC1 && !color_image.empty());
         //读取真实位姿
	 Sophus::SE3 pose_truth= gt[index];
	 
         if(index==0)
         {
           // 设置参考帧
           frame_ref.reset(new RGBD_DSLAM::Frame(cam, color_image,depth_image,c.scale ,0.0));
           frame_ref->T_f_w_ = pose_truth.inverse();
	    frame_ref->timestamp_ = vTimestamps[index];
           //save the camera trajectory
            Quaterniond q2 = frame_ref->T_f_w_.inverse().unit_quaternion();
           ofs << std::fixed<<setprecision(6) << vTimestamps[index]<<" "<<setprecision(9) << frame_ref->T_f_w_.inverse().translation().transpose() << " " << q2.x()<<" "<< q2.y()<<" "<<q2.z()<<" "<<q2.w()<< std::endl;

           // 提取FAST特征，并生成3D点
           feature_detector.detect(frame_ref.get(), frame_ref->img_pyr_, RGBD_DSLAM::Config::triangMinCornerScore(), frame_ref->fts_);
           printf("Added %zu 3d pts to the reference frame.\n", frame_ref->nObs());
	   //选取为关键帧
	   keyframes.push_back(frame_ref);
	   /****************************
	        向optimizer增加第一个顶点
	   ****************************/
	   //g2o::VertexSE3Expmap * v = new g2o::VertexSE3Expmap();
           g2o::VertexSE3* v = new g2o::VertexSE3();
           v->setId(frame_ref->id_);
	   //cout<<"frame id is :"<<frame_ref->id_<<endl;
           v->setEstimate( toEigenIsometry(frame_ref->T_f_w_.inverse() ) ); //估计为单位矩阵
           v->setFixed( true ); //第一个顶点固定，不用优化
           optimizer.addVertex( v );
	   /********验证类型转换*******
	   cout<<"frame_ref->T_f_w_.matrix()= \n"<<frame_ref->T_f_w_.matrix()<<endl;
	   cout<<"toEigenIsometry(frame_ref->T_f_w_).matrix()"<<toEigenIsometry(frame_ref->T_f_w_).matrix()<<endl;
	   /*************************/
           
	   /********验证ground truth******right
	   cout<<"truht trans = "<<pose_truth.translation().transpose()<<endl;
	   /**************************/
	   
	   //******保存位姿位移矢量误差*****
	   Sophus::SE3 T_f_gt = frame_ref->T_f_w_ * pose_truth;
	   trans_error.push_back(T_f_gt.translation().norm());
	   
	   /*******保存光度误差平方和均值***/
	   intensity_error.push_back(0.0);
	  
	   //保存李代数协方差行列式的自然对数ln(det(covariance))
	   ln_cov.push_back(-300.0);
	    //*****************************/
	   T_prev_w = frame_ref->T_f_w_;
           continue;
         }

         frame_cur.reset(new RGBD_DSLAM::Frame(cam, color_image, depth_image,c.scale,0.0));
         //frame_cur_->T_f_w_ = frame_ref_->T_f_w_; // start at reference frame
         frame_cur->T_f_w_ = T_prev_w; // start at last frame
         frame_cur->timestamp_ = vTimestamps[index];
         // run image align
         vk::Timer t;

         RGBD_DSLAM::SparseImgAlign img_align(RGBD_DSLAM::Config::kltMaxLevel(), RGBD_DSLAM::Config::kltMinLevel(),
                                         30, RGBD_DSLAM::SparseImgAlign::GaussNewton, false, false);
         img_align.run(frame_ref, frame_cur,T_cur_ref);
         frame_cur->T_f_w_ = T_cur_ref * frame_ref->T_f_w_;
	  
            //******保存位姿位移矢量误差*****
	   Sophus::SE3 T_f_gt = frame_cur->T_f_w_ * pose_truth;
	   trans_error.push_back(T_f_gt.translation().norm());
	   
	   printf("[%3.i] time = %f ms \t |t| = %f \t translation error = %f\n",
           index, t.stop()*1000, (frame_ref->T_f_w_*pose_truth).translation().norm(),
           trans_error.back());
	   /*******保存光度误差平方和均值***/
	   intensity_error.push_back(img_align.getChi2());
	   cout<<"vo chi2: "<<intensity_error.back()<<endl;
	   
	   /*******李代数信息矩阵的逆为协方差矩阵**********/
	   Matrix<double, 6, 6>  covariance = img_align.getFisherInformation().inverse();
	   //ln(det(covariance))
	   ln_cov.push_back( log(covariance.determinant()) );
	   //cout<<"ln(det(covariance)) : "<<ln_cov.back()<<endl;
	   //*****************************/
	   
        
	 
         //********* check key frame关键帧检测 ************/
	 if (isKeyFrame(keyframes.back() , frame_cur, fai_max,fai_min) )
	 {
	      cout<<" It is a keyframe\n";
	      //增加g2o定点和边
	      //顶点
	      g2o::VertexSE3* v = new g2o::VertexSE3();
              v->setId(frame_cur->id_);
	      //cout<<"frame id is :"<<frame_cur->id_<<endl;
              v->setEstimate( toEigenIsometry(frame_cur->T_f_w_.inverse()) ); //估计位姿
              optimizer.addVertex( v );
	      //添加位姿图的边
	      RGBD_DSLAM::FramePtr lastkeyframe = keyframes.back();
              add_edge(lastkeyframe,frame_cur,lastkeyframe->T_f_w_*(frame_cur->T_f_w_.inverse()),optimizer);
	      //回环检测
	      //近回环检测
	      //frame_cur->T_f_w_=pose_truth.inverse();
	      
	      //NearbyLoopDet( keyframes, frame_cur, optimizer);//2017.2.8
              // loopdetect(keyframes,frame_cur,optimizer);
	      
	      //提取参考帧特征点
              feature_detector.detect(frame_cur.get(), frame_cur->img_pyr_, RGBD_DSLAM::Config::triangMinCornerScore(), frame_cur->fts_);
	     
	     // frame_cur->T_f_w_=pose_truth.inverse();
	      keyframes.push_back(frame_cur);
	 }
             //save the camera trajectory
              Quaterniond q2 = frame_cur->T_f_w_.inverse().unit_quaternion();
              ofs <<std::fixed<< setprecision(6)<<vTimestamps[index]<<" "<<setprecision(9) << frame_cur->T_f_w_.inverse().translation().transpose() << " "
             << q2.x()<<" "<< q2.y()<<" "<<q2.z()<<" "<<q2.w()<< std::endl;
	     
	   /********验证ground truth******right
	   cout<<"truht trans = "<<pose_truth.translation().transpose()<<endl;
	   //**************************/
         // save old pose for next iteration
         T_prev_w = frame_cur->T_f_w_;
	 
	  //绘画相邻帧的匹配点
          run = RGBD_DSLAM::test_utils::plot_points(frame_ref,frame_cur,"vo result");
	  if( !run)
	    break;
	 
              /***更新参考帧
              frame_ref= frame_cur;
	      frame_ref->T_f_w_ = pose_truth.inverse();
              //提取参考帧特征点
              //vk::Timer t1;
              feature_detector.detect( frame_ref.get(), frame_ref->img_pyr_, RGBD_DSLAM::Config::triangMinCornerScore(), frame_ref->fts_);
             // printf("Fast corner detection took %f ms, %zu corners detected (ref i7-W520: 7.166360ms, 40000)\n", t1.stop()*1000, frame_ref_->fts_.size());*/
	      frame_ref = keyframes.back();
     }
     
     cout<<"关键帧数量 ："<< keyframes.size()<<endl;
     ofs.close();
     //光度误差平方和均值
      string IntensityError_Name(result_path+"/intensity_error.txt");
      ofs.open(IntensityError_Name.c_str());
      for(auto it=intensity_error.begin(),  ite=intensity_error.end(); it!=ite; ++it)
      {
	ofs << *it <<endl;
      }
      ofs.close();
      
      //位移矢量误差
      string TransError_Name(result_path+"/trans_error.txt");
      ofs.open(TransError_Name.c_str());
      for(auto it=trans_error.begin(),  ite=trans_error.end(); it!=ite; ++it)
      {
	ofs << *it <<endl;
      }
      ofs.close();
      //李代数协方差行列式的自然对数ln(det(covariance))
      string LnCov_Name(result_path+"/ln_cov.txt");
      ofs.open(LnCov_Name.c_str());
      for(auto it=ln_cov.begin(),  ite=ln_cov.end(); it!=ite; ++it)
      {
	ofs << *it <<endl;
      }
      ofs.close();
       // 保存优化前位姿
       string BefOptTraj_Name(result_path+"/bef_opt_traj.txt");
       ofs.open(BefOptTraj_Name.c_str());
       for(size_t i=0; i<keyframes.size();i++)
       {
	  g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*>(optimizer.vertex( keyframes[i]->id_));
          Eigen::Isometry3d pose = vertex->estimate(); //该帧优化后的位姿
	  Eigen::Quaterniond q3 = Quaterniond ( pose.rotation());
	  ofs <<std::fixed<< setprecision(6)<<keyframes[i]->timestamp_<<" "<<setprecision(9) << pose.translation().transpose() << " "
             << q3.x()<<" "<< q3.y()<<" "<<q3.z()<<" "<<q3.w()<< std::endl;
      }
      ofs.close();
      
      //g2o 优化
       cout<<"optimizing pose graph, vertices: "<<optimizer.vertices().size()<<endl;
       optimizer.save(string(result_path+"/result_before.g2o").c_str());
       optimizer.initializeOptimization();
       optimizer.optimize( 30 ); //可以指定优化步数
       optimizer.save( string(result_path+"/result_after.g2o" ).c_str());
       cout<<"optimization done."<<endl;
       
       // 保存优化后位姿
       string AfOptTraj_Name(result_path+"/af_opt_traj.txt");
       ofs.open(AfOptTraj_Name.c_str());
       for(size_t i=0; i<keyframes.size();i++)
       {
	  g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*>(optimizer.vertex( keyframes[i]->id_));
          Eigen::Isometry3d pose = vertex->estimate(); //该帧优化后的位姿
	  Eigen::Quaterniond q3 = Quaterniond ( pose.rotation());
	  ofs <<std::fixed<< setprecision(6)<<keyframes[i]->timestamp_<<" "<<setprecision(9) << pose.translation().transpose() << " "
             << q3.x()<<" "<< q3.y()<<" "<<q3.z()<<" "<<q3.w()<< std::endl;
      }
      ofs.close();
      
      optimizer.clear();//结束时出现段错误
     return 0;
}

bool isKeyFrame(RGBD_DSLAM::FramePtr frame1, RGBD_DSLAM::FramePtr frame2, const double& fai_max,const double &fai_min)
{
   cv::Mat rvec , tvec;
   Sophus::SE3   T = frame1->T_f_w_ * frame2->T_f_w_.inverse();//Tc1c2
   cv::Mat Tc1c2 = toCvMat(T.matrix());
   cv::Mat Tc1c2_R = Tc1c2(cv::Rect(0,0,3,3));
   cv::Rodrigues(Tc1c2_R , rvec);
   tvec = Tc1c2(cv::Rect(3,0,1,3));
  // cout<<"tvec\n" <<tvec <<endl;
   cv::Mat w1 = (cv::Mat_<float>(1,3) << 0.6, 0.7, 0.7);
   double w2 = 1.3;
   //cout<<"w1 = "<<w1<<endl;
   double fai = w2*fabs( min( cv::norm(rvec), 2*M_PI - cv::norm(rvec) ) ) +  cv::norm(w1*tvec);
   //cout<<"norm(w1*tvec) = " << cv::norm(w1*tvec)<<endl;
   //cout<<" fai =" <<fai<<endl;
   if (fai >= fai_max || fai <= fai_min)
     return false;
   else
     return true ;
}
void add_edge(RGBD_DSLAM::FramePtr frame1, RGBD_DSLAM::FramePtr frame2, Sophus::SE3  T_f1_f2,g2o::SparseOptimizer& opt)
{
      g2o::EdgeSE3* edge = new g2o::EdgeSE3();
      static g2o::RobustKernel* robustKernel = g2o::RobustKernelFactory::instance()->construct( "Cauchy" );      
      // 连接此边的两个顶点id
      edge->vertices() [0] = opt.vertex( frame1->id_);
      edge->vertices() [1] = opt.vertex( frame2->id_);
      edge->setRobustKernel( robustKernel );
       // 信息矩阵
       Eigen::Matrix<double, 6, 6> information = Eigen::Matrix< double, 6,6 >::Identity();
       // 信息矩阵是协方差矩阵的逆，表示我们对边的精度的预先估计
       // 因为pose为6D的，信息矩阵是6*6的阵，假设位置和角度的估计精度均为0.1且互相独立
       // 那么协方差则为对角为0.01的矩阵，信息阵则为100的矩阵
        information(0,0) = information(1,1) = information(2,2) = 100;
        information(3,3) = information(4,4) = information(5,5) = 100;
         // 也可以将角度设大一些，表示对角度的估计更加准确
         edge->setInformation( information );
         // 边的估计是T_f1_f2
         edge->setMeasurement( toEigenIsometry(T_f1_f2));
           // 将此边加入图中
         opt.addEdge(edge);
}
//void NearbyLoopDet(vector<RGBD_DSLAM::FramePtr> & keyframes, RGBD_DSLAM::FramePtr frame_cur, g2o::SparseOptimizer& opt,
//		                     RGBD_DSLAM::SparseImgAlign &img_align,const int & nearby_loop, const double& chi2_threshold)
/******在当前帧附近nearbyloop(5)关键帧搜索回环，chi2_threshold为chi2阈值**********/
void NearbyLoopDet(vector<RGBD_DSLAM::FramePtr> & keyframes, RGBD_DSLAM::FramePtr frame_cur, g2o::SparseOptimizer& opt,
                                     const int & nearby_loop, const double& chi2_threshold)
{
        vector<RGBD_DSLAM::FramePtr> candidate_loop;
	vector<double> chi2s;
	vector<Sophus::SE3> T;
	Sophus::SE3 T_cur_ref;
	/******在当前帧附近nearbyloop(5)关键帧搜索回环**********/
	//判断关键帧数量
	if( keyframes.size() <= nearby_loop)
	{
	    for(auto it=keyframes.begin(),  ite=keyframes.end(); it!=ite-1; ++it)
            {
	         RGBD_DSLAM::SparseImgAlign img_align2(RGBD_DSLAM::Config::kltMaxLevel(), RGBD_DSLAM::Config::kltMinLevel(),
                                         30, RGBD_DSLAM::SparseImgAlign::GaussNewton, false, false);
		 
	          img_align2.run( *it,  frame_cur,T_cur_ref);
		  printf("Added %zu 3d pts to the reference frame.\n", (*it)->nObs());
		  cout<<" loop detect chi2:"<<img_align2.getChi2()<<endl;
		     /*****测试回环误差*******
		   Sophus::SE3 T_f_gt = T_cur_ref * (*it)->T_f_w_*frame_cur->T_f_w_ .inverse();
	           printf("|t| = %f \t translation error = %f\n", ((*it)->T_f_w_*frame_cur->T_f_w_ .inverse()).translation().norm(),
                    T_f_gt.translation().norm() );
		   /***********************/
		  RGBD_DSLAM::test_utils::plot_points(*it,frame_cur,T_cur_ref,"loop detect");
		  
		  if( img_align2.getChi2() <= chi2_threshold)
		  {
		    candidate_loop.push_back(*it);
		    chi2s.push_back(img_align2.getChi2());
		    T.push_back(T_cur_ref);
		  }     
            }
	}
        else
	{
	        for(auto it=keyframes.end()-nearby_loop,  ite=keyframes.end(); it!=ite-1; ++it)
            {
	          RGBD_DSLAM::SparseImgAlign img_align2(RGBD_DSLAM::Config::kltMaxLevel(), RGBD_DSLAM::Config::kltMinLevel(),
                                         30, RGBD_DSLAM::SparseImgAlign::GaussNewton, false, false);
		  
	           img_align2.run( *it,  frame_cur,T_cur_ref);
		   printf("Added %zu 3d pts to the reference frame.\n", (*it)->nObs());
		   cout<<" loop detect chi2:"<<img_align2.getChi2()<<endl;
		   /*****测试回环误差*******
		   Sophus::SE3 T_f_gt = T_cur_ref * (*it)->T_f_w_*frame_cur->T_f_w_ .inverse();
	           printf("|t| = %f \t translation error = %f\n", ((*it)->T_f_w_*frame_cur->T_f_w_ .inverse()).translation().norm(),
                    T_f_gt.translation().norm() );
		   /***********************/
		   RGBD_DSLAM::test_utils::plot_points(*it,frame_cur,T_cur_ref,"loop detect");
		   
		  if( img_align2.getChi2() <= chi2_threshold)
		  {
		    candidate_loop.push_back(*it);
		    chi2s.push_back(img_align2.getChi2());
		     T.push_back(T_cur_ref);
		  }   
            }
	}
       /*****选择chi2最小的两帧作为会还加入到位姿图中********/
       // 判断候选会还数量
       if(chi2s.size() <= 2)
       {
	  vector<Sophus::SE3>::iterator itT = T.begin();
	  for(auto it=candidate_loop.begin() , ite=candidate_loop.end(); it!=ite; ++it,++itT)
	  {
	    add_edge(*it, frame_cur, (*itT).inverse(),opt);
	    cout<<"add loop"<<endl;
	  }
      }
      else
      {
        vector<size_t> index=RGBD_DSLAM::sort_indexes(chi2s);
        for( size_t i=0;i <2;i++ )
	{
	  add_edge(candidate_loop[ index[i] ], frame_cur,T[ index[i] ].inverse() ,opt);
	  cout<<"add loop"<<endl;
	}
      }
}
/******在当前帧附近2关键帧作为回环**********/
void NearbyLoopDet(vector<RGBD_DSLAM::FramePtr> & keyframes, RGBD_DSLAM::FramePtr frame_cur, g2o::SparseOptimizer& opt)
{
	Sophus::SE3 T_cur_ref;
	/******在当前帧附近nearbyloop(5)关键帧搜索回环**********/
	//判断关键帧数量
	if( keyframes.size() <= 3)
	{
	    for(auto it=keyframes.begin(),  ite=keyframes.end(); it!=ite-1; ++it)
            {
	         RGBD_DSLAM::SparseImgAlign img_align2(RGBD_DSLAM::Config::kltMaxLevel(), RGBD_DSLAM::Config::kltMinLevel(),
                                         30, RGBD_DSLAM::SparseImgAlign::GaussNewton, false, false);
		 
	          img_align2.run( *it,  frame_cur,T_cur_ref);
		  printf("Added %zu 3d pts to the reference frame.\n", (*it)->nObs());
		  cout<<" loop detect chi2:"<<img_align2.getChi2()<<endl;
		     /*****测试回环误差*******/
		   Sophus::SE3 T_f_gt = T_cur_ref * (*it)->T_f_w_*frame_cur->T_f_w_ .inverse();
	           printf("|t| = %f \t translation error = %f\n", ((*it)->T_f_w_*frame_cur->T_f_w_ .inverse()).translation().norm(),
                    T_f_gt.translation().norm() );
		   /***********************/
		  RGBD_DSLAM::test_utils::plot_points(*it,frame_cur,T_cur_ref,"loop detect");
                  add_edge(*it, frame_cur, T_cur_ref.inverse(),opt);
            }
	}
        else
	{
	        for(auto it=keyframes.end()-3,  ite=keyframes.end(); it!=ite-1; ++it)
            {
	          RGBD_DSLAM::SparseImgAlign img_align2(RGBD_DSLAM::Config::kltMaxLevel(), RGBD_DSLAM::Config::kltMinLevel(),
                                         30, RGBD_DSLAM::SparseImgAlign::GaussNewton, false, false);
		  
	           img_align2.run( *it,  frame_cur,T_cur_ref);
		   printf("Added %zu 3d pts to the reference frame.\n", (*it)->nObs());
		   cout<<" loop detect chi2:"<<img_align2.getChi2()<<endl;
		   /*****测试回环误差*******
		   Sophus::SE3 T_f_gt = T_cur_ref * (*it)->T_f_w_*frame_cur->T_f_w_ .inverse();
	           printf("|t| = %f \t translation error = %f\n", ((*it)->T_f_w_*frame_cur->T_f_w_ .inverse()).translation().norm(),
                    T_f_gt.translation().norm() );
		   /***********************/
		   RGBD_DSLAM::test_utils::plot_points(*it,frame_cur,T_cur_ref,"loop detect");
                   add_edge(*it, frame_cur, T_cur_ref.inverse(),opt);
            }
	}
}
bool loopdetect(vector<RGBD_DSLAM::FramePtr> & keyframes, RGBD_DSLAM::FramePtr frame_cur, g2o::SparseOptimizer& opt, const double& radius)
{
   for(auto it=keyframes.begin(),  ite=keyframes.end(); it!=ite; ++it)
   {
     Sophus::SE3   T = (*it)->T_f_w_ * frame_cur->T_f_w_.inverse();//Tc1c2
     cv::Mat Tc1c2 = toCvMat(T.matrix());
  }
  return true;
}
cv::Mat toCvMat(const Eigen::Matrix<double,4,4>& m)
{
  cv::Mat cvMat(4,4,CV_32F);
  for(int i=0; i<4;i++)
    for(int j=0;j<4;j++)
      cvMat.at<float>(i,j) = m(i,j);
   
      return cvMat.clone();
}
Eigen::Isometry3d toEigenIsometry(const Sophus::SE3& m)
{
  Eigen::Isometry3d T;
  T = m.unit_quaternion();
  T.translation() = m.translation();
  return T;
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
void LoadGroudtruth(const string& GroudtruthFilename, vector<Sophus::SE3>& gt)
{
    ifstream groundtruth;
    groundtruth.open(GroudtruthFilename.c_str());
    Quaterniond q;
    Vector3d t;
    while(!groundtruth.eof())
    {
        string s;
        getline(groundtruth,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double m,qx,qy,qz,qw;
            ss >> m;
            ss >> t[0];
	    ss >> t[1];
	    ss >> t[2];
	    ss >> qx;
	    ss >> qy;
	    ss >> qz;
	    ss >> qw;
	    q=Eigen::Quaterniond(qw, qx, qy, qz);
	    gt.push_back(Sophus::SE3(q,t));
        }
    }
}
