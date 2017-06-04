#include <svo/config.h>
#include <svo/frame_handler_mono.h>
#include <svo/map.h>
#include <svo/frame.h>
#include<svo/initialization.h>
#include <vector>
#include <string>
#include <vikit/math_utils.h>
#include <vikit/vision.h>
#include <vikit/abstract_camera.h>
#include <vikit/atan_camera.h>
#include <vikit/pinhole_camera.h>
#include <opencv2/opencv.hpp>
#include <sophus/se3.h>
#include <iostream>
#include "test_utils.h"

using namespace svo;
using namespace std;
bool addfirstframe = true;
bool addsecondframe = true;

int main(int argc, char *argv[])
{
	vk::AbstractCamera* cam = new vk::PinholeCamera(752, 480, 315.5, 315.5, 376.0, 240.0);
	svo	::initialization::KltHomographyInit init;

	for(int img_id = 2; img_id < 188; ++img_id)
  {
        // load image
        std::stringstream ss;
        
        ss << svo::test_utils::getDatasetDir() << "/sin2_tex2_h1_v8_d/img/frame_"
        << std::setw( 6 ) << std::setfill( '0' ) << img_id << "_0.png";
        cout<<"hello"<<endl;
        if(img_id == 2)
        std::cout << "reading image " << ss.str() << std::endl;
        cv::Mat img(cv::imread(ss.str().c_str(), 0));
        assert(!img.empty() && img.type() == CV_8UC1);

        FramePtr frame(new Frame(cam, img, 0.0));
        if(addfirstframe)
          {
              if (init.addFirstFrame(frame)!= svo::initialization::SUCCESS)
                 continue;
              else
                {
                  addfirstframe = false;
                  continue;

                }

          }

     if(addsecondframe)
       {
          if( init.addSecondFrame(frame) != svo::initialization::SUCCESS)
          continue;
          else
          {
            std::cout << frame->T_f_w_.matrix() << std::endl;
            addsecondframe = false;
            break;
          }
       }
  }
	return 0;
}
