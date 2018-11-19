#include <iostream>
#include <opencv2/highgui.hpp>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "mynteye/api/api.h"
#include "bi_cam.h"
using namespace std;
MYNTEYE_USE_NAMESPACE
int main() {

    /*
    auto &&api = API::Create();
    if (!api)
        return 1;
    api->EnableStreamData(Stream::LEFT_RECTIFIED);
    api->EnableStreamData(Stream::RIGHT_RECTIFIED);
    api->Start(Source::VIDEO_STREAMING);

    //qqcv::namedWindow("Frame");
    std::cout << "Hello, World!" << std::endl;
    while(true){
        api->WaitForStreams();

        auto &&left_data=api->GetStreamData(Stream::LEFT_RECTIFIED);
        auto &&right_data=api->GetStreamData(Stream::RIGHT_RECTIFIED);

        if(!left_data.frame.empty() && !right_data.frame.empty()){
            cv::Mat img;
            cv::hconcat(left_data.frame, right_data.frame, img);
            cv::imshow("frame",img);
        }

        char key = static_cast<char>(cv::waitKey(1));
        if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
            break;
        }

    }
    */
    bi_cam bi;
    int w=9, h=6;
    float  s=33.0;
    string filename = "stereo_calib.xml";
    string save_path = "./imglist/";
    bool res = bi.collect_images(save_path);
    if(res)
        cout<<"success"<<endl;
    else
        cout<<"failed"<<endl;
    //bi.calib_bino(w,h,s,filename);
    //bi.calib_bino(w,h, s, filename);

    cv::waitKey();
    return 0;
}