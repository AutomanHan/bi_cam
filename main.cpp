#include <iostream>
#include <opencv2/highgui.hpp>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "mynteye/api/api.h"
#include "bi_cam.h"


//#include <pcl-1.7/pcl/common/common_headers.h>
//#include <pcl-1.7/pcl/io/pcd_io.h>
//#include <pcl-1.7/pcl/visualization/pcl_visualizer.h>
//#include <pcl-1.7/pcl/visualization/cloud_viewer.h>
//#include <pcl-1.7/pcl/console/parse.h>

using namespace std;
MYNTEYE_USE_NAMESPACE
int main() {

    bi_cam bi;
    int w=9, h=6;
    float  s=33.0;
    string filename = "stereo_calib.xml";
    string save_path = "../imglist/";
    //bool res = bi.collect_images(save_path);
    //bi.calib_bino(7, 5, 33.0, filename);
    //bi.manual_exposure(24, 180, 127);
    /*
    Mat left = imread("../aloeL.jpg");
    Mat right = imread("../aloeR.jpg");
    cout<<left.size()<<endl;
    cout<<right.size()<<endl;
    Mat dis_16s = Mat(left.rows, left.cols, CV_16S);
    Mat dis_8u = Mat(right.rows, right.cols, CV_8UC1);
    bi.get_disparity(left, right, dis_16s,dis_8u);
    //imshow("disparity", dis_8u);
    Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
    cameraMatrix.at<float>(0, 0) = 843.9546;
    cameraMatrix.at<float>(0, 1) = 0;
    cameraMatrix.at<float>(0, 2) = 288.5969;
    cameraMatrix.at<float>(1, 1) = 844.0386;
    cameraMatrix.at<float>(1, 2) = 221.0515;
    Mat depth_16U = Mat(dis_8u.rows, dis_8u.cols, CV_16UC1);
    Mat depth_8U = Mat(dis_8u.rows, dis_8u.cols, CV_8UC1);
    bi.get_depth(dis_8u, depth_16U, cameraMatrix);
    double minVal, maxVal;
    minMaxLoc(depth_16U, &minVal, &maxVal);
    cout<<"minval: "<<minVal<<endl;
    cout<<"maxVal: "<<maxVal<<endl;
    //depth_16U.convertTo(depth_8U, CV_8UC1, 255/(maxVal-minVal));
    //imshow("depth", depth_8U);

    //bi.calib_bino(w,h,s,filename);
    //bi.calib_bino(w,h, s, filename)；
    */
    string intrinsics_file = "intrinsics.yml";
    string extrinsics_file = "extrinsics.yml";
    cam_paras paras;
    bi.get_ymlfile(intrinsics_file, extrinsics_file, paras);

    //cout<<paras.cameraMatrix[0]<<endl;
    auto &&api = API::Create();
    api->Start(Source::VIDEO_STREAMING);
    while (true) {
        api->WaitForStreams();

        auto &&left_data = api->GetStreamData(Stream::LEFT);
        auto &&right_data = api->GetStreamData(Stream::RIGHT);

        Mat left_rec, right_rec;
        if(!left_data.frame.empty() && !right_data.frame.empty()){
            cv::Mat img;
            cv::hconcat(left_data.frame, right_data.frame, img);
            cv::imshow("frame", img);

            bi.undistortrectiry(left_data.frame, right_data.frame, paras, left_rec, right_rec );
            Mat dis_16s = Mat(left_rec.rows, left_rec.cols, CV_16S);
            Mat dis_8u = Mat(right_rec.rows, right_rec.cols, CV_8UC1);
            //imshow("left_rec", left_rec);
            bi.get_disparity(left_rec, right_rec, dis_16s, dis_8u);

            imshow("disparity", dis_8u);
        }
        char key = static_cast<char>(cv::waitKey(1));
        if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
            break;
        }
    }
    return 0;
}