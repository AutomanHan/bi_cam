#ifndef __BI_CAM_H__
#define __BI_CAM_H__
#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/utility.hpp>

#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>


//#include <direct.h>
#include <sys/stat.h>
#include <sys/types.h>
//#include <sys/io.h>
#include<stdio.h>
#include <unistd.h>

#include "mynteye/api/api.h"
#include "mynteye/logger.h"
//#include "cv_painter.h"
//#include <pcl/point_cloud.h>
#include <pcl-1.8/pcl/point_cloud.h>
#include <pcl-1.8/pcl/io/pcd_io.h>
#include <pcl-1.8/pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>
//#include "pcl/point_cloud.h"
//#include <pcl/point_cloud.h>

using namespace std;
using namespace cv;
using namespace pcl;

struct cam_paras{
    Mat cameraMatrix[2];
    Mat distCoeffs[2];
    Mat R[2];
    Mat P[2];
};
struct rec{
    int w;
    int h;
};
class bi_cam{
public:
    rec src_img;
    rec roi_img;
public:
    bi_cam();
public:
    static bool readStringList(const string& filename, vector<string>& l);
    static void StereoCalib(const vector<string>& imagelist, Size boardSize, float squareSize, bool displayCorners = false, bool useCalibrated=true, bool showRectified=true);

public:
    //collect images
    bool collect_images(string save_path);

    //calibrate camera
    //single camera
    void calib_single(string imglist);
    //Binocular camera

    int calib_bino(int w, int h, float s, string filename);

    //get disparity
    void get_disparity(const Mat &left, const Mat &right, Mat &imgDisparity16S, Mat &imgDisparity8U);

    //handling disparity
    void handling_diparity();

    //get depth
    void get_depth(const Mat& dispMap, Mat &depthMap, Mat k);

    //get cloud point
    void get_point(const Mat& depthMap, PointCloud<PointXYZ>::Ptr pc, Mat k);

    //modify params of camera
    void manual_exposure(int gain, int brightness, int contrast);

    // read yml file
    void get_ymlfile(string intrinsics_file, string extrinsics_file, cam_paras &paras);

    //UndistortRectify image
    void undistortrectiry(const Mat& left_src,const Mat& right_src, const cam_paras& paras, Mat &left_rec, Mat &right_rec);

    // show point cloud
    void show_pointclouds(string file_path);
};

#endif