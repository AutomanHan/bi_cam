#ifndef __BI_CAM_H__
#define __BI_CAM_H__
#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

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

using namespace std;
using namespace cv;
class bi_cam{
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
    void get_disparity();

    //handling disparity
    void handling_diparity();

    //get depth
    void get_depth();

    //get cloud point
    void get_point();
};

#endif