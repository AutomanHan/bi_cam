#ifndef __BI_CAM_H__
#define __BI_CAM_H__
#include <string>
using namespace std;
class bi_cam{
    static bool readStringList(const string& filename, vector<string>& l);
    //calibrate camera
    //single camera
    void calib_single(string imglist);
    //Binocular camera
    void calib_binocular(const vector<string>& imagelist, Size boardSize, float squareSize, bool displayCorners = false, bool useCalibrated=true, bool showRectified=true);

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