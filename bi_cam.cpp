#include "bi_cam.h"
using namespace std;
using namespace cv;
MYNTEYE_USE_NAMESPACE
void bi_cam::calib_single(string imglist) {

}
//read imglist from xml file
bool bi_cam::readStringList(const string &filename, vector <string> &l) {
    l.resize(0);
    FileStorage fs(filename, FileStorage::READ);
    if( !fs.isOpened() )
        return false;
    FileNode n = fs.getFirstTopLevelNode();
    if( n.type() != FileNode::SEQ )
        return false;
    FileNodeIterator it = n.begin(), it_end = n.end();
    for( ; it != it_end; ++it )
        l.push_back((string)*it);
    return true;
}
void bi_cam::StereoCalib(const vector<string>& imagelist, Size boardSize, float squareSize, bool displayCorners , bool useCalibrated, bool showRectified) {
    if( imagelist.size() % 2 != 0 )
    {
        cout << "Error: the image list contains odd (non-even) number of elements\n";
        return;
    }

    const int maxScale = 2;
    // ARRAY AND VECTOR STORAGE:

    vector<vector<Point2f> > imagePoints[2];
    vector<vector<Point3f> > objectPoints;
    Size imageSize;

    int i, j, k, nimages = (int)imagelist.size()/2;

    imagePoints[0].resize(nimages);
    imagePoints[1].resize(nimages);
    vector<string> goodImageList;

    for( i = j = 0; i < nimages; i++ )
    {
        for( k = 0; k < 2; k++ )
        {
            const string& filename = imagelist[i*2+k];
            Mat img = imread(filename, 0);
            if(img.empty())
                break;
            if( imageSize == Size() )
                imageSize = img.size();
            else if( img.size() != imageSize )
            {
                cout << "The image " << filename << " has the size different from the first image size. Skipping the pair\n";
                break;
            }
            bool found = false;
            vector<Point2f>& corners = imagePoints[k][j];
            for( int scale = 1; scale <= maxScale; scale++ )
            {
                Mat timg;
                if( scale == 1 )
                    timg = img;
                else
                    resize(img, timg, Size(), scale, scale, INTER_LINEAR_EXACT);
                found = findChessboardCorners(timg, boardSize, corners,
                                              CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
                if( found )
                {
                    if( scale > 1 )
                    {
                        Mat cornersMat(corners);
                        cornersMat *= 1./scale;
                    }
                    break;
                }
            }
            if( displayCorners )
            {
                cout << filename << endl;
                Mat cimg, cimg1;
                cvtColor(img, cimg, COLOR_GRAY2BGR);
                drawChessboardCorners(cimg, boardSize, corners, found);
                double sf = 640./MAX(img.rows, img.cols);
                resize(cimg, cimg1, Size(), sf, sf, INTER_LINEAR_EXACT);
                imshow("corners", cimg1);
                char c = (char)waitKey(500);
                if( c == 27 || c == 'q' || c == 'Q' ) //Allow ESC to quit
                    exit(-1);
            }
            else
                putchar('.');
            if( !found )
                break;
            cornerSubPix(img, corners, Size(11,11), Size(-1,-1),
                         TermCriteria(TermCriteria::COUNT+TermCriteria::EPS,
                                      30, 0.01));
        }
        if( k == 2 )
        {
            goodImageList.push_back(imagelist[i*2]);
            goodImageList.push_back(imagelist[i*2+1]);
            j++;
        }
    }
    cout << j << " pairs have been successfully detected.\n";
    nimages = j;
    if( nimages < 2 )
    {
        cout << "Error: too little pairs to run the calibration\n";
        return;
    }

    imagePoints[0].resize(nimages);
    imagePoints[1].resize(nimages);
    objectPoints.resize(nimages);

    for( i = 0; i < nimages; i++ )
    {
        //vector<cv::Point3d> obj;
        for( j = 0; j < boardSize.height; j++ )
            for( k = 0; k < boardSize.width; k++ )
                objectPoints[i].push_back(Point3f(k*squareSize, j*squareSize, 0));
    }

    cout << "Running stereo calibration ...\n";

    Mat cameraMatrix[2], distCoeffs[2];
    cameraMatrix[0] = initCameraMatrix2D(objectPoints,imagePoints[0],imageSize,0);
    cameraMatrix[1] = initCameraMatrix2D(objectPoints,imagePoints[1],imageSize,0);
    Mat R, T, E, F;

    /*
    Mat cameraMatrix[2],R;
    Vec3d T;
    Vec4d distCoeffs[2];
    */
    int flag = 0;
    flag |= cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;
    flag |= cv::fisheye::CALIB_CHECK_COND;
    flag |= cv::fisheye::CALIB_FIX_SKEW;
    flag |= cv::fisheye::CALIB_FIX_K2;
    flag |= cv::fisheye::CALIB_FIX_K3;
    flag |= cv::fisheye::CALIB_FIX_K4;
    double rms = fisheye::stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
                                 cameraMatrix[0], distCoeffs[0],
                                 cameraMatrix[1], distCoeffs[1],
                                 imageSize, R, T,
                                 flag,
                                 //TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 100, 1e-5) );
                                 cv::TermCriteria(3, 12, 4));

    //fisheye::calibrate(objectPoints, imagePoints[0], imageSize, cameraMatrix[0],distCoeffs[0],
    //                    R, T, flag, cv::TermCriteria(3, 20, 1e-6));


    cout << "done with RMS error=" << rms << endl;

// CALIBRATION QUALITY CHECK
// because the output fundamental matrix implicitly
// includes all the output information,
// we can check the quality of calibration using the
// epipolar geometry constraint: m2^t*F*m1=0
/*
    double err = 0;
    int npoints = 0;
    vector<Vec3f> lines[2];
    for( i = 0; i < nimages; i++ )
    {
        int npt = (int)imagePoints[0][i].size();
        Mat imgpt[2];
        for( k = 0; k < 2; k++ )
        {
            imgpt[k] = Mat(imagePoints[k][i]);
            undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);
            computeCorrespondEpilines(imgpt[k], k+1, F, lines[k]);
        }
        for( j = 0; j < npt; j++ )
        {
            double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] +
                                imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
                           fabs(imagePoints[1][i][j].x*lines[0][j][0] +
                                imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
            err += errij;
        }
        npoints += npt;
    }
    cout << "average epipolar err = " <<  err/npoints << endl;
*/
    // save intrinsic parameters
    FileStorage fs("intrinsics.yml", FileStorage::WRITE);
    if( fs.isOpened() )
    {
        fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
           "M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
        fs.release();
    }
    else
        cout << "Error: can not save the intrinsic parameters\n";

    Mat R1, R2, P1, P2, Q;
    Rect validRoi[2];

    fisheye::stereoRectify(cameraMatrix[0], distCoeffs[0],
                  cameraMatrix[1], distCoeffs[1],
                  imageSize, R, T, R1, R2, P1, P2, Q,
                  CALIB_ZERO_DISPARITY, imageSize, 0.0, 1.0);

    fs.open("extrinsics.yml", FileStorage::WRITE);
    if( fs.isOpened() )
    {
        fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
        fs.release();
    }
    else
        cout << "Error: can not save the extrinsic parameters\n";

    // OpenCV can handle left-right
    // or up-down camera arrangements
    bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));

// COMPUTE AND DISPLAY RECTIFICATION
    if( !showRectified )
        return;

    Mat rmap[2][2];
// IF BY CALIBRATED (BOUGUET'S METHOD)


    if( useCalibrated )
    {
        // we already computed everything
    }
// OR ELSE HARTLEY'S METHOD
    else
        // use intrinsic parameters of each camera, but
        // compute the rectification transformation directly
        // from the fundamental matrix
    {
        vector<Point2f> allimgpt[2];
        for( k = 0; k < 2; k++ )
        {
            for( i = 0; i < nimages; i++ )
                std::copy(imagePoints[k][i].begin(), imagePoints[k][i].end(), back_inserter(allimgpt[k]));
        }
        F = findFundamentalMat(Mat(allimgpt[0]), Mat(allimgpt[1]), FM_8POINT, 0, 0);
        Mat H1, H2;
        stereoRectifyUncalibrated(Mat(allimgpt[0]), Mat(allimgpt[1]), F, imageSize, H1, H2, 3);

        R1 = cameraMatrix[0].inv()*H1 * cameraMatrix[0];
        R2 = cameraMatrix[1].inv()*H2*cameraMatrix[1];
        P1 = cameraMatrix[0];
        P2 = cameraMatrix[1];
    }

    //Precompute maps for cv::remap()
    fisheye::initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
    fisheye::initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

    Mat canvas;
    double sf;
    int w, h;
    if( !isVerticalStereo )
    {
        sf = 600./MAX(imageSize.width, imageSize.height);
        w = cvRound(imageSize.width*sf);
        h = cvRound(imageSize.height*sf);
        canvas.create(h, w*2, CV_8UC3);
    }
    else
    {
        sf = 300./MAX(imageSize.width, imageSize.height);
        w = cvRound(imageSize.width*sf);
        h = cvRound(imageSize.height*sf);
        canvas.create(h*2, w, CV_8UC3);
    }

    for( i = 0; i < nimages; i++ )
    {
        for( k = 0; k < 2; k++ )
        {
            Mat img = imread(goodImageList[i*2+k], 0), rimg, cimg;
            remap(img, rimg, rmap[k][0], rmap[k][1], INTER_LINEAR);
            cvtColor(rimg, cimg, COLOR_GRAY2BGR);
            Mat canvasPart = !isVerticalStereo ? canvas(Rect(w*k, 0, w, h)) : canvas(Rect(0, h*k, w, h));
            resize(cimg, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);
            if( useCalibrated )
            {
                Rect vroi(cvRound(validRoi[k].x*sf), cvRound(validRoi[k].y*sf),
                          cvRound(validRoi[k].width*sf), cvRound(validRoi[k].height*sf));
                rectangle(canvasPart, vroi, Scalar(0,0,255), 3, 8);
            }
        }

        if( !isVerticalStereo )
            for( j = 0; j < canvas.rows; j += 32 )
                line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
        else
            for( j = 0; j < canvas.cols; j += 32 )
                line(canvas, Point(j, 0), Point(j, canvas.rows), Scalar(0, 255, 0), 1, 8);
        imshow("rectified", canvas);
        char c = (char)waitKey();
        if( c == 27 || c == 'q' || c == 'Q' )
            break;
    }
}

int bi_cam::calib_bino(int w, int h, float s, string filename) {
    Size boardSize;
    boardSize.width = w;
    boardSize.height= h;
    bool showRectified = true;
    float squareSize = s;
    vector<string> imagelist;

    bool ok = readStringList(filename, imagelist);
    if(!ok || imagelist.empty()){
        cout<<"can not open "<< filename<<"or the string list is empty"<<endl;
        return 0;
    }
    StereoCalib(imagelist, boardSize, squareSize, false, true, showRectified);
    return 1;
}
bool bi_cam::collect_images(string save_path) {
    int res = access(save_path.c_str(), F_OK);
    if(res){
        cout<<save_path.c_str()<<" is not exist"<<endl;
        int res = mkdir(save_path.c_str(),S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
        if(res != 0)
            return false;
    }
    auto &&api = API::Create();
    if (!api)
        return false;
    //api->EnableStreamData(Stream::LEFT_RECTIFIED);
    //api->EnableStreamData(Stream::RIGHT_RECTIFIED);
    api->Start(Source::VIDEO_STREAMING);

    //qqcv::namedWindow("Frame");
    std::cout << "Hello, World!" << std::endl;
    char key;
    int i = 0;
    while(true){
        api->WaitForStreams();

        auto &&left_data=api->GetStreamData(Stream::LEFT);
        auto &&right_data=api->GetStreamData(Stream::RIGHT);

        key = static_cast<char>(cv::waitKey(1));
        if(!left_data.frame.empty() && !right_data.frame.empty()){
            cv::Mat img;
            cv::hconcat(left_data.frame, right_data.frame, img);
            cv::imshow("frame",img);

            if(key=='s'|| key=='S'){
                i++;
                char num[10];
                sprintf(num, "%02d", i);
                string left = save_path+"left"+num+".jpg";
                string right = save_path+"right"+num+".jpg";
                imwrite(left, left_data.frame);
                imwrite(right, right_data.frame);
            }

        }
        if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
            break;
        }
    }
    return true;
}
//獲取雙目視差圖
/*
 * params:
 *
 * left: left image
 * right: right image
 * imgDistparity16S: result of sgbm->computer, type: CV_16S
 * imgDistparity8U : visualized disparity image  type: CV_8UC1
 *
 * */
void bi_cam::get_disparity(const Mat &left, const Mat &right, Mat &imgDisparity16S, Mat &imgDisparity8U) {
    //1.Call the constructor for StereoBM
    int cha_left = left.channels();
    int cha_right = right.channels();
    Mat left_gray, right_gray;
    if(cha_left != 1)
        cvtColor(left, left_gray, CV_BGR2GRAY);
    else
        left_gray=left;
    if(cha_right !=1 )
        cvtColor(right, right_gray, CV_BGR2GRAY);
    else
        right_gray=right;

    int numberOfDisparities = ((left_gray.cols / 8) + 15) & -16;
    int SADWindowSize = 9;

    Ptr<StereoSGBM> sgbm = StereoSGBM::create(0, 16,3);

    sgbm->setPreFilterCap(63);
    //int SADWindowSize = 9;
    //int numberOfDisparities=64;
    int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
    sgbm->setBlockSize(sgbmWinSize);
    int cn = left.channels();
    sgbm->setP1(8 * cn*sgbmWinSize*sgbmWinSize);
    sgbm->setP2(32 * cn*sgbmWinSize*sgbmWinSize);
    sgbm->setMinDisparity(0);
    sgbm->setNumDisparities(numberOfDisparities);
    sgbm->setUniquenessRatio(10);
    sgbm->setSpeckleWindowSize(100);
    sgbm->setSpeckleRange(32);
    sgbm->setDisp12MaxDiff(1);

    sgbm->setMode(StereoSGBM::MODE_SGBM);
    sgbm->compute(left_gray, right_gray, imgDisparity16S);

    double minVal, maxVal;
    minMaxLoc(imgDisparity16S, &minVal, &maxVal);
    imgDisparity16S.convertTo(imgDisparity8U, CV_8UC1, 255/(maxVal-minVal));
    //cout<<maxVal<<"\t"<<minVal<<endl;
}

void bi_cam::handling_diparity() {

}
/*
函数作用：视差图转深度图
输入：
　　dispMap ----视差图，8位单通道，CV_8UC1
　　K       ----内参矩阵，float类型
输出：
　　depthMap ----深度图，16位无符号单通道，CV_16UC1
*/
void bi_cam::get_depth(const Mat &dispMap, Mat &depthMap, Mat k) {
    int type = dispMap.type();

    float fx = k.at<float>(0,0);
    float fy = k.at<float>(1,1);
    float dx = k.at<float>(0,2);
    float dy = k.at<float>(1,2);
    float baseline = 1;

    if(type == CV_8U){
        int height = dispMap.rows;
        int width = dispMap.cols;

        unsigned char* dispData = (uchar*)dispMap.data;
        unsigned short* depthData = (ushort*)depthMap.data;
        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
                int id = i * width + j;
                if (!dispData[id]) continue;  //防止0除
                depthData[id] = ushort((float) fx * baseline / ((float) dispData[id]));
            }
        }
    }
    else{
        cout<<"please confirm dispImg's type!"<<endl;
    }
}

void bi_cam::get_point() {

}


//modify params of camera
void bi_cam::manual_exposure(int gain, int brightness, int contrast){
    auto &&api = API::Create();

    api->SetOptionValue(Option::GAIN, gain);
    api->SetOptionValue(Option::BRIGHTNESS, brightness);
    api->SetOptionValue(Option::CONTRAST, contrast);

    LOG(INFO) << "Enable manual-exposure";
    LOG(INFO) << "Set GAIN to " << api->GetOptionValue(Option::GAIN);
    LOG(INFO) << "Set BRIGHTNESS to " << api->GetOptionValue(Option::BRIGHTNESS);
    LOG(INFO) << "Set CONTRAST to " << api->GetOptionValue(Option::CONTRAST);

    api->EnableStreamData(Stream::LEFT_RECTIFIED);
    api->EnableStreamData(Stream::RIGHT_RECTIFIED);

    api->Start(Source::VIDEO_STREAMING);

    //CVPainter painter(api->GetOptionValue(Option::FRAME_RATE));

    cv::namedWindow("frame");

    while (true) {
        api->WaitForStreams();

        auto &&left_data = api->GetStreamData(Stream::LEFT_RECTIFIED);
        auto &&right_data = api->GetStreamData(Stream::RIGHT_RECTIFIED);

        if (!left_data.frame.empty() && !right_data.frame.empty()) {
            cv::Mat img;
            cv::hconcat(left_data.frame, right_data.frame, img);
            cv::imshow("frame", img);
        }

        char key = static_cast<char>(cv::waitKey(1));
        if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
            break;
        }

    }

    api->Stop(Source::VIDEO_STREAMING);
}
void bi_cam::get_ymlfile(string intrinsics_file, string extrinsics_file, cam_paras& paras) {

    FileStorage fs(intrinsics_file, FileStorage::READ);
    if( fs.isOpened() ){
        fs["M1"] >> paras.cameraMatrix[0];
        fs["M2"]>> paras.cameraMatrix[1];
        fs["D1"] >> paras.distCoeffs[0];
        fs["D2"] >> paras.distCoeffs[1];

        fs.release();
    }
    else
        cout << "Error: can not save the intrinsic parameters\n";
    fs.open(extrinsics_file, FileStorage::READ);
    if(fs.isOpened()){
        fs["R1"] >> paras.R[0];
        fs["R2"] >> paras.R[1];
        fs["P1"] >> paras.P[0];
        fs["P2"] >> paras.P[1];
        fs.release();
    } else
        cout<<"Error: can not save the intrinsic parameters\n";
}
void bi_cam::undistortrectiry(const Mat& left_src,const Mat& right_src, const cam_paras& paras, Mat &left_rec, Mat &right_rec){
    Mat cameraMatrix[2], distCoeffs[2];
    Mat R1, R2, P1, P2, Q;
    Mat rmap[2][2];
    cameraMatrix[0] = paras.cameraMatrix[0];
    cameraMatrix[1] = paras.cameraMatrix[1];
    distCoeffs[0] = paras.distCoeffs[0];
    distCoeffs[1] = paras.distCoeffs[1];

    R1 = paras.R[0];
    R2 = paras.R[1];
    P1 = paras.P[0];
    P2 = paras.P[1];

    Size imageSize = left_src.size();

    //Precompute maps for cv::remap()
    fisheye::initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
    fisheye::initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

    bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));

    Mat canvas;
    double sf;
    int w, h;
    if( !isVerticalStereo )
    {
        sf = 600./MAX(imageSize.width, imageSize.height);
        w = cvRound(imageSize.width*sf);
        h = cvRound(imageSize.height*sf);
        canvas.create(h, w*2, CV_8UC3);
    }
    else
    {
        sf = 300./MAX(imageSize.width, imageSize.height);
        w = cvRound(imageSize.width*sf);
        h = cvRound(imageSize.height*sf);
        canvas.create(h*2, w, CV_8UC3);
    }
    Mat  cimg_l;
    remap(left_src, left_rec, rmap[0][0], rmap[0][1], INTER_LINEAR);
    cvtColor(left_rec, cimg_l, COLOR_GRAY2BGR);
    Mat canvasPart_l = !isVerticalStereo ? canvas(Rect(w*0, 0, w, h)) : canvas(Rect(0, h*0, w, h));
    resize(cimg_l, canvasPart_l, canvasPart_l.size(), 0, 0, INTER_AREA);

    Mat cimg_r;
    remap(right_src, right_rec, rmap[1][0], rmap[1][1], INTER_LINEAR);
    cvtColor(right_rec, cimg_r, COLOR_GRAY2BGR);
    Mat canvasPart_r = !isVerticalStereo ? canvas(Rect(w*1, 0, w, h)) : canvas(Rect(0, h*1, w, h));
    resize(cimg_r, canvasPart_r, canvasPart_r.size(), 0, 0, INTER_AREA);

    imshow("rectified", canvas);
}