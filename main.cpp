#include <iostream>
#include <opencv2/highgui.hpp>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "mynteye/api/api.h"
#include "bi_cam.h"

using namespace std;
MYNTEYE_USE_NAMESPACE
using namespace pcl;
//using namespace pcl;
int user_data;
const double u0 = 319.52883;
const double v0 = 271.61749;
const double fx = 528.57523;
const double fy = 527.57387;

void viewerOneOff (pcl::visualization::PCLVisualizer& viewer) {
    viewer.setBackgroundColor(1.0, 0.5, 1.0);
    pcl::PointXYZ o;
    o.x = 1.0;
    o.y = 0;
    o.z = 0;
    //viewer.addSphere (o, 0.25, "sphere", 0);
    std::cout << "i only run once" << std::endl;
}

void
viewerPsycho (pcl::visualization::PCLVisualizer& viewer){
    static unsigned count = 0;
    std::stringstream ss;
    ss << "Once per viewer loop: " << count++;
    viewer.removeShape ("text", 0);
    viewer.addText (ss.str(), 200, 300, "text", 0);

    //FIXME: possible race condition here:
    user_data++;}


//show points cloud stram
boost::mutex updateModelMutex;

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setCameraPosition(0, 0, 0, 0, 0, 0, 0, 0, -1);
    viewer->setBackgroundColor(0,0,0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->initCameraParameters ();
    return (viewer);
}

void viewerRunner(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100));
    }
}

int main() {
    int frame = 0;
    bi_cam bi;
    int w=9, h=6;
    float  s=33.0;
    string filename = "stereo_calib.xml";
    string save_path = "../imglist/";
    int gain=2;
    int brightness=50;
    int contrast=140;

    string intrinsics_file = "intrinsics.yml";
    string extrinsics_file = "extrinsics.yml";
    cam_paras paras;
    bi.get_ymlfile(intrinsics_file, extrinsics_file, paras);

    //cout<<paras.cameraMatrix[0]<<endl;
    auto &&api = API::Create();
    api->SetOptionValue(Option::GAIN, gain);
    api->SetOptionValue(Option::BRIGHTNESS, brightness);
    api->SetOptionValue(Option::CONTRAST, contrast);

    LOG(INFO) << "Enable manual-exposure";
    LOG(INFO) << "Set GAIN to " << api->GetOptionValue(Option::GAIN);
    LOG(INFO) << "Set BRIGHTNESS to " << api->GetOptionValue(Option::BRIGHTNESS);
    LOG(INFO) << "Set CONTRAST to " << api->GetOptionValue(Option::CONTRAST);
    cout<<"matrix 0: "<<paras.cameraMatrix[0].at<float>(0) <<endl;
    //PointCloud<PointXYZ> pc;
    auto pc = PointCloud<PointXYZ>::Ptr (new PointCloud<PointXYZ>);
    //auto cloud = PointCloud<PointXYZ>::Ptr (pc);
    api->Start(Source::VIDEO_STREAMING);

    //pcl show points cloud
    pcl::visualization::PCLVisualizer viewer("cloud");
    //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    //viewer = simpleVis(pc);
    //boost::thread vthread(&viewerRunner,viewer);
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
            Mat depth_16U = Mat(left_rec.rows, left_rec.cols, CV_16UC1);
            //imshow("left_rec", left_rec);
            bi.get_disparity(left_rec, right_rec, dis_16s, dis_8u);
            bi.get_depth(dis_16s, depth_16U, paras.cameraMatrix[0]);
            bi.get_point(depth_16U, pc, paras.cameraMatrix[0]);
            //FileStorage fs("depth.")
            imwrite("depth.jpg", depth_16U);

            /*
            for(size_t i=0; i<pc->points.size(); ++i){
                cout<<" "<<pc->points[i].x<<" "<<pc->points[i].y<<" "<<pc->points[i].z<<endl;
            }
             */


            /*
            boost::mutex::scoped_lock updateLock(updateModelMutex);
            viewer->updatePointCloud<pcl::PointXYZ>(pc,"sample cloud");
            updateLock.unlock();
            boost::this_thread::sleep (boost::posix_time::microseconds (100));
            */

            //cloud = &pc;
            char buf[233];
            if (frame != 0)
            {
                sprintf(buf, "frame%d", frame);
                viewer.removePointCloud(buf);
            }
            sprintf(buf, "frame%d", ++frame);
            viewer.addPointCloud(pc, buf);
            viewer.spinOnce();


            //viewer.showCloud(pc);
            //viewer.runOnVisualizationThreadOnce (viewerOneOff);
            //viewer.runOnVisualizationThread (viewerPsycho);

            //while (!viewer.wasStopped ())
            //{
            //    user_data++;
            //}
            //cout<<"show"<<endl;

            imshow("disparity", dis_8u);
            //viewer.runOnVisualizationThreadOnce (viewerOneOff);
            //while (!viewer.wasStopped ());
            //cv::waitKey(0);
        }
        char key = static_cast<char>(cv::waitKey(1));
        if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
            break;
        }
    }
    //vthread.join();
    return 0;
}