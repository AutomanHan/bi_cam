#include <iostream>
//#include <opencv2/highgui.hpp>
//#include "opencv2/core.hpp"
//#include "opencv2/imgproc.hpp"
//#include "mynteye/api/api.h"
//#include "bi_cam.h"
#include "pc_process.h"

using namespace std;
//MYNTEYE_USE_NAMESPACE
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
    viewer->setBackgroundColor(1.0,0.5,1.0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "simple cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "simple cloud");
    viewer->initCameraParameters ();
    return (viewer);
}
boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis (
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
{
    //ŽŽœš3DŽ°¿Ú²¢ÌíŒÓµãÔÆÆä°üÀš·šÏß
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0.7);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> normals_color_handler (cloud, 0,255,0);
    //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZ> rgb(cloud);
    //viewer->addPointCloud<pcl::PointXYZ> (cloud, rgb, "sample cloud");
    //viewer->addPointCloud<pcl::PointXYZ> (cloud, "simple cloud");
    viewer->addPointCloud<pcl::PointXYZ>(cloud, normals_color_handler, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, normals, 20, 20, "normals");

    //viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud1");
    //viewer->addPointCloud<pcl::PointXYZ> (cloud, normals_color_handler, "sample cloud1");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    return (viewer);
}
boost::shared_ptr<pcl::visualization::PCLVisualizer> kmeans_vis(
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> vec_kmeans_pc){
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (1.0, 0.5, 1.0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pc_colorhandler0(vec_kmeans_pc[0], 255,0,0);
    viewer->addPointCloud<pcl::PointXYZ>(vec_kmeans_pc[0], pc_colorhandler0, "pc0");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pc_colorhandler1(vec_kmeans_pc[1], 0,255,0);
    viewer->addPointCloud<pcl::PointXYZ>(vec_kmeans_pc[1], pc_colorhandler1, "pc1");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pc_colorhandler2(vec_kmeans_pc[2], 0,0,255);
    viewer->addPointCloud<pcl::PointXYZ>(vec_kmeans_pc[2], pc_colorhandler2, "pc2");

    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    return(viewer);
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
#if 0
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
    //auto pc = PointCloud<PointXYZ>::Ptr (new PointCloud<PointXYZ>);
    //auto cloud = PointCloud<PointXYZ>::Ptr (pc);
    api->Start(Source::VIDEO_STREAMING);
#endif

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
#if 1
    // point cloud process
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    //viewer = simpleVis(cloud);
    //boost::thread vthread(&viewerRunner,viewer);


    pc_process pc_pro;
    std::string ply_path = "/home/hanc/code/bi_cam/data_src/PointCloud/pcd_2.ply", ext = "ply";

    pc_pro.get_pointcloud(cloud, ply_path, ext);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZ>);
    pc_pro.preprocess_pointcloud(cloud, cloud_filter);

    std::vector<pcl::PointCloud<pcl::PointXYZ> > vec;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_extractor(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_passthrough(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::VectorXf plane_coefficient;
    pc_pro.seg_plane(cloud_filter, cloud_extractor,cloud_plane);

    pc_pro.get_planemodel(cloud_plane, plane_coefficient);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
    pc_pro.Passthrough_pointcloud(cloud_extractor, cloud_f, plane_coefficient);


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_object(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ center;
    pc_pro.MassCenter_pointcloud(cloud_f, center);
    pc_pro.ec_object(cloud_f, vec);
    pc_pro.extract_object(vec, center, cloud_object);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_color;
    pcl::RegionGrowing<POINT_T, pcl::Normal> reg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_smooth(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pc_pro.smooth_pointcloud(cloud_object, cloud_smooth);

    pc_pro.NormalEst_pointcloud(cloud_smooth, cloud_normals);
    //pc_pro.save_pointcloud1(cloud_smooth);
    if(cloud_normals == NULL)
        std::cout<<"normals are empty"<<std::endl;
    else
        std::cout<<"normals have been finished"<<std::endl;



    //kmeans
    std::vector<Point_Normal> vec_pn;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> vec_kmeans_pc;
    vec_kmeans_pc.resize(3);
    pcl::PointCloud<pcl::PointXYZ>::Ptr kmeans_pc0(new pcl::PointCloud<pcl::PointXYZ>);
    vec_kmeans_pc[0]=kmeans_pc0;

    pcl::PointCloud<pcl::PointXYZ>::Ptr kmeans_pc1(new pcl::PointCloud<pcl::PointXYZ>);
    vec_kmeans_pc[1]=kmeans_pc1;

    pcl::PointCloud<pcl::PointXYZ>::Ptr kmeans_pc2(new pcl::PointCloud<pcl::PointXYZ>);
    vec_kmeans_pc[2]=kmeans_pc2;

    pc_pro.KMeanNormal_pointcloud(cloud_smooth, cloud_normals,vec_pn);
    std::cout<<"kmeans have been finished"<<std::endl;
    pc_pro.Point_Normal2PointCloud(vec_pn, vec_kmeans_pc);
    std::cout<<"size of vector after kmeans:"<<vec_kmeans_pc.size()<<std::endl;

    //pc_pro.RegionGrow_pointcloud(cloud_smooth, reg);
    //cloud_color = reg.getColoredCloud();
    //std::cout<<"size of color cloud: "<<cloud_color->points.size()<<std::endl;

#if 0
    pcl::visualization::CloudViewer vie("cluster viewer");
    vie.showCloud(cloud_color);
    while(!vie.wasStopped()){

    }
#endif
    //pc_pro.save_pointcloud(vec);

#endif
#if 0
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_smooth(new pcl::PointCloud<pcl::PointXYZ>);

    pc_process pc_pro;
    std::string ply_path = "/home/hanc/code/bi_cam/save_data/cloud_cluster_1.pcd", ext = "pcd";

    pc_pro.get_pointcloud(cloud, ply_path, ext);
    pc_pro.smooth_pointcloud(cloud, cloud_smooth);
#endif

    pcl::PointCloud<pcl::PointXYZ>::Ptr center_mass(new pcl::PointCloud<pcl::PointXYZ>);
    center_mass->points.push_back(center);
    //std::cout<<"center mass x: "<<center.x<<" y: "<<center.y<<" z: "<<center.z<<std::endl;
    //std::cout<<"size of point cloud: "<<center_mass->points.size()<<std::endl;
    //std::cout<<"size of cloud normals: "<<cloud_normals->size()<<std::endl;

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler (center_mass, 0, 255, 0);

#if 1
    //viewer = simpleVis(cloud_f);
    //viewer=simpleVis(cloud_smooth);
    //viewer = normalsVis(cloud_smooth, cloud_normals);
    viewer = kmeans_vis(vec_kmeans_pc);
    //viewer = simpleVis(cloud_color);
    //viewer->addPointCloud<pcl::PointXYZ> (center_mass, keypoints_color_handler, "keypoints");
    //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");
    //viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud_smooth, cloud_normals, 10, 0.5,"simple cloud");
    //viewer->addPointCloud<pcl::Normal> (cloud_normals, normals_color_handler, "normals");
    //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 3, "normals");
    //viewer.showCloud(cloud_f);
    //viewer.runOnVisualizationThreadOnce(viewerOneOff);

    while(!viewer->wasStopped()){
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
#endif

    //while (true) {
        /*
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
            //imwrite("depth.jpg", depth_16U);


            for(size_t i=0; i<pc->points.size(); ++i){
                cout<<" "<<pc->points[i].x<<" "<<pc->points[i].y<<" "<<pc->points[i].z<<endl;
            }

            boost::mutex::scoped_lock updateLock(updateModelMutex);
            viewer->updatePointCloud<pcl::PointXYZ>(pc,"sample cloud");
            updateLock.unlock();
            boost::this_thread::sleep (boost::posix_time::microseconds (100));


            //cloud = &pc;

            char buf[233];
            if (frame != 0)
            {
                sprintf(buf, "frame%d", frame);
                viewer.removePointCloud(buf);
            }
            sprintf(buf, "frame%d", ++frame);
            viewer.addPointCloud(pc, buf);
            viewer.spinOnce(1000);


            //viewer.showCloud(pc);
            //viewer.runOnVisualizationThreadOnce (viewerOneOff);
            //viewer.runOnVisualizationThread (viewerPsycho);

            //while (!viewer.wasStopped ())
            //{
            //    user_data++;
            //}
            //cout<<"show"<<endl;

            imshow("disparity", dis_8u);*/
            //viewer.runOnVisualizationThreadOnce (viewerOneOff);
            //while (!viewer.wasStopped ());
            //cv::waitKey(0);
        //}

        //char key = static_cast<char>(cv::waitKey(1));
        //if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
        //    break;
        //}
    //}
    //vthread.join();
    return 0;
}