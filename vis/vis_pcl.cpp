#include "vis_pcl.h"


//visualize point cloud with PointXYZ
boost::shared_ptr<pcl::visualization::PCLVisualizer> vis_pcl::simpleVis(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
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

//visualize point cloud with normals
boost::shared_ptr<pcl::visualization::PCLVisualizer> vis_pcl::normalsVis (
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals){
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

//visualize a vector of point clouds
boost::shared_ptr<pcl::visualization::PCLVisualizer> kmeans_vis(
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> vec_kmeans_pc){
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (1.0, 0.5, 1.0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pc_colorhandler0(vec_kmeans_pc[0], 255,0,0);
    viewer->addPointCloud<pcl::PointXYZ>(vec_kmeans_pc[0], pc_colorhandler0, "pc0");
    std::cout<<"red points's number is: "<< vec_kmeans_pc[0]->points.size()<<std::endl;
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pc_colorhandler1(vec_kmeans_pc[1], 0,255,0);
    viewer->addPointCloud<pcl::PointXYZ>(vec_kmeans_pc[1], pc_colorhandler1, "pc1");
    std::cout<<"green points's number is: "<< vec_kmeans_pc[1]->points.size()<<std::endl;
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pc_colorhandler2(vec_kmeans_pc[2], 0,0,255);
    viewer->addPointCloud<pcl::PointXYZ>(vec_kmeans_pc[2], pc_colorhandler2, "pc2");
    std::cout<<"blue points's number is: "<< vec_kmeans_pc[2]->points.size()<<std::endl;

    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    return(viewer);
}