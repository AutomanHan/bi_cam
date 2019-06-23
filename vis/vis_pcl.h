#ifndef __VIS_PCL_H__
#define __VIS_PCL_H__

#include "pcl-h.h"
#include <iostream>
class vis_pcl{
    //stand a simple object of PCLVisualizer

    //visualize point cloud with PointXYZ
    boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
    //visualize point cloud with normals
    boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis (
            pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals);
    //visualize a vector of point clouds
    boost::shared_ptr<pcl::visualization::PCLVisualizer> kmeans_vis(
            std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> vec_kmeans_pc);
};

#endif