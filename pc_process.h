//
// Created by hanc on 18-12-19.
//

#ifndef __PC_PROCESS_H__
#define __PC_PROCESS_H__

#include <string>
#include <iostream>
#include <vector>
#include <math.h>
#include <cstdlib>


#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/passthrough.h>
//#include <pcl/filters/>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_normal_plane.h>


#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/ModelCoefficients.h>
//#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>

#include <pcl/features/normal_3d.h>

//#include <boost/shared_ptr.hpp>

typedef pcl::PointXYZ POINT_T;

typedef struct Point_Normal{
    pcl::PointXYZ point_xyz;
    pcl::Normal point_normal;
    int type;
}Point_Normal;


class pc_process{
public:
    //get point cloud from disk
    int get_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,std::string pc_path, std::string ext);

    //preprocess point cloud(such filter)
    int preprocess_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter);

    //segment plane
    int seg_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter, pcl::PointCloud<pcl::PointXYZ>::Ptr plane);

    //euclidean cluster extract
    int ec_object(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src, std::vector<pcl::PointCloud<pcl::PointXYZ> > &cc_vec);

    //select center object
    int extract_object(const std::vector<pcl::PointCloud<pcl::PointXYZ> > &vec,
            const pcl::PointXYZ& mass_center ,pcl::PointCloud<pcl::PointXYZ>::Ptr object_pc);

    //icp registration
    int icp_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target, Eigen::Matrix4d &trans_matrix);

    //save point cloud after cluster
    int save_pointcloud(std::vector<pcl::PointCloud<pcl::PointXYZ> > &cc_vec);
    int save_pointcloud1(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);

    //filter point cloud
    int smooth_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_smooth);

    //get plane model coefficient
    int get_planemodel(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src, Eigen::VectorXf& model_coefficients);

    //passthrough filter for plannar normal
    int Passthrough_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src,
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter, const Eigen::VectorXf& model_coefficients);

    //mass center for after filter
    int MassCenter_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src, pcl::PointXYZ& mass_center);
    int Centroid_Normals(const std::vector<Point_Normal> &vec_nor, pcl::Normal& nor_center);


    //region growing
    int RegionGrow_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src, pcl::RegionGrowing<POINT_T, pcl::Normal> &reg);

    //normalization estimation
    int NormalEst_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals);

    //k-means cluster based on normalization
    int KMeanNormal_pointcloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_src,
            pcl::PointCloud<pcl::Normal>::ConstPtr cloud_Normals,std::vector<Point_Normal> &vec_pn);

    //convert pointcloud+normals to struct PointNormal
    int PointCloudNormal2Point_Normal(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_src, pcl::PointCloud<pcl::Normal>::ConstPtr cloud_Normals,
            std::vector<Point_Normal> &pn);
    //convert struct PointNormal to pointcloud+normals
    int Point_Normal2PointCloud(std::vector<Point_Normal>& vec_pn, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &point_cloud);

public:
    //calculate distance between two points
    static int distance(const pcl::PointXYZ& center_1, const pcl::PointXYZ& center_2, float& dis);

    //calculate distance between two normals
    static int distance_normals(const Point_Normal p1, const Point_Normal p2,float &dis);

    static int distance_normals(const pcl::Normal p1, const pcl::Normal p2, float &dis);
};
#endif //BI_CAM_PC_PROCESS_H
