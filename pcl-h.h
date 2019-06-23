#ifndef __PCL-H__H__
#define __PCL-H__H__

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>

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


#endif