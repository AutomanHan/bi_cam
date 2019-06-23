//
// Created by hanc on 18-12-19.
//
#include "pc_process.h"
#include <ctime>
//get point cloud
int pc_process::get_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,std::string pc_path, std::string ext) {
    if(ext == "pcd"){
        pcl::PCDReader reader;
        reader.read(pc_path, *cloud);
        if(cloud==NULL)
            return -1;
    }
    if(ext == "ply"){
        pcl::PLYReader reader;
        reader.read(pc_path, *cloud);
        if(cloud==NULL)
            return -1;
    }
    return 0;
}

int pc_process::get_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string pc_path) {
    if(pc_path.empty() || cloud==NULL){
        return -1;
        std::cout<<"cloud is empty or path is wrong"<<std::endl;
    }
    pcl::PolygonMesh mesh;
    //pcl::io::loadPolygonFileSTL(pc_path, mesh);
    pcl::io::loadPolygonFileOBJ(pc_path, mesh);
    pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
    if(cloud->points.size() == 0){
        std::cout<<"no file is loaded"<<std::endl;
    }
    else{
        std::cout<<"size of stl is: "<<cloud->points.size()<<std::endl;
    }
    return 0;
}

//preprocess point cloud
int pc_process::preprocess_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src,
                                      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sor(new pcl::PointCloud<pcl::PointXYZ>);

    if(cloud_src==NULL)
        return -1;
#if 1
    std::cout<<"src point cloud: width "<<cloud_src->width<<" height: "<<cloud_src->height<<std::endl;
#endif

#if 0
    //使用StatisticalOutlierRemoval濾波器移除outlier point
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_;
    sor_.setInputCloud(cloud_src);
    //sor_.setLeafSize(0.5f, 0.5f, 0.5f);
    sor_.setMeanK(50);
    sor_.setStddevMulThresh(0.1);

    //sor_.setNegative(true);
    sor_.filter(*cloud_sor);
#else
    *cloud_sor = *cloud_src;
#endif
#if 1
    std::cout<<"after StatisticalOutlierRemoval: width "<<cloud_sor->width<<" height: "<<cloud_sor->height<<std::endl;
#endif

    //DownSample
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud_sor);
    vg.setLeafSize(0.01f, 0.01f, 0.01f);
    vg.filter(*cloud_filter);

#if 1
    std::cout<<"after DownSample: width "<<cloud_filter->width<<" height: "<<cloud_filter->height<<std::endl;
#endif
    return 0;
}

//segment plane
int pc_process::seg_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr plane) {

    if(cloud_src == NULL)
        return -1;
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    //can change it(true/false)
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);

    seg.setDistanceThreshold(10);
    //seg.setProbability(0.95);

    //int i=0, nr_points = (int)cloud_src.points.size();
    //while(cloud_filter->poin)

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);

    int i=0, nr_points = (int)cloud_src->points.size();
    std::cout<<"source pointcloud size: "<<nr_points<<std::endl;
    *cloud_filter= *cloud_src;
    while(cloud_filter->points.size() > 0.3* nr_points){
        seg.setInputCloud(cloud_filter);
        seg.segment(*inliers, *coefficients);

        if(inliers->indices.size() ==0){
            PCL_ERROR("Could not estimate a plannar model for the given datset");
            break;
        }
        //extract the plannar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_src);
        extract.setIndices(inliers);
        extract.setNegative(false);

        //Get the points associated with the planar surface
        extract.filter(*cloud_plane);
        std::cout<<"PointCloud representing the planar component: "<< cloud_plane->points.size () << " data points." << std::endl;

        *plane = *plane + *cloud_plane;

        //remove the planar inliers, extract the rest
        //Extract the planar inliers from the input cloud
        extract.setNegative(true);
        extract.filter(*cloud_f);
        *cloud_filter = *cloud_f;
    }
#if 0
    //使用StatisticalOutlierRemoval濾波器移除outlier point
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_;
    sor_.setInputCloud(cloud_filter);
    //sor_.setLeafSize(0.5f, 0.5f, 0.5f);
    sor_.setMeanK(50);
    sor_.setStddevMulThresh(0.1);

    //sor_.setNegative(true);
    sor_.filter(*cloud_filter);
#endif

#if 1
    std::cout<<"after extract the planar: width "<<cloud_filter->width<<" height: "<<cloud_filter->height<<std::endl;
#endif

#if 0
    pcl::visualization::CloudViewer viewer("cloud_extract Viewer");
    viewer.showCloud(cloud_filter);
    while(!viewer.wasStopped());
#endif

    return 0;
}

//euclideancluster extract
int pc_process::ec_object(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src,
                          std::vector<pcl::PointCloud<pcl::PointXYZ> > &cc_vec) {

    if(cloud_src == NULL)
        return -1;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_src);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(20);
    ec.setMinClusterSize(1000);
    ec.setMaxClusterSize(100000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_src);
    ec.extract(cluster_indices);
    std::cout<<"cloud plane: "<<cluster_indices.size()<<std::endl;
    int i=0;
    for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it!=cluster_indices.end(); it++){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for(std::vector<int>::const_iterator pit = it->indices.begin(); pit!=it->indices.end(); pit++){
            cloud_cluster->points.push_back(cloud_src->points[*pit]);
        }
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        cc_vec.push_back(*cloud_cluster);

    }

    return 0;
}

//extract object
int pc_process::extract_object(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_object){
    std::vector<pcl::PointCloud<pcl::PointXYZ> > vec;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_extractor(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_passthrough(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::VectorXf plane_coefficient;
    seg_plane(cloud_src, cloud_extractor,cloud_plane);

    get_planemodel(cloud_plane, plane_coefficient);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
    Passthrough_pointcloud(cloud_extractor, cloud_f, plane_coefficient);


    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_object(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ center;
    MassCenter_pointcloud(cloud_f, center);
    ec_object(cloud_f, vec);
    extract_object(vec, center, cloud_object);
    return 0;
}


//icp registration
int pc_process::icp_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
                                 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target, Eigen::Matrix4d &trans_matrix) {
    if(cloud_in == NULL || cloud_target == NULL)
        return -1;
    int iterations=1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr final;
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaximumIterations(iterations);
    icp.setInputSource(cloud_in);
    icp.setInputTarget(cloud_target);
    icp.align(*final);

    if(icp.hasConverged()){
        std::cout<<"\nICP has converged, score is: "<<icp.getFitnessScore()<<std::endl;
        std::cout<<"\nICP transformation "<<iterations<<": cloud_in -> cloud_target"<<std::endl;
        trans_matrix = icp.getFinalTransformation().cast<double>();
        return 0;
    }
    return -1;
}

//save point cloud after cluster
int pc_process::save_pointcloud(std::vector<pcl::PointCloud<pcl::PointXYZ> > &cc_vec) {
    if(cc_vec.size()==0)
        return -1;
    int j=0;
    pcl::PCDWriter writer;
    for(int i=0; i<cc_vec.size(); i++){
        std::stringstream ss;
        ss<<"/home/hanc/code/bi_cam/save_data/cloud_cluster_"<<i<<".pcd";
        std::cout<< ss.str() <<std::endl;
        writer.write(ss.str(), cc_vec[i], false);
        std::cout<<cc_vec[i].size()<<std::endl;
    }
    return 0;
}
//save point cloud after cluster
int pc_process::save_pointcloud1(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud) {
    if(point_cloud ==NULL){
        return -1;
    }

    int j=0;
    pcl::PCDWriter writer;

        std::stringstream ss;
        ss<<"/home/hanc/code/bi_cam/data_src/gongjian_catch.pcd";
        std::cout<< ss.str() <<std::endl;

        writer.write(ss.str(), *point_cloud, false);
        std::cout<<point_cloud->points.size()<<std::endl;

    return 0;
}
//filter point cloud
int pc_process::smooth_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src,
                                  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_smooth) {
    if(cloud_src == NULL)
        return -1;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr search(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;

    search->setInputCloud(cloud_src);

    mls.setInputCloud(cloud_src);
    mls.setPolynomialOrder(4);
    mls.setSearchMethod(search);
    mls.setSearchRadius(50);
    mls.process(*cloud_smooth);

    return 0;
}

//get plane model coefficient
int pc_process::get_planemodel(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src,
                                       Eigen::VectorXf& model_coefficients) {

    if(cloud_src == NULL)
        return -1;
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud_src));
    int total_nums = cloud_src->points.size();
    std::vector<int> inliers;

    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);

    //std::cout<<"size of cloud_src: "<<cloud_src->points.size()<<std::endl;

    ransac.setDistanceThreshold(1);
    ransac.computeModel();
    ransac.getInliers(inliers);

    //std::cout<<"inliers: "<<inliers.size()<<std::endl;
    ransac.getModelCoefficients(model_coefficients);
    //model_p->m

    //model_p->computeModelCoefficients(index, model_coefficients);

    std::cout<<"a: "<<model_coefficients[0]<<" b: "<<model_coefficients[1]<<" c:"
    <<model_coefficients[2]<<" d:"<<model_coefficients[3]<<std::endl;
    //std::cout<<"model_coefficeients size: "<<model_coefficients.size()<<std::endl;
    return 0;
}

//passthrough filter for plannar normal
int pc_process::Passthrough_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src,
                                       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter,
                                       const Eigen::VectorXf &model_coefficients) {
    if(cloud_src == NULL)
        return -1;

    float A, B, C, D;
    A = model_coefficients[0];
    B = model_coefficients[1];
    C = model_coefficients[2];
    D = model_coefficients[3];
    float div_ABC = sqrt(pow(A,2) + pow(B,2) + pow(C,2));
    if(div_ABC == 0){
        std::cerr<<"div_ABC is negtivate"<<std::endl;
        return -1;
    }

    int nums = cloud_src->points.size();
    for(int i=0; i<nums; i++){
        float x, y, z;
        x = cloud_src->points[i].x;
        y = cloud_src->points[i].y;
        z = cloud_src->points[i].z;

        float mul = abs(A*x + B*y + C*z + D);
        float dis = mul / div_ABC;
        if(dis < 150){
            pcl::PointXYZ point;
            point.x = x;
            point.y = y;
            point.z = z;
            cloud_filter->push_back(point);
        }
    }
    return 0;
}


//passthrough filter for plannar normal
int pc_process::Passthrough_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src,
                                       const Eigen::VectorXf &model_coefficients) {
    if(cloud_src == NULL) return -1;

    float A, B, C, D;
    A = model_coefficients[0];
    B = model_coefficients[1];
    C = model_coefficients[2];
    D = model_coefficients[3];
    float div_ABC = sqrt(pow(A,2) + pow(B,2) + pow(C,2));
    if(div_ABC == 0){
        std::cerr<<"div_ABC is negtivate"<<std::endl;
        return -1;
    }

    pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud_src->begin();
    for(; it!=cloud_src->end(); ){
        float x, y, z;
        x = it->x;
        y = it->y;
        z = it->z;
        //x = cloud_src->points[i].x;
        //y = cloud_src->points[i].y;
        //z = cloud_src->points[i].z;
        float mul = abs(A*x + B*y + C*z + D);
        float dis = mul / div_ABC;
        if(dis > 2){
            if(dis < 25){
                float temp = (A*x+B*y+C*z+D)/(pow(A,2)+pow(B,2)+pow(C,2));
                it->x = x - A*temp;
                it->y = y - B*temp;
                it->z = z - C*temp;
                it++;
            }
            else
                it = cloud_src->erase(it);
        }
        else{
            it++;
        }
    }
}

//mass center for after filter
int pc_process::MassCenter_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src,
                pcl::PointXYZ& mass_center) {
    if(cloud_src == NULL)
        return -1;

    int nums = cloud_src->points.size();
    float sum_x=0.0, sum_y = 0.0, sum_z = 0.0;
    float center_x = 0.0, center_y = 0.0, center_z = 0.0;
    for(int i=0; i<nums; i++){
        sum_x += cloud_src->points[i].x;
        sum_y += cloud_src->points[i].y;
        sum_z += cloud_src->points[i].z;
    }
    center_x = sum_x / nums;
    center_y = sum_y / nums;
    center_z = sum_z / nums;

    mass_center.x = center_x;
    mass_center.y = center_y;
    mass_center.z = center_z;
    return 0;
}

int pc_process::Centroid_Normals(const std::vector<Point_Normal> &vec_nor, pcl::Normal &nor_center) {
    int nums = vec_nor.size();
    float sum_x=0.0, sum_y=0.0, sum_z=0.0;
    float center_x=0.0, center_y=0.0, center_z=0.0;
    for(int i=0; i<nums; i++){
        sum_x+=vec_nor[i].point_normal.normal_x;
        sum_y+=vec_nor[i].point_normal.normal_y;
        sum_z+=vec_nor[i].point_normal.normal_z;
    }
    center_x = sum_x/nums;
    center_y = sum_y/nums;
    center_z = sum_z/nums;

    nor_center.normal_x = center_x;
    nor_center.normal_y = center_y;
    nor_center.normal_z = center_z;
    return 0;
}

//select center object
int pc_process::extract_object(const std::vector<pcl::PointCloud<pcl::PointXYZ> > &vec,
        const pcl::PointXYZ& mass_center ,pcl::PointCloud<pcl::PointXYZ>::Ptr object_pc) {
    int nums = vec.size();
    if(nums == 0){
        std::cerr<<"vector is empty!"<<std::endl;
    }
    float dist_min = VTK_FLOAT_MAX;
    int index = 0;
    for(int i=0; i<nums; i++){
        pcl::PointXYZ center_object;
        float dist;
        pcl::PointCloud<pcl::PointXYZ> pc = vec[i];
        pcl::PointCloud<pcl::PointXYZ>::Ptr pc_Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        *pc_Ptr = pc;
        MassCenter_pointcloud(pc_Ptr, center_object);
        distance(mass_center, center_object, dist);
        if(dist< dist_min){
            std::cout<<"dist is: "<< dist<<"  "<<i<<std::endl;
            *object_pc = pc;
            dist_min =dist;
        }
    }
    return 0;
}

//region growing
int pc_process::RegionGrow_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src,
                                      pcl::RegionGrowing<POINT_T, pcl::Normal> &reg) {
    //kd tree
    if(cloud_src == NULL){
        std::cout<<"cloud_src is empty"<<std::endl;
        return -1;
    }

    std::cout<<"size of cloud_object: "<<cloud_src->points.size()<<std::endl;
    int nums_size = cloud_src->points.size();

    pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<POINT_T, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(cloud_src);
    normal_estimator.setKSearch(50);
    normal_estimator.compute(*normals);

    std::cout<<"normals are: "<<normals->points.size() <<std::endl;


    reg.setMinClusterSize(50);
    reg.setMaxClusterSize(nums_size/2);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(30);
    reg.setInputCloud(cloud_src);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(3.0/180.0 *M_PI);
    reg.setCurvatureThreshold(5.0);

    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);
    std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
    std::cout << "First cluster has " << clusters[0].indices.size () << " points." << endl;
    std::cout << "These are the indices of the points of the initial" <<
              std::endl << "cloud that belong to the first cluster:" << std::endl;

    std::cout<<"reg successfully"<<std::endl;
    return 0;

}

//normalization estimation
int pc_process::NormalEst_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals) {
    //create the normal estimation class, and pass the input dataset to it
    if(cloud_src == NULL){
        std::cout<<"cloud is empty!"<<std::endl;
        return -1;
    }
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud_src);

    //create an empty kdtree represtation, and pass it to the normal estimation object.
    //Its content will be filled inside the object, based on the given input dataset(as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
    tree->setInputCloud(cloud_src);
    ne.setSearchMethod(tree);

    //Output datasets
    //pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

    //Use all neighbours in a sphere of radius 3cm
    //ne.setRadiusSearch(10);
    ne.setKSearch(20);
    //compute the features
    ne.compute(*cloud_normals);
    return 0;
}
//k-means cluster based on normalization
int pc_process::KMeanNormal_pointcloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_src,
                                       pcl::PointCloud<pcl::Normal>::ConstPtr cloud_Normals,std::vector<Point_Normal>& vec_pn) {
    int center_nums = 3;

    PointCloudNormal2Point_Normal(cloud_src, cloud_Normals, vec_pn);
    int nums = vec_pn.size();
    std::vector<Point_Normal> vec_center(center_nums);
    std::vector<int> total_types(center_nums);
    std::vector<float> error_dist(center_nums);

    srand((int)time(0));

    for(int i=0; i<center_nums; i++){
        int cen =rand()%(nums+1);
        std::cout<<"random number: "<<cen<<std::endl;
        vec_center[i] = vec_pn[cen];
        std::cout<<"center mass x: "<<vec_center[i].point_xyz.x<<" y: "<<vec_center[i].point_xyz.y<<" z: "<<vec_center[i].point_xyz.z<<std::endl;
    }
    float oSSE = VTK_FLOAT_MAX;
    float nSSE = 0.0;
    int steps=0;
    std::cout<<"substract is: %f"<<fabs(oSSE-nSSE)<<std::endl;
    while(fabs(oSSE-nSSE)>0.01){
        steps++;
        oSSE = nSSE;
        nSSE=0;
        /*
        for(int i=0; i<center_nums; i++){
            vec_normals[i].clear();
        }
         */
        //label for all point cloud
        for(int i=0; i<nums; i++){
            float shorest = VTK_FLOAT_MAX;
            int cur=vec_pn[i].type;
            for(int j=0; j<center_nums; j++) {
                float temp;
                distance_normals(vec_pn[i], vec_center[j], temp);
                if (temp < shorest) {
                    shorest = temp;
                    cur = j;
                }
            }
            vec_pn[i].type=cur;
        }
        //centroid update
        for(int i=0; i<center_nums; i++){
            vec_center[i].point_normal.normal_x=0;
            vec_center[i].point_normal.normal_y=0;
            vec_center[i].point_normal.normal_z=0;
            total_types[i]=0;
            error_dist[i]=0;
        }
        for(int i=0; i<nums; i++){
            vec_center[vec_pn[i].type].point_normal.normal_x+=vec_pn[i].point_normal.normal_x;
            vec_center[vec_pn[i].type].point_normal.normal_y+=vec_pn[i].point_normal.normal_y;
            vec_center[vec_pn[i].type].point_normal.normal_z+=vec_pn[i].point_normal.normal_z;
            total_types[vec_pn[i].type]++;
        }
        for(int i=0; i<center_nums; i++){
            std::cout<<"number of type i is: "<<total_types[i]<<std::endl;
            vec_center[i].point_normal.normal_x /=total_types[i];
            vec_center[i].point_normal.normal_y /=total_types[i];
            vec_center[i].point_normal.normal_z /=total_types[i];
        }

        //calculate error
        float temp;
        for(int i=0; i<nums; i++){
            distance_normals(vec_pn[i], vec_center[vec_pn[i].type], temp);
            error_dist[vec_pn[i].type]+=temp;
        }
        for(int i=0; i<center_nums; i++){
            error_dist[i]/=total_types[i];
            nSSE+=error_dist[i];
        }
        std::cout<<"now error is: "<<fabs(oSSE-nSSE)<<std::endl;
    }
    std::cout<<"finish kmeans, total steps are: "<<steps<<std::endl;
    return 0;
}

//convert pointcloud+normals to struct PointNormal
int pc_process::PointCloudNormal2Point_Normal(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_src,
                                              pcl::PointCloud<pcl::Normal>::ConstPtr cloud_Normals,
                                              std::vector<Point_Normal> &pn) {
    if(cloud_src==NULL || cloud_Normals==NULL){
        std::cerr<<"point clouds or normals are empty!"<<std::endl;
        return -1;
    }
    int nums = cloud_src->points.size();
    for(int i=0; i<nums; i++){
        Point_Normal p;
        p.point_normal = cloud_Normals->points[i];
        p.point_xyz = cloud_src->points[i];
        p.type=0;
        pn.push_back(p);
    }
    return 0;
}

//convert struct PointNormal to pointcloud+normals
int pc_process::Point_Normal2PointCloud(std::vector<Point_Normal> &vec_pn,
                                       std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &point_cloud) {

    if(vec_pn.size()==0){
        std::cerr<<"vector is empty!"<<std::endl;
        return -1;
    }
    int nums = vec_pn.size();

    //point_cloud.resize(3);
    for(int i=0; i<nums; i++){
        //std::cout<<"type is: "<<vec_pn[i].type<<std::endl;
        point_cloud[vec_pn[i].type]->points.push_back(vec_pn[i].point_xyz);
    }
    return 0;
}

//filter each plannar(totally 3 plannars)
int pc_process::Filter_Plannar(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &vec_pointcloud) {
    if(vec_pointcloud.empty()) return -1;
    int nums = vec_pointcloud.size();
    for(int i=0; i<nums; i++){
        pcl::PointCloud<pcl::PointXYZ>::Ptr pc = vec_pointcloud[i];
        Eigen::VectorXf model_coefficients;

        get_planemodel(pc, model_coefficients);

        Passthrough_pointcloud(pc, model_coefficients);
    }
}

//calculate distance between two points
int pc_process::distance(const pcl::PointXYZ &center_1, const pcl::PointXYZ &center_2, float &dis) {
    float x1 = center_1.x, x2 = center_2.x;
    float y1 = center_1.y, y2 = center_2.y;
    float z1 = center_1.z, z2 = center_2.z;
    dis = sqrt(pow(x1-x2, 2)+pow(y1-y2, 2)+pow(z1-z2, 2));
    return 0;
}

//calculate distance between two normals
int pc_process::distance_normals(const Point_Normal p1, const Point_Normal p2, float &dis) {

    pcl::Normal normal1 = p1.point_normal;
    pcl::Normal normal2 = p2.point_normal;
    float nor_x1 = normal1.normal_x, nor_x2 = normal2.normal_x;
    float nor_y1 = normal1.normal_y, nor_y2 = normal2.normal_y;
    float nor_z1 = normal1.normal_z, nor_z2 = normal2.normal_z;
    dis = pow(nor_x1 - nor_x2,2) + pow(nor_y1-nor_y2, 2) + pow(nor_z1-nor_z2,2);
    return 0;
}

int pc_process::distance_normals(const pcl::Normal p1, const pcl::Normal p2, float &dis) {
    float nor_x1 = p1.normal_x, nor_x2 = p2.normal_x;
    float nor_y1 = p1.normal_y, nor_y2 = p2.normal_y;
    float nor_z1 = p1.normal_z, nor_z2 = p2.normal_z;
    dis = pow(nor_x1-nor_x2,2)+pow(nor_y1-nor_y2,2)+pow(nor_z1-nor_z2,2);
    return 0;
}