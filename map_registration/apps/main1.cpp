/**
 * Created by js.yang on 30/8/2019 for map registration step by step
 */



#include <math.h>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <string>
#include <vector>
#include <iomanip>
#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common_headers.h>
#include <iostream>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/console/time.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
//#include <boost/thread/thread.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>


#include <time.h>
//Bilateral Filter
#include <pcl/filters/bilateral.h>//required
#include <pcl/filters/fast_bilateral.h>
#include <pcl/filters/fast_bilateral_omp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>

#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/ndt.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <mutex>
#include <thread>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
//#include <pclomp/ndt_omp.h>
//ros::NodeHandle nh;
//ros::Publisher point_chooseed_publisher1;
#include <tic_toc.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/registration/ndt.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/icp_nl.h>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef pcl::visualization::PCLVisualizer Visualizer;

int vp_1, vp_2;
int index_g;
int num=0;//
PointCloud::Ptr cloud_tmp(new PointCloud);
PointCloud::Ptr target(new PointCloud);
PointCloud::Ptr source(new PointCloud);

bool flag=true;
boost::mutex cloud_mutex;

Eigen::Matrix4d FinalTransform;

std::stringstream ss;
std::string cloudName;

/*****************************/

typedef pcl::PointNormal PointNormal;
typedef pcl::PointCloud<PointNormal> PointCloudWithNormal;

typedef pcl::FPFHSignature33 FPFHT;
typedef pcl::PointCloud<FPFHT> FPFHCloud;
const float VOXEL_GRID_SIZE = 0.03;
const double radius_normal=20;
const double radius_feature=30;
const double max_sacis_iteration=1000;
const double min_correspondence_dist=0.01;
const double max_correspondence_dist=1000;
using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

pcl::PointCloud<pcl::Normal>::Ptr source_normal (new pcl::PointCloud<pcl::Normal>);
pcl::PointCloud<pcl::Normal>::Ptr target_normal (new pcl::PointCloud<pcl::Normal>);

std::string map_path;
int num_map;


pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point> ());

/*****************************/



struct cloud_choosed
{
    cloud_choosed() : cloud (new PointCloud){};

    cloud_choosed(std::string name,std::vector<int> tmp_vector,int num_neibor=0,int this_index=0) :
            cloud (new PointCloud),viewer(new Visualizer(name)),num_of_neighbors(num_neibor),
            neigbor_id(tmp_vector),this_index(this_index),Transform(Eigen::Matrix4d::Identity())
    {};

    int index()
    {
        return static_cast<int>(this_index);
    }
    Eigen::Matrix4d Transform;
    int num_of_neighbors;
    int this_index;
    std::vector<int> neigbor_id;
    PointCloud::Ptr cloud;
    std::string map_name;///

    std::multimap<int,PointCloud::Ptr> multi_cloud_choosed;///
    std::map<int,PointCloud::Ptr> neigh_cloud_choosed;
    boost::shared_ptr<Visualizer> viewer;

};
struct cloud_choosed* cloud_choosed_ptr;


int id,count;
int pre_id=INT_MAX,cur_id;
PointCloud::Ptr cloud_temp(new PointCloud);
void ppi_callback(const pcl::visualization::AreaPickingEvent &event, void *args)
{
        if(index_g==cloud_choosed_ptr[index_g].neigbor_id[count])
            return;
        id=cloud_choosed_ptr[index_g].neigbor_id[count];
        std::cout << "id:" << id << std::endl;
        cur_id = id;
        if (0 ==
            std::count(cloud_choosed_ptr[index_g].neigbor_id.begin(), cloud_choosed_ptr[index_g].neigbor_id.end(), id))
            PCL_ERROR("The two clouds are not neighbor!\n");

        PointCloud::Ptr cloud(new PointCloud);
        PointCloud::Ptr cloud_temp1(new PointCloud);
        std::vector<int> indices;
        if (event.getPointsIndices(indices) == -1)
            return;

        for (int i = 0; i < indices.size(); ++i)
        {
            cloud->points.push_back(cloud_choosed_ptr[index_g].cloud->points.at(indices[i]));
        }

        if (pre_id == INT_MAX || pre_id == cur_id) {
            *cloud_temp = (*cloud_temp) + (*cloud);
            pre_id = cur_id;
        }
        if (cur_id != pre_id) {
            cloud_temp->clear();
            *cloud_temp = (*cloud_temp) + (*cloud);
            pre_id = cur_id;
        }
        *cloud_temp1 = *cloud_temp;

        cloud_choosed_ptr[index_g].neigh_cloud_choosed[id] = cloud_temp1;
        std::cout << "size of choosed point :" << cloud->size() << std::endl;
        std::cout << "all size of choosed point :" << cloud_choosed_ptr[index_g].neigh_cloud_choosed[id]->size() << std::endl;


        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(
                cloud_choosed_ptr[index_g].neigh_cloud_choosed[id], 255, 0, 0);
        ss << num++;
        ss >> cloudName;
        cloudName += "_cloudName";

        cloud_choosed_ptr[index_g].viewer->addPointCloud(cloud_choosed_ptr[index_g].neigh_cloud_choosed[id], red, cloudName);
        cloud_choosed_ptr[index_g].viewer->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, cloudName);
        count++;

}
void areai_choose(const int index)
{
    count=1;
    cloud_temp->clear();
    std::cout<<"current cloud ID:"<<cloud_choosed_ptr[index].index()<<std::endl;
    for(int i=0;i<cloud_choosed_ptr[index].neigbor_id.size();i++)
        std::cout<<"neighbor id:"<<cloud_choosed_ptr[index].neigbor_id[i]<<std::endl;

    cloud_choosed_ptr[index].viewer->addPointCloud(cloud_choosed_ptr[index].cloud, std::to_string(index));
    cloud_choosed_ptr[index].viewer->addCoordinateSystem(100.0);
    cloud_choosed_ptr[index].viewer->setCameraPosition(0, 0, -2, -1, 0, 0);
    cloud_choosed_ptr[index].viewer->registerAreaPickingCallback(ppi_callback, (void *)&cloud_choosed_ptr[index].cloud);
    cloud_choosed_ptr[index].viewer->spin();

    std::cout<<"before removePointCloud:"<<index<<std::endl;
    cloud_choosed_ptr[index].viewer->removePointCloud(std::to_string(index));
    cloud_choosed_ptr[index].viewer->removePointCloud(cloudName);
    std::cout<<" removePointCloud index end:"<<index<<std::endl;


}





PointCloud::Ptr filter(PointCloud::Ptr cloud_in)
{
    PointCloud::Ptr filtered_cloud(new PointCloud);
    pcl::ApproximateVoxelGrid<Point> approximate_voxel_filter;
    approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);
    approximate_voxel_filter.setInputCloud(cloud_in);
    approximate_voxel_filter.filter(*filtered_cloud);
    return filtered_cloud;
}


void help()
{
    printf("*********************************************\n");
    printf("This is the code which proform registration between source PointCloud and map \n"
                   "Press 'x' to choose interested area to registrate \n"
                   "Press 'q' to quit visualizer and continue!\n");
    printf("*********************************************\n\n\n");
}


/***********************FPFH start***********************/





pcl::PointCloud<pcl::Normal>::Ptr getNormals(PointCloud::Ptr cloud, double radius)
{
    pcl::PointCloud<pcl::Normal>::Ptr normalsPtr (new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<Point,pcl::Normal> norm_est;
    norm_est.setInputCloud(cloud);
    //nonicp
    // norm_est.setRadiusSearch(radius);
    //norm_est.compute(*normalsPtr);
    //icp
    norm_est.setSearchMethod(tree);
    norm_est.setKSearch(radius);
    norm_est.compute(*normalsPtr);
    return normalsPtr;

}

FPFHCloud::Ptr getFeatures(PointCloud::Ptr cloud,pcl::PointCloud<pcl::Normal>::Ptr normals,double radius)
{
    FPFHCloud::Ptr features (new FPFHCloud);
    //nonicp
//    pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point>);
//    pcl::FPFHEstimation<Point,pcl::Normal,FPFHT> fpfh_est;
//    fpfh_est.setInputCloud(cloud);
//    fpfh_est.setInputNormals(normals);
//    fpfh_est.setSearchMethod(tree);
//    fpfh_est.setRadiusSearch(radius);
//    fpfh_est.compute(*features);

    //icp
    pcl::FPFHEstimationOMP<Point,pcl::Normal,FPFHT> fpfh_est;
    fpfh_est.setNumberOfThreads(8);
    fpfh_est.setInputCloud(cloud);
    fpfh_est.setInputNormals(normals);
    fpfh_est.setSearchMethod(tree);
    fpfh_est.setKSearch(radius);
    // fpfh_est.setRadiusSearch(radius);
    fpfh_est.compute(*features);

    return features;
}

Eigen::Matrix4f sac_ia_align(PointCloud::Ptr source,PointCloud::Ptr target,
                             FPFHCloud::Ptr source_feature,FPFHCloud::Ptr target_feature,
                             int max_sacia_iterations,double min_correspondence_dist,double max_correspondence_dist)
{
    pcl::SampleConsensusInitialAlignment<Point,Point,FPFHT> sac_ia;
    Eigen::Matrix4f final_transformation;
    sac_ia.setInputSource(target);
    sac_ia.setSourceFeatures(target_feature);
    sac_ia.setInputTarget(source);
    sac_ia.setTargetFeatures(source_feature);
    sac_ia.setMaximumIterations(max_sacia_iterations);
    sac_ia.setMinSampleDistance(min_correspondence_dist);
    sac_ia.setMaxCorrespondenceDistance(max_correspondence_dist);
    PointCloud::Ptr finalcloud (new PointCloud);
    TicToc tt;
    sac_ia.align(*finalcloud);
    cout<<"Finished SAC_IA Initial Regisration in "<<tt.toc()<<"ms"<<endl;
    final_transformation=sac_ia.getFinalTransformation();
    return final_transformation;

}
//左右显示变换前后点云对
void viewPair(PointCloud::Ptr cloud1, PointCloud::Ptr cloud2,
              PointCloud::Ptr cloud1al, PointCloud::Ptr cloud2al)
{

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    //pcl::visualization::PCLVisualizer viewer("3D viewer");
    viewer->initCameraParameters();
    int v1(0), v2(0);
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor(0, 0, 0, v1);
    viewer->addText("Before Alignment", 10, 10, "v1 text", v1);
    PointCloudColorHandlerCustom<Point> green(cloud1, 0, 255, 0);
    PointCloudColorHandlerCustom<Point> red(cloud2, 255, 0, 0);
    viewer->addPointCloud(cloud1, green, "v1_target", v1);
    viewer->addPointCloud(cloud2, red, "v1_sourse", v1);

    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor(0, 0, 0, v2);
    viewer->addText("After Alignment", 10, 10, "v2 text", v2);
    PointCloudColorHandlerCustom<Point> green2(cloud1al, 0, 255, 0);
    PointCloudColorHandlerCustom<Point> red2(cloud2al, 255, 0, 0);
    viewer->addPointCloud(cloud1al, green2, "v2_target", v2);
    viewer->addPointCloud(cloud2al, red2, "v2_sourse", v2);
    viewer->spin();

}

/***********************FPFH end***********************/



void sac_ia_align(PointCloud::Ptr source,PointCloud::Ptr target,PointCloud::Ptr finalcloud,Eigen::Matrix4f init_transform,
                  int max_sacia_iterations,double min_correspondence_dist,double max_correspondence_dist)
{
    //todo sac_ia_align(source,target,init_result,init_transform

    std::vector<int> indices1;
    std::vector<int> indices2;
    PointCloud::Ptr sourceds (new PointCloud);
    PointCloud::Ptr targetds (new PointCloud);
    pcl::removeNaNFromPointCloud(*source,*source,indices1);
    pcl::removeNaNFromPointCloud(*target,*target,indices2);
//降采样
    //  voxelFilter(source,sourceds,VOXEL_GRID_SIZE);
    //  voxelFilter(target,targetds,VOXEL_GRID_SIZE);
    sourceds=filter(source);
    targetds=filter(target);
    cout<<"1"<<endl;
//计算法向量
    pcl::PointCloud<pcl::Normal>::Ptr source_normal (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr target_normal (new pcl::PointCloud<pcl::Normal>);
    source_normal=getNormals(sourceds,radius_normal);
    target_normal=getNormals(targetds,radius_normal);
    cout<<"2"<<endl;
//计算FPFH特征
    FPFHCloud::Ptr source_feature (new FPFHCloud);
    FPFHCloud::Ptr target_feature (new FPFHCloud);
    source_feature=getFeatures(sourceds,source_normal,radius_feature);
    target_feature=getFeatures(targetds,target_normal,radius_feature);
    cout<<"3"<<endl;
//SAC-IA配准
    pcl::SampleConsensusInitialAlignment<Point,Point,FPFHT> sac_ia;
    Eigen::Matrix4f final_transformation;
    sac_ia.setInputSource(sourceds);
    sac_ia.setSourceFeatures(source_feature);
    sac_ia.setInputTarget(targetds);
    sac_ia.setTargetFeatures(target_feature);
    sac_ia.setMaximumIterations(max_sacia_iterations);
    sac_ia.setMinSampleDistance(min_correspondence_dist);
    sac_ia.setMaxCorrespondenceDistance(max_correspondence_dist);

    PointCloud::Ptr output (new PointCloud);
    TicToc tt;
    sac_ia.align(*output);
    cout<<"Finished SAC_IA Initial Regisration in "<<tt.toc()<<"ms\n"<<endl;
    init_transform=sac_ia.getFinalTransformation();
    pcl::transformPointCloud(*source,*finalcloud,init_transform);
}


bool loadData (ros::NodeHandle &n){
    std::string config_file;
    if(n.getParam("config_file",config_file))
    {
       // config_file = n.getParam("config_file", config_file);
        ROS_INFO_STREAM("Loaded "<<"config_file:"<< config_file);
    } else
    {
        ROS_ERROR_STREAM("Failed to loaded config file!");
        n.shutdown();
        return false;
    }
    cv::FileStorage fsSetting(config_file,cv::FileStorage::READ);
    if(!fsSetting.isOpened())
        std::cerr<<"ERROR: Wrong path to setting\n";

    fsSetting["map_path"] >> map_path;
    fsSetting["num"]>>num_map;

    std::string extension (".pcd");

    struct cloud_choosed cloud_choosed_tmp[num_map];
    int num_neighbor;
    int neighbor_id;

    cloud_choosed_ptr=cloud_choosed_tmp;
    // Suppose the first argument is the actual test model
    for (int i = 0; i < num_map; i++)
    {
        TicToc tt;
        std::string map_name = map_path + std::to_string (i+1)+".pcd";
        if (map_name.size () <= extension.size ())
            continue;
        cv::Mat ID;
        typedef Eigen::Matrix<int,Eigen::Dynamic,1> IDVector;
        IDVector idVector;
        //check that the argument is a pcd file
        if (map_name.compare (map_name.size () - extension.size (), extension.size (), extension) == 0)
        {
            std::string yaml_mapName="map"+std::to_string(i+1);

            fsSetting[yaml_mapName]>>ID;
            cv::cv2eigen(ID,idVector);
            std::vector<int> ids;
            for(int k=0;k<idVector.size();k++)
            {
             ids.push_back(idVector[k]);
            }
            cloud_choosed tmp(yaml_mapName,ids,ids.size(),i+1);
            pcl::io::loadPCDFile(map_name, *tmp.cloud);
            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(*tmp.cloud,*tmp.cloud, indices);

            cloud_choosed_tmp[i]=tmp;
        }
        std::cout<<"load cloud "<<i<<". time used: "<<tt.toc()<<"ms\n";
        index_g=i;
        areai_choose(i);
    }
}



void print4x4Matrix(Eigen::Matrix4f &matrix)
{

    printf ("Rotation matrix:\n");
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
    printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
    printf ("Translation vector :\n");
    printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}


void icp_registration(PointCloud::Ptr source1,PointCloud::Ptr target1,int id1,int id2)
{
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*target1, *target1, indices);
    std::vector<int> indices1;
    pcl::removeNaNFromPointCloud(*source1, *source1, indices1);
    Eigen::Matrix4d FinalTransform1;
    PointCloud::Ptr aligned(new PointCloud());
    std::cout <<"target:"<<target1->size()<<endl;
    std::cout <<"source1:"<<source1->size()<<endl;
    std::cout << id1<<" with "<<id2<<" icp is Iterating...!!!" << std::endl;

    pcl::IterativeClosestPoint<Point, Point> icp;
    icp.setMaximumIterations(500);
    icp.setInputSource(source1);
    icp.setInputTarget(target1);
    icp.align(*aligned);
    // FinalTransform = icp.getFinalTransformation().cast<double>();
    cloud_choosed_ptr[id2].Transform=icp.getFinalTransformation().cast<double>();
    if(icp.getFitnessScore()<1.0)
    {

        std::cout << "icp Converged:" << icp.hasConverged() << std::endl;
        std::cout << "icp score is " << icp.getFitnessScore() << std::endl;
        std::cout << "icp getFinalTransformation:" << icp.getFinalTransformation().cast<double>()<< std::endl;
        return;
    }


    std::cout << "icp Converged:" << icp.hasConverged() << std::endl;
    std::cout << "icp score is " << icp.getFitnessScore() << std::endl;
    std::cout << "icp getFinalTransformation:" << icp.getFinalTransformation().cast<double>()<< std::endl;
}

/***********************FPFH_ICP start***********************/

void FPFH_ICP(ros::NodeHandle &n) {
    int cout = 0;
    if (!loadData(n))
        return;

    std::cout << "load successful!\n";
    return;

    // Check user input
    if (!cloud_choosed_ptr) {
        PCL_ERROR ("Syntax is: %s <source.pcd> <target.pcd> [*]");
        PCL_ERROR (
                "[*] - multiple files can be added. The registration results of (i, i+1) will be registered against (i+2), etc");
        return;
    }


    struct cloud_choosed cloud_choosed_tmp[cout];
    int num_neighbor;
    int neighbor_id;

    cloud_choosed_ptr = cloud_choosed_tmp;
//    for(int i=0; i<data.size();i++)
//    {
//        std::cout<<"data name:\n"<<data[i].f_name<<std::endl;
//        std::cout<<"input num_neighbor of:"<<std::endl;
//
//        std::scanf("%d",&num_neighbor);
//        //std::cin>>num_neighbor;
//        std::cout<<"num_neighbor:"<<num_neighbor<<std::endl;
//
//        std::cout<<"input neighbor index:\n";
//        std::vector<int> tmp_vector;
//        //cin>>neighbor_id;
//        for(int i=0;i<num_neighbor;i++)
//        {
//            std::scanf("%d",&neighbor_id);
//            tmp_vector.push_back(neighbor_id);
//        }
//
//        cloud_choosed tmp("viewer "+std::to_string(i),tmp_vector,num_neighbor,i);
//        tmp.cloud=data[i].cloud;
//        cloud_choosed_tmp[i]=tmp;
//        index_g=i;
//        areai_choose(i);
//    }

    //todo 这里有毒
    //std::cout << "index_g :" << index_g <<" id:" << id<< std::endl;
    boost::shared_ptr<Visualizer> viewer_tmp(new Visualizer("viewer111_tmp"));
    std::cout << "cloud_choosed_ptr[0]:" << cloud_choosed_ptr[0].this_index << endl;
    viewer_tmp->addPointCloud(cloud_choosed_ptr[0].neigh_cloud_choosed[1], "tmp");
    viewer_tmp->addPointCloud(cloud_choosed_ptr[1].neigh_cloud_choosed[2], "tmp1");
    viewer_tmp->addPointCloud(cloud_choosed_ptr[1].neigh_cloud_choosed[3], "tmp2");
    viewer_tmp->addCoordinateSystem(100.0);
    viewer_tmp->setCameraPosition(0, 0, -2, -1, 0, 0);

    while (!viewer_tmp->wasStopped()) {
        viewer_tmp->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    viewer_tmp->spin();



    //根据相邻的点云ID 设置将要匹配的对象

    // view_point_choosed();

    // p = new pcl::visualization::PCLVisualizer (argc, argv, "Pairwise Incremental Registration example");
    //p->createViewPort (0.0, 0, 0.5, 1.0, vp_1);
    // p->createViewPort (0.5, 0, 1.0, 1.0, vp_2);

    // PointCloud::Ptr result(new PointCloud);
    //  Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;

//    for (size_t i = 1; i < data.size (); ++i)
//    {
//       // source = cloud_choosed_ptr[i-1].cloud_choosed;
//        std::cout<<"source cloud:"<<cloud_choosed_ptr[i-1].f_name<<std::endl;
//      //  target = cloud_choosed_ptr[i].cloud_choosed;
//        std::cout<<"target cloud:"<<cloud_choosed_ptr[i].f_name<<std::endl;
//
//        // Add visualization data
//        showCloudsLeft(target, source);
//
//        PointCloud::Ptr temp (new PointCloud);
//        PointCloud::Ptr final (new PointCloud);
//        final=data[0].cloud;
//        PointCloud::Ptr init_result (new PointCloud);
//        /*/
/*init_result = *target;
//        Eigen::Matrix4f init_transform = Eigen::Matrix4f::Identity ();
//        PCL_INFO ("Aligning %s (%d) with %s (%d).\n", data[i-1].f_name.c_str (), source->points.size (), data[i].f_name.c_str (), target->points.size ());
//        //source 到target的变换， init_result指向变换后的source
//        sac_ia_align(source,target,init_result,init_transform,max_sacis_iteration,min_correspondence_dist,max_correspondence_dist);
//        pairAlign (init_result, target, temp, pairTransform, true);
//        pairTransform*=init_transform;
//
//        pcl::transformPointCloud (*temp, *result, GlobalTransform);
//        *final+=*result;
//
//        GlobalTransform = GlobalTransform * pairTransform;
//
//        //save aligned pair, transformed into the first cloud's frame
//        std::stringstream ss;
//        ss << i << ".pcd";
//        pcl::io::savePCDFile (ss.str (), *final, true);
//
//   }

    int registration_count=0;
    //遍历所有点云
    for(int i=0;i<cout;i++)
    {
        //遍历当前点云的邻接点云，
        for(auto iter=cloud_choosed_ptr[i].neigh_cloud_choosed.begin();iter!=cloud_choosed_ptr[i].neigh_cloud_choosed.end();iter++)
        {

            if(cloud_choosed_ptr[iter->first].this_index==iter->first)
            {
                //将当前点云作为target，邻接点云作为source
                // std::map<int,PointCloud::Ptr> tmp;
                // tmp=cloud_choosed_ptr[iter->first].cloud_choosed;

                registration_count++;
                icp_registration(cloud_choosed_ptr[iter->first].neigh_cloud_choosed[cloud_choosed_ptr[i].this_index],cloud_choosed_ptr[i].neigh_cloud_choosed[iter->first],
                                 cloud_choosed_ptr[i].this_index,iter->first);

                cloud_choosed_ptr[iter->first].neigh_cloud_choosed.erase(cloud_choosed_ptr[i].this_index);
            }

        }
    }
    std::cout<<"registration_count:"<<registration_count<<std::endl;

    PointCloud::Ptr transformed_cloud(new PointCloud());
    PointCloud::Ptr transformed_cloud1(new PointCloud());
    PointCloud::Ptr transformed_cloud2(new PointCloud());
    pcl::transformPointCloud(*cloud_choosed_ptr[1].cloud, *transformed_cloud, cloud_choosed_ptr[1].Transform);

    pcl::transformPointCloud(*cloud_choosed_ptr[2].cloud, *transformed_cloud1, cloud_choosed_ptr[2].Transform);
    pcl::transformPointCloud(*transformed_cloud1, *transformed_cloud1, cloud_choosed_ptr[1].Transform);

    pcl::transformPointCloud(*cloud_choosed_ptr[3].cloud, *transformed_cloud2, cloud_choosed_ptr[3].Transform);
    pcl::transformPointCloud(*transformed_cloud2, *transformed_cloud2, cloud_choosed_ptr[1].Transform);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    //pcl::visualization::PCLVisualizer viewer("3D viewer");
    viewer->initCameraParameters();

    // viewer->createViewPort(0.0, 0.0, 0.5, 1.0);
    viewer->setBackgroundColor(0, 0, 0);

    PointCloudColorHandlerCustom<Point> red(cloud_choosed_ptr[0].cloud, 255, 0, 0);
    PointCloudColorHandlerCustom<Point> green(transformed_cloud, 0, 255, 0);
    PointCloudColorHandlerCustom<Point> blue(transformed_cloud1, 0, 0, 255);
    PointCloudColorHandlerCustom<Point> blue1(transformed_cloud2, 100, 0, 200);

    viewer->addPointCloud(cloud_choosed_ptr[0].cloud, red, "target");
    viewer->addPointCloud(transformed_cloud, green, "source1");
    viewer->addPointCloud(transformed_cloud1, blue, "source2");
    viewer->addPointCloud(transformed_cloud2, blue1, "source3");

    pcl::io::savePCDFileASCII("/home/gp/data/saved1.pcd", *transformed_cloud);
    pcl::io::savePCDFileASCII("/home/gp/data/saved2.pcd", *transformed_cloud1);
    pcl::io::savePCDFileASCII("/home/gp/data/saved3.pcd", *transformed_cloud2);

    viewer->spin ();

}


/***********************FPFH_ICP end***********************/
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"map_registration");
    ros::NodeHandle n("~");
    help();

    TicToc t_t;
    FPFH_ICP(n);
    PCL_INFO("time used in registration: %fms",t_t.toc());
    return 0;
}

//
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/filters/voxel_grid.h>
//#include <iostream>
//#include <vector>
//
//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));
//pcl::PointCloud<pcl::PointXYZ>::Ptr clicked_points_3d(new pcl::PointCloud<pcl::PointXYZ>);
//int num = 0;
//
//void pp_callback(const pcl::visualization::AreaPickingEvent& event, void* args)
//{
//    std::vector< int > indices;
//    if (event.getPointsIndices(indices)==-1)
//        return;
//
//    for (int i = 0; i < indices.size(); ++i)
//    {
//        clicked_points_3d->points.push_back(cloud->points.at(indices[i]));
//    }
//
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(clicked_points_3d, 255, 0, 0);
//
//    std::stringstream ss;
//    std::string cloudName;
//    ss << num++;
//    ss >> cloudName;
//    cloudName += "_cloudName";
//
//    viewer->addPointCloud(clicked_points_3d, red, cloudName);
//    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, cloudName);
//}
//
//int main(int argc , char** argv)
//{
//    if (pcl::io::loadPCDFile("/home/gp/data/map/2.pcd", *cloud))
//    {
//        std::cerr << "ERROR: Cannot open file " << std::endl;
//        return 0;
//    }
//    viewer->addPointCloud(cloud, "bunny");
//    viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);
//    viewer->registerAreaPickingCallback(pp_callback, (void*)&cloud);
//
//    //VoxelGrid
//    pcl::VoxelGrid<pcl::PointXYZ> vgF;
//    vgF.setInputCloud(cloud);
//    vgF.setLeafSize(0.1, 0.1, 0.1);
//    vgF.filter(*cloud);
//
//    while (!viewer->wasStopped())
//    {
//        viewer->spinOnce(100);
//        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
//    }
//    return 0;
//}