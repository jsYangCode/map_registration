/**
 *
 * Created by js.yang on 30/8/2019 for map registration step by step
 *
 * */
#ifndef MAP_REGISTRATION_H
#define MAP_REGISTRATION_H

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

#include <pcl/visualization/pcl_visualizer.h>
//#include <boost/thread/thread.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <sensor_msgs/PointCloud2.h>


#include <time.h>
//Bilateral Filter
#include <pcl/filters/bilateral.h>//required
#include <pcl/filters/fast_bilateral.h>
#include <pcl/filters/fast_bilateral_omp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>


#include <pcl/registration/ndt.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <mutex>
#include <thread>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pclomp/ndt_omp.h>
//ros::NodeHandle nh;
//ros::Publisher point_chooseed_publisher1;
#include <tic_toc.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/registration/ndt.h>
//namespace map_registration
//{
    typedef pcl::PointXYZ Point;
    typedef pcl::PointCloud<Point> PointCloud;
    typedef pcl::visualization::PCLVisualizer Visualizer;

    class map_regis
    {
        public:
            map_regis();
            void help();
            PointCloud::Ptr filter(PointCloud::Ptr cloud_in);

            void loadPCD(int argc, char**argv);

            void area1_choose();
            void area2_choose();

            void pp1_callback(const pcl::visualization::AreaPickingEvent &event, void *args);
            void pp2_callback(const pcl::visualization::AreaPickingEvent &event, void *args);
            void view_point_choosed();

            void icp_registration();
            void save_PointCloud();
            void Transform_to_target();
            static boost::shared_ptr<map_regis> instance(void);

        private:
            PointCloud::Ptr source;
            PointCloud::Ptr target;
            PointCloud::Ptr clicked_points_3d;
            PointCloud::Ptr clicked_points_3d1;
            int id;
            const std::string source_file_name;
            const std::string target_file_name;
            boost::shared_ptr<Visualizer> viewer;
            boost::shared_ptr<Visualizer> viewer2;
            boost::shared_ptr<Visualizer> viewer3;
            boost::shared_ptr<Visualizer> viewer4;
            bool flag;
            boost::mutex cloud_mutex;
            Eigen::Matrix4d FinalTransform;
            struct callback_args
            {
            // structure used to pass arguments to the callback function
            PointCloud::Ptr clicked_points_3d;
            Visualizer::Ptr viewerPtr;
            };
            static boost::shared_ptr<map_regis> m_instance;
    };

//}

#endif