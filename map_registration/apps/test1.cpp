#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <iostream>
#include <vector>

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));
pcl::PointCloud<pcl::PointXYZ>::Ptr clicked_points_3d(new pcl::PointCloud<pcl::PointXYZ>);
int num = 0;

void pp_callback(const pcl::visualization::AreaPickingEvent& event, void* args)
{
    std::vector< int > indices;
    if (event.getPointsIndices(indices)==-1)
        return;

    for (int i = 0; i < indices.size(); ++i)
    {
        clicked_points_3d->points.push_back(cloud->points.at(indices[i]));
    }

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(clicked_points_3d, 255, 0, 0);

    std::stringstream ss;
    std::string cloudName;
    ss << num++;
    ss >> cloudName;
    cloudName += "_cloudName";

    viewer->addPointCloud(clicked_points_3d, red, cloudName);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, cloudName);
}

int main(int argc , char** argv)
{
    if (pcl::io::loadPCDFile("/home/gp/data/tianhongjie_regis2sandun.pcd", *cloud))
    {
        std::cerr << "ERROR: Cannot open file " << std::endl;
        return 0;
    }
    viewer->addPointCloud(cloud, "bunny");
    viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);
    viewer->registerAreaPickingCallback(pp_callback, (void*)&cloud);

    //VoxelGrid
    pcl::VoxelGrid<pcl::PointXYZ> vgF;
    vgF.setInputCloud(cloud);
    vgF.setLeafSize(0.1, 0.1, 0.1);
    vgF.filter(*cloud);

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    return 0;
}
