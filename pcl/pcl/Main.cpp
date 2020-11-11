#pragma warning (disable:4996)
#include "SphereMatching.hpp"

int main(int argc, char **argv)
{
    // load point cloud
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_knears(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::iterator i;
    pcl::io::loadPCDFile("Pcd_convert_old.pcd", *cloud);
    
    /*for (i = cloud->begin(); i != cloud->end(); i= i+1000)
    {
        std::cout << (*i).x << " " << (*i).y << " " << (*i).z << std::endl;
    }*/
    //CylinderFittingProcess(cloud);
    SphereFittingProcess(ScalePointCloud(cloud, 50), 3000);
    NormalSphereFittingProcess(ScalePointCloud(cloud, 10), 500);
    
    
    
    /*
    std::default_random_engine rng1(1);
    std::shuffle(cloud->points.begin(), cloud->points.end(), rng1);

    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.5);
    viewer.addPointCloud(cloud, "1");

    cloud_knears = ExtractRandomOnePointKNearSet(cloud, 50);

    pcl::visualization::PointCloudColorHandlerCustom<PointT> rgb(cloud_knears, 255, 0, 0);
    viewer.addPointCloud(cloud_knears, rgb, "2");

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
    */


    return 0;
}