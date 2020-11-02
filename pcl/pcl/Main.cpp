#pragma warning (disable:4996)

#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);
VTK_MODULE_INIT(vtkRenderingVolumeOpenGL2);

#include <iostream>
#include <algorithm>
#include <random>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

typedef pcl::PointXYZ PointT;

void VisualizationExample()
{
    // load point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("pcds/Pcd_convert.pcd", *cloud);

    // visualize
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.5);
    viewer.addPointCloud(cloud);

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
}

int main()
{
    // load point cloud
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::io::loadPCDFile("Pcd_convert.pcd", *cloud);

    // variables
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
    pcl::PointCloud<PointT>::Ptr cloud_cylinder_centers(new pcl::PointCloud<PointT>);

    // Estimate point normals
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud);
    ne.setKSearch(50);
    ne.compute(*cloud_normals);

    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(0.1);
    seg.setMaxIterations(10000);
    seg.setDistanceThreshold(0.05);
    seg.setRadiusLimits(0, 0.1);
    seg.setInputCloud(cloud);
    seg.setInputNormals(cloud_normals);

    std::default_random_engine rng1(1);
    std::default_random_engine rng2(rng1);
    //for (int i = 0; i < 20; ++i)
    //{
    //    std::shuffle(cloud->points.begin(), cloud->points.end(), rng1);
    //    std::shuffle(cloud_normals->points.begin(), cloud_normals->points.end(), rng2);
    //    // Obtain the cylinder inliers and coefficients
    //    seg.segment(*inliers_cylinder, *coefficients_cylinder);
    //    std::cerr << "[" << i << "]" << std::endl;
    //    std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

    //    cloud_cylinder_centers->points.push_back(PointT(coefficients_cylinder->values[0], coefficients_cylinder->values[1], coefficients_cylinder->values[2]));
    //}
    
    // visualize
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.5);
    viewer.addPointCloud(cloud, "1");
    viewer.addPointCloudNormals<PointT, pcl::Normal>(cloud, cloud_normals);
    //pcl::visualization::PointCloudColorHandlerCustom<PointT> rgb(cloud_cylinder_centers, 255, 0, 0);
    //viewer.addPointCloud(cloud_cylinder_centers, rgb, "2");
    

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
    
    return 0;
}