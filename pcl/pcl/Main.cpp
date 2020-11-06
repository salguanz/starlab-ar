#pragma warning (disable:4996)

#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);
VTK_MODULE_INIT(vtkRenderingVolumeOpenGL2);

#include <iostream>

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

#include "Model.hpp"

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
    pcl::io::loadPCDFile("pcds/cylinders/cylinder4.pcd", *cloud);
    
    // variables
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::PointCloud<PointT>::Ptr cylinder_centers(new pcl::PointCloud<PointT>);

    // Estimate point normals
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud);
    ne.setKSearch(50);
    ne.compute(*normals);

    // Extract cylinder centers
    Model::ExtractPipels(cloud, normals, *cylinder_centers, 20, false);
    
    // visualize
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.5);
    viewer.addPointCloud(cloud, "1");
    viewer.addPointCloudNormals<PointT, pcl::Normal>(cloud, normals);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> rgb(cylinder_centers, 255, 0, 0);
    viewer.addPointCloud(cylinder_centers, rgb, "2");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "2");
    

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
    
    return 0;
}