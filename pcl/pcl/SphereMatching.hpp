#pragma once
#pragma warning (disable:4996)
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
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>

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

void CylinderFittingProcess(pcl::PointCloud<PointT>::Ptr cloud)
{
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
    seg.setOptimizeCoefficients(false);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(0.1);
    seg.setMaxIterations(10000);
    seg.setDistanceThreshold(0.05);
    seg.setRadiusLimits(0, 30.0);
    seg.setInputCloud(cloud);
    seg.setInputNormals(cloud_normals);

    std::default_random_engine rng1(1);
    std::default_random_engine rng2(rng1);
    for (int i = 0; i < 200; ++i)
    {
        std::shuffle(cloud->points.begin(), cloud->points.end(), rng1);
        std::shuffle(cloud_normals->points.begin(), cloud_normals->points.end(), rng2);
        // Obtain the cylinder inliers and coefficients
        seg.segment(*inliers_cylinder, *coefficients_cylinder);
        std::cerr << "[" << i << "]" << std::endl;
        std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

        cloud_cylinder_centers->points.push_back(PointT(coefficients_cylinder->values[0], coefficients_cylinder->values[1], coefficients_cylinder->values[2]));
    }

    // visualize
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.5);
    viewer.addPointCloud(cloud, "1");
    viewer.addPointCloudNormals<PointT, pcl::Normal>(cloud, cloud_normals);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> rgb(cloud_cylinder_centers, 255, 0, 0);
    viewer.addPointCloud(cloud_cylinder_centers, rgb, "2");

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
}
pcl::PointCloud<PointT>::Ptr ExtractRandomOnePointKNearSet(pcl::PointCloud<PointT>::Ptr cloud, int K)
{
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    pcl::PointCloud<PointT>::Ptr pcdKNset(new pcl::PointCloud<PointT>);

    tree->setInputCloud(cloud);

    std::vector<int> indices(K);
    std::vector<float> dists(K);
    

    tree->nearestKSearchT(*(cloud->points.begin()), K, indices, dists);

    for (std::size_t i = 0; i < indices.size(); ++i)
    {
        pcdKNset->points.push_back(PointT((*cloud)[indices[i]].x, (*cloud)[indices[i]].y, (*cloud)[indices[i]].z));
        //std::cout << "x: "<< (*cloud)[indices[i]].x << "y: " << (*cloud)[indices[i]].y << "z: " << (*cloud)[indices[i]].z << std::endl;
    }

    return pcdKNset;
}

std::tuple<pcl::PointCloud<PointT>::Ptr, pcl::PointCloud< pcl::Normal >::Ptr> ExtractRandomOnePointNormalKNearSet(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, int K)
{
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    pcl::PointCloud<PointT>::Ptr pcdKNset(new pcl::PointCloud<PointT>);
    pcl::PointCloud< pcl::Normal >::Ptr normalKNset(new pcl::PointCloud< pcl::Normal >);
    std::tuple<pcl::PointCloud<PointT>::Ptr, pcl::PointCloud< pcl::Normal >::Ptr> kNset;

    tree->setInputCloud(cloud);

    std::vector<int> indices(K);
    std::vector<float> dists(K);


    tree->nearestKSearchT(*(cloud->points.begin()), K, indices, dists);

    for (std::size_t i = 0; i < indices.size(); ++i)
    {
        pcdKNset->points.push_back(PointT((*cloud)[indices[i]].x, (*cloud)[indices[i]].y, (*cloud)[indices[i]].z));
        normalKNset->points.push_back(pcl::Normal((*cloud_normals)[indices[i]].normal_x, (*cloud_normals)[indices[i]].normal_y, (*cloud_normals)[indices[i]].normal_z));
        //std::cout << "x: "<< (*cloud)[indices[i]].x << "y: " << (*cloud)[indices[i]].y << "z: " << (*cloud)[indices[i]].z << std::endl;
    }

    return make_tuple(pcdKNset, normalKNset);
}

pcl::PointCloud<PointT>::Ptr ExtractFromSphereKNearSet(pcl::PointCloud<PointT>::Ptr cloud, int K, PointT center)
{
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    pcl::PointCloud<PointT>::Ptr pcdKNset(new pcl::PointCloud<PointT>);
    tree->setInputCloud(cloud);

    std::vector<int> indices(K);
    std::vector<float> dists(K);


    tree->nearestKSearchT(center, K, indices, dists);

    for (std::size_t i = 0; i < indices.size(); ++i)
    {
        pcdKNset->points.push_back(PointT((*cloud)[indices[i]].x, (*cloud)[indices[i]].y, (*cloud)[indices[i]].z));
        //std::cout << "x: "<< (*cloud)[indices[i]].x << "y: " << (*cloud)[indices[i]].y << "z: " << (*cloud)[indices[i]].z << std::endl;
    }
    return pcdKNset;
}

std::tuple<pcl::PointCloud<PointT>::Ptr, pcl::PointCloud< pcl::Normal >::Ptr> ExtractNormalFromSphereKNearSet(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, int K, PointT center)
{
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    pcl::PointCloud<PointT>::Ptr pcdKNset(new pcl::PointCloud<PointT>);
    pcl::PointCloud< pcl::Normal >::Ptr normalKNset(new pcl::PointCloud< pcl::Normal >);
    std::tuple<pcl::PointCloud<PointT>::Ptr, pcl::PointCloud< pcl::Normal >::Ptr> kNset;
    tree->setInputCloud(cloud);

    std::vector<int> indices(K);
    std::vector<float> dists(K);


    tree->nearestKSearchT(center, K, indices, dists);

    for (std::size_t i = 0; i < indices.size(); ++i)
    {
        pcdKNset->points.push_back(PointT((*cloud)[indices[i]].x, (*cloud)[indices[i]].y, (*cloud)[indices[i]].z));
        normalKNset->points.push_back(pcl::Normal((*cloud_normals)[indices[i]].normal_x, (*cloud_normals)[indices[i]].normal_y, (*cloud_normals)[indices[i]].normal_z));
        //std::cout << "x: "<< (*cloud)[indices[i]].x << "y: " << (*cloud)[indices[i]].y << "z: " << (*cloud)[indices[i]].z << std::endl;
    }
    return make_tuple(pcdKNset, normalKNset);
}
void SphereFittingProcess(pcl::PointCloud<PointT>::Ptr cloud, int K)
{
    pcl::SACSegmentation<PointT> seg;

    pcl::ModelCoefficients::Ptr coefficients_sphere(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_sphere(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_second_sphere(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_second_sphere(new pcl::PointIndices);

    pcl::PointCloud<PointT>::Ptr cloud_first_sphere_centers(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_second_sphere_centers(new pcl::PointCloud<PointT>);


    pcl::PointCloud<PointT>::Ptr nearestSpherePoints(new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr nearestSphereNormals(new pcl::PointCloud <pcl::Normal>);
    pcl::PointCloud<PointT>::Ptr inputNearestPoints(new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr inputNearestNormals(new pcl::PointCloud<pcl::Normal>);

    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

    std::cerr << "START" << std::endl;

    // add normal on cloud

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_SPHERE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.05);
    //seg.setRadiusLimits(0.1, 30.0);
    //seg.setInputCloud(cloud);

    std::default_random_engine rng1(2);
    std::default_random_engine rng2(rng1);

    for (int i = 0; i < 200; ++i)
    {
        std::shuffle(cloud->points.begin(), cloud->points.end(), rng1);
        seg.setRadiusLimits(0, 50.0);

        inputNearestPoints = ExtractRandomOnePointKNearSet(cloud, 50);
        seg.setInputCloud(inputNearestPoints);
        // Obtain the Sphere inliers and coefficients
        seg.segment(*inliers_sphere, *coefficients_sphere);

        std::cerr << "[" << i << "]:1" << std::endl;
        std::cerr << "Sphere coefficients: " << *coefficients_sphere << std::endl;
        std::cerr << "Sphere inliers: " << (*inliers_sphere).indices.size() << std::endl;

        PointT first_center = PointT(coefficients_sphere->values[0], coefficients_sphere->values[1], coefficients_sphere->values[2]);

        cloud_first_sphere_centers->points.push_back(first_center);


        //seg.setRadiusLimits(0, 200.0);

        nearestSpherePoints = ExtractFromSphereKNearSet(cloud, K, first_center);
        seg.setInputCloud(nearestSpherePoints);

        // Obtain the Sphere inliers and coefficients
        seg.segment(*inliers_second_sphere, *coefficients_second_sphere);

        std::cerr << "[" << i << "]:2" << std::endl;
        std::cerr << "Sphere coefficients: " << *coefficients_second_sphere << std::endl;
        std::cerr << "Sphere inliers: " << (*inliers_second_sphere).indices.size() << std::endl;


        PointT second_center = PointT(coefficients_second_sphere->values[0], coefficients_second_sphere->values[1], coefficients_second_sphere->values[2]);
        cloud_second_sphere_centers->points.push_back(second_center);
    }
    // visualize
    std::cerr << "END" << std::endl;
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.5);
    viewer.addPointCloud(cloud, "1");
    pcl::visualization::PointCloudColorHandlerCustom<PointT> rgb(cloud_first_sphere_centers, 255, 0, 0);
    viewer.addPointCloud(cloud_first_sphere_centers, rgb, "2");
    pcl::visualization::PointCloudColorHandlerCustom<PointT> rgb2(cloud_second_sphere_centers, 0, 255, 0);
    viewer.addPointCloud(cloud_second_sphere_centers, rgb2, "3");
    pcl::visualization::PointCloudColorHandlerCustom<PointT> rgb3(nearestSpherePoints, 255, 128, 128);
    viewer.addPointCloud(nearestSpherePoints, rgb3, "4");
    pcl::visualization::PointCloudColorHandlerCustom<PointT> rgb4(inputNearestPoints, 128, 255, 255);
    viewer.addPointCloud(inputNearestPoints, rgb4, "5");

    viewer.addSphere(*coefficients_sphere, "sphere");
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "sphere");

    viewer.addSphere(*coefficients_second_sphere, "sphere2");
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "sphere2");

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }

}
void NormalSphereFittingProcess(pcl::PointCloud<PointT>::Ptr cloud, int K)
{
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
    pcl::NormalEstimation<PointT, pcl::Normal> ne;

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

    pcl::ModelCoefficients::Ptr coefficients_sphere(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_sphere(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_second_sphere(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_second_sphere(new pcl::PointIndices);

    pcl::PointCloud<PointT>::Ptr cloud_first_sphere_centers(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_second_sphere_centers(new pcl::PointCloud<PointT>);
    
    
    pcl::PointCloud<PointT>::Ptr nearestSpherePoints(new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr nearestSphereNormals(new pcl::PointCloud <pcl::Normal> );
    pcl::PointCloud<PointT>::Ptr inputNearestPoints(new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr inputNearestNormals(new pcl::PointCloud<pcl::Normal>);

    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

    std::cerr << "START" << std::endl;

    // add normal on cloud
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud);
    ne.setKSearch(50);
    ne.compute(*cloud_normals);

    seg.setOptimizeCoefficients(false);
    seg.setModelType(pcl::SACMODEL_NORMAL_SPHERE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.003);
    //seg.setRadiusLimits(0.1, 30.0);
    //seg.setInputCloud(cloud);
    
    std::default_random_engine rng1(2);
    std::default_random_engine rng2(rng1);
    
    for (int i = 0; i < 200; ++i)
    {
        std::shuffle(cloud->points.begin(), cloud->points.end(), rng1);
        std::shuffle(cloud_normals->points.begin(), cloud_normals->points.end(), rng2);
        seg.setRadiusLimits(0, 100.0);

        auto firstN = ExtractRandomOnePointNormalKNearSet(cloud,cloud_normals, 200);
        inputNearestPoints = std::get<0>(firstN);
        inputNearestNormals = std::get<1>(firstN);
        seg.setInputNormals(inputNearestNormals);
        seg.setInputCloud(inputNearestPoints);

        // Obtain the Sphere inliers and coefficients
        seg.segment(*inliers_sphere, *coefficients_sphere);
        
        std::cerr << "[" << i << "]:1" << std::endl;
        std::cerr << "Sphere coefficients: " << *coefficients_sphere << std::endl;
        std::cerr << "Sphere inliers: " << (*inliers_sphere).indices.size() << std::endl;
        
        PointT first_center = PointT(coefficients_sphere->values[0], coefficients_sphere->values[1], coefficients_sphere->values[2]);
        
        cloud_first_sphere_centers->points.push_back(first_center);


        //seg.setRadiusLimits(0, 200.0);

        auto secondN = ExtractNormalFromSphereKNearSet(cloud, cloud_normals, K, first_center);

        nearestSpherePoints = std::get<0>(secondN);
        nearestSphereNormals = std::get<1>(secondN);
        seg.setInputNormals(nearestSphereNormals);
        seg.setInputCloud(nearestSpherePoints);

        // Obtain the Sphere inliers and coefficients
        seg.segment(*inliers_second_sphere, *coefficients_second_sphere);

        std::cerr << "[" << i << "]:2" << std::endl;
        std::cerr << "Sphere coefficients: " << *coefficients_second_sphere << std::endl;
        std::cerr << "Sphere inliers: " << (*inliers_second_sphere).indices.size() << std::endl;


        PointT second_center = PointT(coefficients_second_sphere->values[0], coefficients_second_sphere->values[1], coefficients_second_sphere->values[2]);

        cloud_second_sphere_centers->points.push_back(second_center);
    }
    // visualize
    std::cerr << "END" << std::endl;
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.5);
    viewer.addPointCloud(cloud, "1");
    pcl::visualization::PointCloudColorHandlerCustom<PointT> rgb(cloud_first_sphere_centers, 255, 0, 0);
    viewer.addPointCloud(cloud_first_sphere_centers, rgb, "2");
    pcl::visualization::PointCloudColorHandlerCustom<PointT> rgb2(cloud_second_sphere_centers, 0, 255, 0);
    viewer.addPointCloud(cloud_second_sphere_centers, rgb2, "3");
    pcl::visualization::PointCloudColorHandlerCustom<PointT> rgb3(nearestSpherePoints, 255, 128, 128);
    viewer.addPointCloud(nearestSpherePoints, rgb3, "4");
    pcl::visualization::PointCloudColorHandlerCustom<PointT> rgb4(inputNearestPoints, 128, 255, 255);
    viewer.addPointCloud(inputNearestPoints, rgb4, "5");

    viewer.addPointCloudNormals<PointT, pcl::Normal>(cloud, cloud_normals);

    viewer.addSphere(*coefficients_sphere, "sphere");
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "sphere");
    viewer.addSphere(*coefficients_second_sphere, "sphere2");
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,"sphere2");

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }

}
pcl::PointCloud<PointT>::Ptr SamplePointCloud(pcl::PointCloud<PointT>::Ptr cloud) 
{
    pcl::PointCloud<PointT>::Ptr sampled_cloud(new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.0001f, 0.0001f, 0.0001f);
    sor.filter(*sampled_cloud);

    return sampled_cloud;
}

pcl::PointCloud<PointT>::Ptr ScalePointCloud(pcl::PointCloud<PointT>::Ptr cloud, float scale)
{
    pcl::PointCloud<PointT>::Ptr transformed_cloud(new pcl::PointCloud<PointT>);
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform(0, 0) = transform(0, 0) * scale;
    transform(1, 1) = transform(1, 1) * scale;
    transform(2, 2) = transform(2, 2) * scale;
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform);

    return transformed_cloud;
}
