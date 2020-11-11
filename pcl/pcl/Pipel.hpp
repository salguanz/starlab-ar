#pragma once
#pragma warning (disable:4996)

#include <iostream>
#include <algorithm>
#include <random>
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
typedef pcl::PointNormal PointN;

/*
*	Input 3 points and normals (if more points given, select only front 3 points) 
*	Return Radius
*/
float ComputeRadius(pcl::PointCloud<PointN>::Ptr cloud)
{
	std::vector<Eigen::Vector3f> P;
	std::vector<Eigen::Vector3f> N;

	P.push_back(Eigen::Vector3f(cloud->points[0].getVector3fMap()));
	P.push_back(Eigen::Vector3f(cloud->points[1].getVector3fMap()));
	P.push_back(Eigen::Vector3f(cloud->points[2].getVector3fMap()));

	N.push_back(Eigen::Vector3f(cloud->points[0].getNormalVector3fMap()));
	N.push_back(Eigen::Vector3f(cloud->points[1].getNormalVector3fMap()));
	N.push_back(Eigen::Vector3f(cloud->points[2].getNormalVector3fMap()));

	return ((P[2] - P[1]).dot(N[1] - N[0]) + (P[1] - P[0]).dot(N[2] - N[1])) / (N[2] - N[1]).dot(N[1] - N[0]);
}

void CalculateNormal(pcl::PointCloud<PointT>::Ptr cloudIn, pcl::PointCloud<PointN>::Ptr cloudOut, int k)
{
	pcl::NormalEstimation<PointT, pcl::Normal> ne;
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
	pcl::PointCloud<pcl::Normal>::Ptr cloudNormal(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<PointN>::Ptr cloudAll(new pcl::PointCloud<PointN>);
	
	ne.setSearchMethod(tree);
	ne.setInputCloud(cloudIn);
	ne.setKSearch(k);
	ne.compute(*cloudNormal);

	pcl::concatenateFields(*cloudIn, *cloudNormal, *cloudOut);
}

void ExtractNearSet(pcl::PointCloud<PointN>::Ptr cloud, pcl::PointCloud<PointN>::Ptr cloudOut, int K, int index)
{
	pcl::search::KdTree<PointN>::Ptr tree(new pcl::search::KdTree<PointN>());

	tree->setInputCloud(cloud);

	std::vector<int> indices(K);
	std::vector<float> dists(K);

	tree->nearestKSearchT(cloud->points[index], K, indices, dists);
	for (std::size_t i = 0; i < indices.size(); ++i)
	{
		cloudOut->points.emplace_back((*cloud)[indices[i]].x, (*cloud)[indices[i]].y, (*cloud)[indices[i]].z, 
			(*cloud)[indices[i]].normal_x, (*cloud)[indices[i]].normal_y, (*cloud)[indices[i]].normal_z);
	}
}

void Godeunger(pcl::PointCloud<PointT>::Ptr rawCloud)
{
	const int K = 50;

	int out = 0;

	pcl::PointCloud<PointN>::Ptr cloud(new pcl::PointCloud<PointN>);
	pcl::PointCloud<PointT>::Ptr cloudCenters(new pcl::PointCloud<PointT>);


	CalculateNormal(rawCloud, cloud, 50);
	int cloudSize = cloud->size();

	std::default_random_engine rng1(2);
	
	std::uniform_int_distribution<int> dist_1(0, cloudSize - 1);
	std::uniform_int_distribution<int> dist_2(0, K - 1);

	for (int i = 0; i < 500; i++)
	{
		pcl::PointCloud<PointN>::Ptr cloudPart(new pcl::PointCloud<PointN>);
		ExtractNearSet(cloud, cloudPart, K, dist_1(rng1));

		//pcl::PointCloud<PointN>::Ptr threePoints(new pcl::PointCloud<PointN>);
		//threePoints->points.push_back(cloudPart);

		std::shuffle(cloudPart->points.begin(), cloudPart->points.end(), rng1);

		float rawR = ComputeRadius(cloudPart);
		float r;
		bool negativeRadius = false;

		if (rawR < 0)
		{
			negativeRadius = true;
			r = -rawR;
		}
		else 
			r = rawR;
		if (r > 0.25)
		{
			++out;
			continue;
		}
		Eigen::Vector3f center = cloudPart->points[0].getVector3fMap() - cloudPart->points[0].getNormalVector3fMap() * rawR/2;

		PointT centerPoint = PointT(center.x(), center.y(), center.z());

		cloudCenters->points.push_back(centerPoint);

		std::cout << i << "-radius : "<< r << std::endl;
		
	}

	std::cout << "Error Rate : " << ((float)out)/500 <<std::endl;
	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	viewer.setBackgroundColor(0.0, 0.0, 0.5);
	viewer.addPointCloud<PointN>(cloud);
	//viewer.addPointCloudNormals<PointN>(cloud,10,0.02,"1");
	pcl::visualization::PointCloudColorHandlerCustom<PointT> rgb(cloudCenters, 255, 0, 0);
	viewer.addPointCloud(cloudCenters, rgb, "2");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "2");

	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
}