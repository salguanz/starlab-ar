#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/io.h>

#include <random>

namespace Model
{
	void ExtractPipels(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals, pcl::PointCloud<pcl::PointXYZ> &cylinder_centers, int pipel_num = 100, bool optimize = true)
	{
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		pcl::ExtractIndices<pcl::PointXYZ> extract_cloud;
		pcl::ExtractIndices<pcl::Normal> extract_normals;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_copy(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::Normal>::Ptr normals_copy(new pcl::PointCloud<pcl::Normal>);

		// create segmentation instance every iteration in order to set random seed
		pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg(true);
		seg.setOptimizeCoefficients(optimize);
		seg.setModelType(pcl::SACMODEL_CYLINDER);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setNormalDistanceWeight(0.1);
		seg.setMaxIterations(10000);
		seg.setDistanceThreshold(0.05);
		seg.setRadiusLimits(0, 3);

		std::default_random_engine rng1(1);
		std::default_random_engine rng2(rng1);

		for (int i = 0; i < pipel_num; ++i)
		{
			// create copies
			copyPointCloud(*cloud, *cloud_copy);
			copyPointCloud(*normals, *normals_copy);

			//std::shuffle(cloud_copy->points.begin(), cloud_copy->points.end(), rng1);
			//std::shuffle(normals_copy->points.begin(), normals_copy->points.end(), rng2);

			do
			{
				// do segmentation
				seg.setInputCloud(cloud_copy);
				seg.setInputNormals(normals_copy);
				seg.segment(*inliers, *coefficients);
				cylinder_centers.points.push_back(pcl::PointXYZ(coefficients->values[0], coefficients->values[1], coefficients->values[2]));
				std::cerr << "[" << i << "]" << std::endl;
				std::cerr << "Cylinder coefficients: " << *coefficients << std::endl;

				// remove inliers from the cloud
				extract_cloud.setInputCloud(cloud_copy);
				extract_cloud.setIndices(inliers);
				extract_cloud.setNegative(true);
				extract_cloud.filter(*cloud_copy);

				extract_normals.setInputCloud(normals_copy);
				extract_normals.setIndices(inliers);
				extract_normals.setNegative(true);
				extract_normals.filter(*normals_copy);
			} 
			// until all cylinders found
			while (inliers->indices.size() > 0);
		}
	}
}