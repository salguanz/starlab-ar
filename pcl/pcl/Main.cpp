#pragma warning (disable:4996)

#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);
VTK_MODULE_INIT(vtkRenderingVolumeOpenGL2);

#include "SphereMatching.hpp"
#include "Pipel.hpp"

int main(int argc, char **argv)
{
    // load point cloud
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::iterator i;
    pcl::io::loadPCDFile("Pcd_convert.pcd", *cloud);
    
    /*for (i = cloud->begin(); i != cloud->end(); i= i+1000)
    {
        std::cout << (*i).x << " " << (*i).y << " " << (*i).z << std::endl;
    }*/
    //CylinderFittingProcess(cloud);
    //SphereFittingProcess(ScalePointCloud(cloud, 50), 3000);
    //NormalSphereFittingProcess(ScalePointCloud(cloud, 10), 500);
    

    Godeunger(ScalePointCloud(cloud, 1));
    
    //std::default_random_engine rng1(1);
    //std::shuffle(cloud->points.begin(), cloud->points.end(), rng1);

    //pcl::visualization::pclvisualizer viewer("pcl viewer");
    //viewer.setbackgroundcolor(0.0, 0.0, 0.5);
    //viewer.addpointcloud(cloud, "1");

    //while (!viewer.wasstopped())
    //{
    //    viewer.spinonce();
    //}
    
    return 0;
}