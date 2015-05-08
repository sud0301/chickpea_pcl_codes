#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
using namespace std;

int
main (int argc,
      char** argv)
/*{
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>);
	  
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGB>);

if (pcl::io::loadPCDFile (argv[1], *cloud_in) < 0)
                       	{
                                std::cout << "Error loading point cloud " << argv[1] << std::endl << std::endl;
                                return -1;
                        }
if (pcl::io::loadPCDFile (argv[2], *cloud_out) < 0)
                        {
                                std::cout << "Error loading point cloud " << argv[1] << std::endl << std::endl;
                                return -1;
                        }
*

  // Fill in the CloudIn data
  cloud_in->width    = 5;
  cloud_in->height   = 1;
  cloud_in->is_dense = false;
  cloud_in->points.resize (cloud_in->width * cloud_in->height);
  for (size_t i = 0; i < cloud_in->points.size (); ++i)
  {
    cloud_in->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_in->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_in->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }
  std::cout << "Saved " << cloud_in->points.size () << " data points to input:"
      << std::endl;
  for (size_t i = 0; i < cloud_in->points.size (); ++i) std::cout << "    " <<
      cloud_in->points[i].x << " " << cloud_in->points[i].y << " " <<
      cloud_in->points[i].z << std::endl;
  *cloud_out = *cloud_in;
  std::cout << "size:" << cloud_out->points.size() << std::endl;
  for (size_t i = 0; i < cloud_in->points.size (); ++i)
    cloud_out->points[i].x = cloud_in->points[i].x + 0.7f;
  std::cout << "Transformed " << cloud_in->points.size () << " data points:"
      << std::endl;
  for (size_t i = 0; i < cloud_out->points.size (); ++i)
    std::cout << "    " << cloud_out->points[i].x << " " <<
      cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;
*/  
/*	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
  icp.setInputCloud(cloud_in);
  icp.setInputTarget(cloud_out);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr Final (new pcl::PointCloud<pcl::PointXYZRGB>()) ;
  icp.align(*Final);
  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;
	
	string output_add_3 = "/home/sud/Pictures/pcl-pcl-1.7.0/build/codes/output_icp.pcd";
        pcl::io::savePCDFileASCII(output_add_3, *Final);

 return (0);
}*/

  // The point clouds we will be using
{  	PointCloudT::Ptr cloud_in (new PointCloudT);  // Original point cloud
  	PointCloudT::Ptr cloud_tr (new PointCloudT);  // Transformed point cloud
  	PointCloudT::Ptr cloud_icp (new PointCloudT);  // ICP output point cloud
	
	if (pcl::io::loadPCDFile (argv[1], *cloud_in) < 0)
        {
               	std::cout << "Error loading point cloud " << argv[1] << std::endl << std::endl;
                return -1;
        }
	if (pcl::io::loadPCDFile (argv[2], *cloud_tr) < 0)
        {
                std::cout << "Error loading point cloud " << argv[2] << std::endl << std::endl;
                return -1;
        }

  
	int iterations = 20;  // Default number of ICP iterations
  
  	pcl::IterativeClosestPoint<PointT, PointT> icp;
  	icp.setMaximumIterations (iterations);
  	icp.setInputSource (cloud_in);
  	icp.setInputTarget (cloud_tr);
//  	icp.setTransformationEpsilon (1e-8);
	icp.align (*cloud_icp);
//  	icp.setMaximumIterations (20);  // We set this variable to 1 for the next time we will call .align () function
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
	*final_cloud = *cloud_tr;
        *final_cloud += *cloud_icp;
	
	string output_add_1 = "/home/sud/Pictures/pcl-pcl-1.7.0/build/codes/output_icp.pcd";
        pcl::io::savePCDFileASCII(output_add_1, *final_cloud);

  return (0);
}
