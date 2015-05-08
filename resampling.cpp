#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

int
main (int argc, char** argv)
{
  // Load input file into a PointCloud<T> with an appropriate type
 	pcl::PointCloud<pcl::PointXYZRGB>::Ptr load_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  // Load bun0.pcd -- should be available with the PCL archive in test 
  	std::string load_cl = argv[1];
  	pcl::io::loadPCDFile (load_cl, *load_cloud);
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>); 
	
	pcl::copyPointCloud (*load_cloud, *cloud);
        cloud->is_dense=false ;
        cloud->points.resize(load_cloud->points.size()); 
	pcl::io::savePCDFile("dfuq.pcd",*cloud);
  // Create a KD-Tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

  // Output has the PointNormal type in order to store the normals calculated by MLS
  pcl::PointCloud<pcl::PointNormal> mls_points;

  // Init object (second point type is for the normals, even if unused)
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
 
  mls.setComputeNormals (true);

  // Set parameters
  mls.setInputCloud (cloud);
  mls.setPolynomialFit (true);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (0.03);

  // Reconstruct
  mls.process (mls_points);

  // Save output
  pcl::io::savePCDFile ("bun0-mls.pcd", mls_points);
}