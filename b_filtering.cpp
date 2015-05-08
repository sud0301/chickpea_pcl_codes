#include <pcl/point_types.h>
 #include <pcl/io/pcd_io.h>
 #include <pcl/kdtree/kdtree_flann.h>
 #include <pcl/filters/bilateral.h>
#include <boost/thread/thread.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

 typedef pcl::PointXYZI PointT;

 int
 main (int argc, char *argv[])
 {
   std::string incloudfile = argv[1];
   std::string outcloudfile = argv[2];
   float sigma_s = atof (argv[3]);
   float sigma_r = atof (argv[4]);

   // Load cloud
   pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
   pcl::io::loadPCDFile (incloudfile.c_str (), *cloud);

   pcl::PointCloud<PointT> outcloud;

   // Set up KDTree
   pcl::KdTreeFLANN<PointT>::Ptr tree (new pcl::KdTreeFLANN<PointT>);

   pcl::BilateralFilter<PointT> bf;
   bf.setInputCloud (cloud);
   bf.setSearchMethod (tree);
   bf.setHalfSize (sigma_s);
   bf.setStdDev (sigma_r);
   bf.filter (outcloud);

   // Save filtered output
   pcl::io::savePCDFile (outcloudfile.c_str (), outcloud);
   return (0);
 }
