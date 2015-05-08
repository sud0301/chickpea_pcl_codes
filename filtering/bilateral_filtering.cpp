#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/filters/filter.h>
#include  <limits>

typedef pcl::PointXYZI PointT;
using namespace pcl;
using namespace std; 

char* pcl_labeled =new char[100];
 
float G (float x, float sigma)
 {
   return exp (- (x*x)/(2*sigma*sigma));
 }

 int main (int argc, char *argv[])
 {
   std::string incloudfile = argv[1];
   std::string outcloudfile = argv[2];
   float sigma_s = atof (argv[3]);
   float sigma_r = atof (argv[4]);

   // Load cloud
   	pcl::PointCloud<pcl::PointXYZRGB>::Ptr load_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
   	pcl::io::loadPCDFile (incloudfile.c_str (), *load_cloud);
   	//std::cout<<load_cloud->points.size()<<std::endl;
	pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
	pcl::copyPointCloud (*load_cloud, *cloud);
	cloud->is_dense=false ;	
	cloud->points.resize(load_cloud->points.size());
	for (size_t i=0; i < load_cloud->points.size(); i++)
	{
		/*cloud->points[i].x =load_cloud->points[i].x;
		cloud->points[i].y =load_cloud->points[i].y;
		cloud->points[i].z =load_cloud->points[i].z;*/
		cloud->points[i].intensity = load_cloud->points[i].z;
	}
	std::cout<<cloud->points.size()<<std::endl;
	sprintf(pcl_labeled, "/home/sud/Pictures/pcl-pcl-1.7.0/build/bin/b_filter.pcd");
	pcl::io::savePCDFileASCII(pcl_labeled, * cloud);
	//std::cout<<load_cloud->points.size()<<std::endl;
	int pnumber = (int)cloud->size ();
   // Output Cloud = Input Cloud
   pcl::PointCloud<PointT> outcloud = *cloud;
   // Set up KDTree
   pcl::KdTreeFLANN<PointT>::Ptr tree (new pcl::KdTreeFLANN<PointT>);
   	cout <<"hi_1"<<endl;

	tree->setInputCloud (cloud);
	cout<<"hi_2"<<endl;
   // Neighbors containers
   std::vector<int> k_indices;
   std::vector<float> k_distances;
   // Main Loop
   for (int point_id = 0; point_id < pnumber; ++point_id)
   {
     float BF = 0;
     float W = 0;

     tree->radiusSearch (point_id, 2 * sigma_s, k_indices, k_distances);

     // For each neighbor
     for (size_t n_id = 0; n_id < k_indices.size (); ++n_id)
     {
       float id = k_indices.at (n_id);
       float dist = sqrt (k_distances.at (n_id));
       float intensity_dist = abs (cloud->points[point_id].intensity - cloud->points[id].intensity);

       float w_a = G (dist, sigma_s);
       float w_b = G (intensity_dist, sigma_r);
       float weight = w_a * w_b;

       BF += weight * cloud->points[id].intensity;
       W += weight;
     }

     outcloud.points[point_id].intensity = BF / W;
   }

   // Save filtered output
   pcl::io::savePCDFile (outcloudfile.c_str (), outcloud);
   return (0);
 }
