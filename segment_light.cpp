#include <iostream>
#include <string.h>
#include <fstream>
#include <sstream>
#include <ctime>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <v4r/SegmenterLight/SegmenterLight.h>

//#include "v4r/SurfaceSegmenter/PreSegmenter.h"
//#include "v4r/SurfaceSegmenter/ModelAbstractor.h"
//#include "v4r/SurfaceSegmenter/ModelRefinement.h"
//#include "v4r/SurfaceSegmenter/Segmenter.h"

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

using namespace pcl;

char* filename =new char[100];
char* pcl_labeled =new char[100];
char* pcl_input =new char[100];


typedef unsigned long long timestamp_t;
static timestamp_t
get_timestamp()
{
	struct timeval now;
	gettimeofday (&now, NULL);
	return now.tv_usec + (timestamp_t)now.tv_sec * 1000000;
}

 

int main (int argc, char **argv)
{
//	srand(time(0));	

	timestamp_t t0=get_timestamp();	
//	for (int j=1; j<2;j++)
//	int	k=1;
//	{
//		sprintf(filename, "/home/sud/Downloads/Web/indoor_scene_color%d.pcd",k);
		
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
		if (pcl::io::loadPCDFile (argv[1], *pcl_cloud) < 0)
  		{
    			std::cout << "Error loading model cloud." << std::endl;
    			//showHelp (argv[0]);
    			return (-1);
  		}
	
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  	//if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("/home/sud/Downloads/Web/OSD-0.2/pcd/learn10.pcd", *pcl_cloud) == -1) //* load the file
	/*if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[1], *pcl_cloud) == -1) / load the file
  	{
    		PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    		return (-1);
  	}*/
	
  	std::string modelPath = "";
  	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr pcl_cloud_labeled(new pcl::PointCloud<pcl::PointXYZRGBL>);
  	segment::SegmenterLight seg(modelPath);
	seg.setFast(false);
  	seg.setDetail(0);        
  	pcl_cloud_labeled = seg.processPointCloud(pcl_cloud);
	
	std:cout<<"SIZE	"<<pcl_cloud_labeled->points.size()<<endl;
	
	for (int j=0; j<256; j++)
	{
		uint8_t r=0,g=0,b=0; 		
		int r_no=rand()%255;
		int b_no=rand()%255; 
		int g_no=rand()%255;

		for (int i=0; i<pcl_cloud_labeled->points.size(); i++ )	
		{							
			if (pcl_cloud_labeled->points[i].label==j)						
			{
				r = r_no , g = g_no, b = b_no;    // Example: Red color
				uint32_t  rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
				pcl_cloud_labeled->points[i].rgba = rgb;//*reinterpret_cast<float*>(&rgb);
			}				
		}
	} 
	
	sprintf(pcl_labeled, "/home/sud/Pictures/pcl-pcl-1.7.0/build/codes/my_setup/output.pcd");
	sprintf(pcl_input, "/home/sud/Pictures/pcl-pcl-1.7.0/build/codes/my_setup/input.pcd");
	
	sprintf(pcl_labeled, "/home/sud/Pictures/pcl-pcl-1.7.0/build/bin/output.pcd");
	pcl::io::savePCDFileASCII (pcl_labeled, *pcl_cloud_labeled);
	pcl::io::savePCDFileASCII ( pcl_input,*pcl_cloud);
//	cout <<k;
//	}
	timestamp_t t1=get_timestamp();
	double secs = (t1-t0)/1000000.0L;
	cout<<"Execution Time: "<<secs<<std::endl;
  return (0);
}
