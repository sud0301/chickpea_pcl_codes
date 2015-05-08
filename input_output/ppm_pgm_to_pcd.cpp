#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iomanip>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>



using namespace std;
using namespace cv;

char * pcl_cloud_name =new char [100];

pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbDepthToPointCloud(const cv::Mat cImg, const cv::Mat dImg, float fx, float fy, float cx, float cy);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbDepthToPointCloud(const cv::Mat cImg, const cv::Mat dImg, float fx, float fy, float cx, float cy)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud->width=dImg.cols;
	cloud->height=dImg.rows;
	cloud->is_dense=false;
	cloud->points.resize(dImg.rows*dImg.cols);
	cv::Vec3b* cImg_ptr=(cv::Vec3b*)(cImg.data);
	for(int r=0;r<dImg.rows;r++)
	{
		for(int c=0;c<dImg.cols;c++)
		{
			int index=r*dImg.cols+c;
			cv::Vec3b col_ptr=cImg_ptr[index];
			float Z=static_cast<float>(dImg.at<unsigned short>(r,c))/1000;
			float X=(c-cx)*Z/fx;
			float Y=(r-cy)*Z/fy;
			cloud->points[index].x=X;
			cloud->points[index].y=Y;
			cloud->points[index].z=Z;
			cloud->points[index].r=col_ptr[2];
			cloud->points[index].g=col_ptr[1];
			cloud->points[index].b=col_ptr[0];
		}
	}
	return cloud;
}

int main(int argc, char **argv)
{
	//cv::Mat img =cv::imread("/home/sud/Documents/MATLAB/bathroom_0001/r-1294886887.778664-289581113.ppm",CV_LOAD_IMAGE_ANYDEPTH);
	//double minv,maxv;
	//cv::minMaxLoc(img, &minv, &maxv);
	//cout<<img;
	//cout <<maxv<<std::endl;
	
	int iter =0;
//	ifstream file ("/home/sud/Documents/MATLAB/rgbd_dataset_freiburg1_xyz/combined.csv");
	ifstream file ("combined.csv");
	string value_1, value_2, value_3, value_4 ;
	
	 
/*	while(file.good())
	{
		getline(file, value_1, ' ');
		getline(file, value_2, ' ');
		getline(file, value_3, ' ');
		getline(file, value_4, '\n');
		cout<<value_2<<"	"<<value_4<<endl;
*/	 
	
	std::string add_rgb ="/home/sud/Pictures/pcl-pcl-1.7.0/build/codes/kinect_dataset/frame_20150506T173843.811635_rgb.tiff";///+value_2;
	std::string add_depth ="/home/sud/Pictures/pcl-pcl-1.7.0/build/codes/kinect_dataset/frame_20150506T173843.811635_depth.tiff";//argv[2]; // "/home/sud/Documents/MATLAB/rgbd_dataset_freiburg2_xyz/"+value_4; 	
		
	cv::Mat  c_image = cv::imread(add_rgb);
	cv::Mat  d_image = cv::imread(add_depth,CV_LOAD_IMAGE_ANYDEPTH);
//	float f_x=0.000f, f_y =0.0, c_x=0.0, c_y =0.0;
//	cout<<c_image.at<uchar>(100,100)[1]<<endl;
//	cout<<d_image<<endl;
	float f_x = 525.000f;
 	bool f_x_specified = pcl::console::find_switch (argc, argv, "-v");
  	if (f_x_specified)
    	pcl::console::parse (argc, argv, "-fx", f_x);

	 float f_y = 525.000f;
  	bool f_y_specified = pcl::console::find_switch (argc, argv, "-v");
  	if (f_y_specified)
    		pcl::console::parse (argc, argv, "-fy", f_y);

 	float c_x = 320.000f;
  	bool c_x_specified = pcl::console::find_switch (argc, argv, "-v");
  	if (c_x_specified)
    		pcl::console::parse (argc, argv, "-cx", c_x);

 	float c_y = 240.000f;
  	bool c_y_specified = pcl::console::find_switch (argc, argv, "-v");
  	if (c_y_specified)
    		pcl::console::parse (argc, argv, "-cy", c_y);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
	pcl_cloud = rgbDepthToPointCloud(c_image, d_image, f_x, f_y, c_x, c_y);
	iter++;
	std::stringstream sstm;
	sstm << iter;
	string iter_s = sstm.str();
	
	string output_add = "output.pcd";
	pcl::io::savePCDFileASCII(output_add, *pcl_cloud);	
	
	//cout 
//	}
	return 0;
	
}

