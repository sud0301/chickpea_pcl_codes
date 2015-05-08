#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <iomanip>

using namespace std;
using namespace cv;

int main()
{
	//cv::Mat img=cv::imread("/home/sud/Desktop/depth_image.tiff");//depth_image/0.pgm");
	cv::Mat img = cv::imread("/home/sud/Desktop/depth_image/42_rpm_A/1427110969_178_depth.png",CV_LOAD_IMAGE_ANYDEPTH);
	//assert(img.type() == CV_16UC1);
//	cv::Mat img;
//	img.convertTo(img_f,CV_16UC1);
	
	double minv,maxv;

	cv::minMaxLoc(img,&minv,&maxv);
	cout << "max: " << maxv << endl;
//	cout << img.at<int>(1,1)<< endl;
//	for (int i =0; i < img.rows; i++)
//	cout << img.at<unsigned short>(i,40)<< endl;
	cout << img.rows<<"	" <<img.cols << endl;
	cv:Mat img_mod(img.rows, img.cols, CV_8UC1);
	
	for (int i=1; i < img.rows; i++)
	{	for (int j=1; j <img.cols; j++)
		{	
			int val=img.at<unsigned short>(i,j);
		//	cout<<val;		
			int val_img = val*255/maxv;
		//	cout<<val_img;
			img_mod.at<uchar>(i,j)=val_img;
		}
	}
	//for (int k =0; k < img.rows; k++)
       // cout << img.at<unsigned short>(k,40)<< endl;
	
	cv::imshow("HI",img_mod);
	cv::waitKey(0);
	//if(img.type()==CV_8UC1)
//		cout <<"HI" << endl;
	return 0;
}
