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
#include <pcl/common/transforms.h>
#include <math.h>
#include <cmath>
#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <Eigen/Geometry>
#include <Eigen/Dense>

using namespace std;
using namespace pcl;

double q0=0, q1=0, q2=0, q3=0;
float e_rx=0, e_ry=0, e_rz=0;

void quat2euler(double q0, double q1, double q2, double q3);
Eigen::Affine3f t_matrix(float tx, float ty, float tz, float rx, float ry, float rz, float rw);
Eigen::Matrix4f tq_matrix(float tx, float ty, float tz, float rx, float ry, float rz, float rw);
//Eigen::Quaternion *quat;
//Eigen::Matrix3 rot;

int main(int argc, char** argv )
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud_1 (new pcl::PointCloud<pcl::PointXYZRGB>());
                if (pcl::io::loadPCDFile (argv[1], *source_cloud_1) < 0)
                        {
                                std::cout << "Error loading point cloud " << argv[1] << std::endl << std::endl;
                                return -1;
                        }

                pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud_2 (new pcl::PointCloud<pcl::PointXYZRGB>());
                                        if (pcl::io::loadPCDFile (argv[2], *source_cloud_2) < 0)
                        {
                                std::cout << "Error loading point cloud " << argv[2] << std::endl << std::endl;
                                return -1;
                        }

                pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud_1 (new pcl::PointCloud<pcl::PointXYZRGB>());
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud_again (new pcl::PointCloud<pcl::PointXYZRGB>());
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud_2 (new pcl::PointCloud<pcl::PointXYZRGB>());
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());

//	Eigen::Quaternion q1.Quaternion(0,0,0,0);
	int iter =1;        	
	FILE *file;
	double time=0, tx=0, ty=0, tz=0, rx=0, ry=0, rz=0, rw=0;
//	double q0=0, q1=0, q2=0, q3=0;
	file = fopen("gt.txt", "r");
	Eigen::Matrix4f transform_inv_1;
	while(iter<1200)
	{
		fscanf(file,"%lf %lf %lf %lf %lf %lf %lf %lf", &time,&tx, &ty, &tz, &rx, &ry, &rz, &rw );
		//printf("%lf %lf %lf %lf %lf %lf %lf %lf\n",time,tx,ty,tz,rx,ry,rz,rw);
	//	Eigen::Affine3f transform_1; 
		Eigen::Matrix4f transform_1;
	
		if (iter ==131)
		{
			transform_1=tq_matrix(tx,ty,tz,rx,ry,rz,rw);
		//	transform_1 = t_matrix (tx, ty, tz, rx, ry,rz, rw);
			printf("%lf %lf %lf %lf %lf %lf %lf %lf\n",time,tx,ty,tz,rx,ry,rz,rw);
			
			//pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud_1 (new pcl::PointCloud<pcl::PointXYZRGB>());
			
			transform_inv_1 = transform_1.inverse();
			cout<<"T_INV_1   "<<endl<<transform_inv_1.matrix()<<endl;
		//	transform_inv_1(0,3)=-tx;
		//	transform_inv_1(1,3)=-ty;
		//	transform_inv_1(2,3)=-tz;
	//		cout << transform_inv_1.matrix()<< endl;
			//Eigen::Matrix4f transform_final_1= transform_inv*transform_1;
	                pcl::transformPointCloud (*source_cloud_1, *transformed_cloud_1, transform_inv_1);
        	//      pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
        	//      pcl::concatenateFields(*source_cloud,*source_cloud_1, *final_cloud );   
        	//      *final_cloud = *source_cloud;   
        	//      *final_cloud += *transformed_cloud;     
                	//string output_add_1 = "/home/sud/Pictures/pcl-pcl-1.7.0/build/codes/output_1.pcd";
                	//pcl::io::savePCDFileASCII(output_add_1, *transformed_cloud_1);
		//	Eigen::Matrix4f transform_inv_again =transform_inv_1.inverse();
			pcl::transformPointCloud (*transformed_cloud_1, *transformed_cloud_again,transform_1);
		}

		if (iter==270)
		{
			printf("%lf %lf %lf %lf %lf %lf %lf %lf\n",time,tx,ty,tz,rx,ry,rz,rw);
			Eigen::Matrix4f transform_2= tq_matrix(tx, ty, tz, rx, ry,rz, rw);
			cout<<"T_2"<<endl<<transform_2.matrix()<<endl;
			//Eigen:: Vector4d q(rx, ry,rz, rw); 
			//Eigen::Matrix<double,1,4> io= q.transpose();
  			//cout<<io<<endl
			//Eigen::Quaternionf e_q(rx,ry,rz,rw);
			//Eigen::Matrix3f rot=e_q.toRotationMatrix();
			//cout<<rot<<endl;
			//Eigen::Affine3f	transform_2 = t_matrix (tx, ty, tz, rx, ry,rz, rw);	
			//Eigen::Affine3f transform_inv = transform_2.inverse();

		//	Eigen::Matrix4f transform_inv_2=transform_2.inverse();
			//transform_inv_2(0,3)=-tx;
                        //transform_inv_2(1,3)=-ty;
                        //transform_inv_2(2,3)=-tz;

			//cout<<transform_2.matrix()<<endl;
			//cout<<transform_inv_2.matrix()<<endl;
			Eigen::Matrix4f transform_final = transform_inv_1*transform_2;
			cout<<"T_FINAL"<<endl<<transform_final.matrix()<<endl;
			//Eigen::Matrix4f transform_final_2= transform_inv_2*transform_1;
			
			//Eigen::Matrix4f transform_final_1=transform_final.inverse();
			//cout << transform_2.matrix()<< endl;

			pcl::transformPointCloud (*source_cloud_2, *transformed_cloud_2, transform_final);		
			
		//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
			
			string output_add_2 = "/home/sud/Pictures/pcl-pcl-1.7.0/build/codes/output_2.pcd";
        		pcl::io::savePCDFileASCII(output_add_2, *transformed_cloud_2);	
		
		//	pcl::concatenateFields(*transformed_cloud_1,*transformed_cloud_2, *final_cloud );
                        *final_cloud = *source_cloud_1;   
                        *final_cloud += *transformed_cloud_2;
			string output_add_3 = "/home/sud/Pictures/pcl-pcl-1.7.0/build/codes/output_3.pcd";
                        pcl::io::savePCDFileASCII(output_add_3, *final_cloud);  

			// Visualization
  			printf(  "\nPoint cloud colors :  white  = original point cloud\n"
      "                        red  = transformed point cloud\n");
  			pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");

  			// Define R,G,B colors for the point cloud
  			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> source_cloud_color_handler (source_cloud_2, 255, 255, 255);
  			// We add the point cloud to the viewer and pass the color handler
  			viewer.addPointCloud (source_cloud_1, source_cloud_color_handler, "original_cloud");

  			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> transformed_cloud_color_handler (transformed_cloud_2, 230, 20, 20); // Red
  			viewer.addPointCloud (transformed_cloud_2, transformed_cloud_color_handler, "transformed_cloud");

  			viewer.addCoordinateSystem (1.0, 0);
  			viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
  			viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
  			viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
  			//viewer.setPosition(800, 400); // Setting visualiser window position

  			while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
    			viewer.spinOnce ();
  			}
		}
		iter++;
//		q0=0, q1=0, q2=0, q3=0;
  //              e_rx=0, e_ry=0, e_rz=0;	
	}	
	return 0;
}
/*
Eigen::Affine3f t_matrix(double tx, double ty, double tz, double rx, double ry, double rz, double rw)
{
		Eigen::Affine3f transform = Eigen::Affine3f::Identity();
		q0=rw; q1=ry; q2=rx; q3=rz;
		
		e_rx= (float)atan2(2* (q0 * q1 + q2 * q3),(1-2* (pow(q1,2) + pow(q2, 2) ))); //roll
        	e_ry= (float)asin(2 * (q0 * q2 - q3 * q1)); //pitch 
        	e_rz= (float)atan2(2 * (q0 * q3 + q1 * q2),(1-2*(pow(q2,2)+ pow(q3, 2)))); //yaw 

                transform.translation() << tx, ty, tz; // Define a translation of 2.5 meters on the x axis.
                transform.rotate (Eigen::AngleAxisf (e_rz, Eigen::Vector3f::UnitZ())); // The rotation matrix as before; tetha radians arround Z axis
                transform.rotate (Eigen::AngleAxisf (e_rx, Eigen::Vector3f::UnitX()));
                transform.rotate (Eigen::AngleAxisf (e_ry, Eigen::Vector3f::UnitY()));
		cout<<transform.matrix()<<endl;
		return transform;
}*/

Eigen::Matrix4f tq_matrix(float tx, float ty, float tz, float rx, float ry, float rz, float rw)
{
	//cout<<tx<<" "<<ty<<" "<<tz<<" "<<rx<<" "<<ry<<" "<<rz<<" "<<rw<<endl;
	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
	q0=rx, q1=ry, q2=rz, q3=rw;
//	q0=sqrt(rx*rx);	q1=sqrt(ry*ry); q2=sqrt(rz*rz) ; q3 =sqrt(rw*rw);
//	transform(0,3)= 1;
	//Eigen::Quaternion quat;
	//(q3,q0,q1,q2);
	//quat.setIdentity();
//	quat= new Quaternion(rw, rx, ry ,rz); 
	//Eigen::Matrix3 rot= quat.toRotationMatrix();
	//cout<<"qwddadad"<<rot<<endl;
/*	0.993991 -0.0853081  0.0685972 -0.0150148
 0.0852162   0.996353 0.00426598  0.0762148
-0.0687112  0.0016051   0.997636   0.133196
         0          0          0          1*/
	Eigen::Quaternionf e_q(rw, rx,ry,rz);
        Eigen::Matrix3f rot=e_q.toRotationMatrix();
        cout<<rot<<endl;

	transform(0,0)=rot(0,0);//(1-2*q1*q1-2*q2*q2);
	transform(0,1)=rot(0,1);//(2*q0*q1-2*q2*q3);
	transform(0,2)=rot(0,2);//(2*q0*q2+2*q1*q3);
	transform(0,3)=tx;
	transform(1,0)=rot(1,0);//2*q0*q1+2*q2*q3;
	transform(1,1)=rot(1,1);//1-2*q0*q0-2*q2*q2;
	transform(1,2)=rot(1,2);//2*q1*q2-2*q0*q3;
	transform(1,3)=ty;
	transform(2,0)=rot(2,0);//2*q0*q2-2*q1*q3;
	transform(2,1)=rot(2,1);//2*q1*q2+2*q0*q3;
	transform(2,2)=rot(2,2);//1-2*q0*q0-2*q1*q1;
	transform(2,3)=tz;
	transform(3,0)=0; 
	transform(3,1)=0; 
	transform(3,2)=0; 
	transform(3,3)=1;
//	Eigen::Matrix4f transform_inv=transform.inverse();
	//	cout << transform.matrix()<< endl;		
		return transform;	
	}


