#include <iostream>
#include <stdio.h>
#include <string.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>

using namespace std;
using namespace pcl;

int get_min(double dis[], int size );

int main(int agrc, char **argv )
{
	ifstream file_rgb("/home/sud/Documents/MATLAB/rgbd_dataset_freiburg1_xyz/combined.csv", std::ios_base::in);	
	double time_stamp, t_extra; string address, a_extra ;
	double rgb_time[793]; int i=1;
	
	while(	file_rgb >> time_stamp>>address>>t_extra>>a_extra)
	{
		rgb_time[i]=time_stamp;
//		printf("%lf\n", rgb_time[i]);
		time_stamp=0;
		i++;
	}
	
	ifstream file_gt("/home/sud/Documents/MATLAB/rgbd_dataset_freiburg1_xyz/gt.csv", std::ios_base::in);	
	double time_gt;	 std::string tx; int j =1; double gt_time[3001];	
	while(file_gt>>time_gt>>tx)
	{
		gt_time[j]=time_gt;
		printf("%lf\n", gt_time[j]);
		time_gt=0;
		j++;
	}
		
	double diff[3001];
	std::fill_n(diff,3001,100);
	double index[793];
	
	for (int i=1; i <= 792; i++)
	{	
		for (int j=i+350; j < 3001; j++)
		{
			diff[j]= (rgb_time[i]-gt_time[j])*(rgb_time[i]-gt_time[j]);
			index[i]=get_min(diff,3001);
		}
		cout<<i<<"      "<<index[i]<<endl;
	}
	
	ofstream file_index("/home/sud/Documents/MATLAB/rgbd_dataset_freiburg1_xyz/trajectory_association.csv");
	if(file_index.is_open())
	{
		for (int i=1; i <793; i++)
		file_index<<i<<" "<<index[i]<<"\n";
	}
	file_index.close();
	cout<<"BYE"<<endl;
	return 0;
}

int get_min(double dis[], int size)
{
	int i;
	float min;
        int min_i=0;
	min=dis[0];
	for (i=1; i<size; i++)
	{	
		if (dis[i] < min)
		{
		 	min=dis[i];			
			min_i=i;			
		}	
	} 
	return min_i;
}

