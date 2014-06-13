
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <boost/thread/thread.hpp>
#include <pcl/filters/passthrough.h>
#include <string>
#include <sstream>
#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include<pcl/common/geometry.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include "opencv2/highgui/highgui.hpp"
#include<opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include<cmath>
#include<math.h>
#include<stdlib.h>
#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <opencv2/highgui/highgui.hpp>

using namespace aruco;
using namespace cv;
using namespace Eigen;

#define PI 3.14159265;
#define total_obj 8; // In our case the total number of objects is three

// Global Variables
int demonstration_count  = 1;
MarkerDetector MDetector;
vector<Marker> Markers;
int max_obj_found = 0;
std::ofstream f0, f1, f2, f3;
bool first_left_click_flag = FALSE;
bool end_flag; // FALSE if all demonstrations are taken
int time_stamp_counter = 1;
// Global data about present states of objects


void CallBackFunc(int event, int x, int y, int flags, void* userdata); // Declaring the function
void Open_Close_files(int demo_count, bool Flag);
void SaveData();



void Open_Close_files(int demo_count, bool Flag)
{
	if(Flag == TRUE)
	{
		std::stringstream oss1;
		oss1<<"/home/niladri-64/module_heisenberg/data/Demo_number_"<<demo_count<<"_info.txt";	
		std::ofstream f0(oss1.str().c_str());

		std::stringstream oss2;
		oss2<<"/home/niladri-64/module_heisenberg/data/id"<<demo_count<<"_info.txt";
		std::ofstream f1(oss2.str().c_str());

		std::stringstream oss3;
		oss3<<"/home/niladri-64/module_heisenberg/data/id"<<demo_count<<"_info.txt";
		std::ofstream f2(oss3.str().c_str());

		std::stringstream oss4;
		oss4<<"/home/niladri-64/module_heisenberg/data/id"<<demo_count<<"_info.txt";
		std::ofstream f3(oss4.str().c_str());

		std::stringstream oss5;
		oss5<<"/home/niladri-64/module_heisenberg/data/id"<<demo_count<<"_info.txt";
		std::ofstream f4(oss5.str().c_str());

		std::stringstream oss6;
		oss6<<"/home/niladri-64/module_heisenberg/data/id"<<demo_count<<"_info.txt";
		std::ofstream f5(oss6.str().c_str());

		std::stringstream oss7;
		oss7<<"/home/niladri-64/module_heisenberg/data/id"<<demo_count<<"_info.txt";
		std::ofstream f6(oss7.str().c_str());

		std::stringstream oss8;
		oss8<<"/home/niladri-64/module_heisenberg/data/id"<<demo_count<<"_info.txt";
		std::ofstream f7(oss8.str().c_str());
	}

	else if(Flag == FALSE)
	{
		f1.close();
		f2.close();
		f3.close();
		f4.close();
		f5.close();
		f6.close();
		f7.close();
		f8.close();
	}

	
}


void SaveData()
{
	if(Markers.size() > max_obj_found)
	{
		max_obj_found = Markers.size();
    }


   for(int k = 0 ; k < Markers.size(); k++)
   {
	    if(Markers[k].id == 24)
		    f3 << Centre_yellow_robot(0) << " " << Centre_yellow_robot(1) << " " <<  Centre_yellow_robot(2) << " " << angle_yellow << std::endl;
		else
			f3 << "0 " << "0 " << "0 " << "0" << std::endl; 

	    if(Markers[k].id == 12)
		    f1 << Markers[k].id << " " << Centre_white_robot(0) << " " << Centre_white_robot(1) << " " <<  Centre_white_robot(2) << " " << angle_white << " " ;
		else
			f1 << "0 " << "0 " << "0 " << "0" << std::endl; 

	   	if(Markers[k].id == 18) // DUMMY VAriable
		    f2 << Markers[k].id << " " << Centre_pink_robot(0) << " " << Centre_pink_robot(1) << " " <<  Centre_pink_robot(2) << " " << angle_pink  << " " ;
		else
			f2 << "0 " << "0 " << "0 " << "0" << std::endl; 
   }

	f1 << std::endl;
	f2 << std::endl;
	f3 << std::endl;
}



void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{

		// If right botton is clicked tha means end of a demonstration 	
		if  ( event == EVENT_RBUTTONDOWN ) 
		{
			cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
			SaveData();

			f0 << time_stamp_counter << " " << max_obj_found;
			

			if(white_flag == TRUE)
			{
			f0 << " 12";
			}
			if(pink_flag == TRUE)
			{
			f0 << " 18";
			}
			if(yellow_flag == TRUE)
			{
			f0 << " 24";

			f0 << endl;
			}	
			Open_Close_files(demo_count, FALSE);
			demonstration_count++;
			std::cout << "End of a demonstration";
			first_left_click_flag = TRUE;
			time_stamp_counter = 1;

		}

		// If left button is clicked that means demonstration is going on
		if  ( event == EVENT_LBUTTONDOWN ) 
		{
			time_stamp_counter++;
			// To check first left click after a right click
			if(first_left_click_flag == TRUE)
			{
				Open_Close_files(demo_count, TRUE);
			}
			first_left_click_flag = FALSE;

			cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
			SaveData();
		}

		// If middle botton is clicked then it would close all the presently opened (total_obj + 1) txt files and open new ones
		if ( event ==EVENT_MBUTTONDOWN)
		{
			end_flag = FALSE;
   	   	}

		}



// Class Declaration
class SimpleOpenNIViewer
 {
   	public:

   void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
	 {
	   if (cv::waitKey(30) != 27)
	   {
		   if(end_flag ==  FALSE)
		   {
			   return;
		   }

		   cv::Mat image(cloud->height, cloud->width, CV_8UC3);
		   cv::Mat depth(cloud->height, cloud->width, CV_32FC1 );

		   for (int h=0; h<image.rows; h++)
		   {
			   for (int w=0; w<image.cols; w++)
			   {
					pcl::PointXYZRGBA point = cloud->at(w, h);
					Eigen::Vector3i rgb = point.getRGBVector3i();
					image.at<cv::Vec3b>(h,w)[0] = rgb[2];
					image.at<cv::Vec3b>(h,w)[1] = rgb[1];
					image.at<cv::Vec3b>(h,w)[2] = rgb[0];
					depth.at<float>(h,w) = point.z;
			   }

		    }

		   MDetector.detect(image,Markers);

		   while(Markers.size()==0)
		   {
			   return ;
		   }

		   for (unsigned int i=0;i<Markers.size();i++)
		   {
				cout<<Markers[i]<<endl;
				Markers[i].draw(image,Scalar(0,0,255),2);
		   }


	//   	    For yellow box or ID=24 co-ordinates of green and red corners are required
	//  		For white box or ID=12  co-ordinates of green and blue corners are required
		   int marker_size = Markers.size();
		   int box_yellow = 100; // DUMMY NUMBER
		   int box_white = 100;
		   int box_pink = 100;


		   for(int i = 0; i < marker_size ; i++)
		   {
			   if(Markers[i]. id == 24) //yellow
				   box_yellow = i;
			   else if(Markers[i]. id == 12) // White
				   box_white = i;
			   else if(Markers[i]. id == 18) // Pink "DUMMY VALUE" inserted
				   box_pink = i;
		   }

		   if(box_yellow != 100)
		   {
			   yellow_flag = TRUE;
			   int row_red_in_yellow = int(Markers[box_yellow].at(0).x + 0.5);
			   int col_red_in_yellow = int(Markers[box_yellow].at(0).y + 0.5);
			   Red_in_yellow << cloud->at(row_red_in_yellow,col_red_in_yellow).x, cloud->at(row_red_in_yellow,col_red_in_yellow).y, cloud->at(row_red_in_yellow,col_red_in_yellow).z,1;
			   int row_green_in_yellow = int(Markers[box_yellow].at(1).x + 0.5);
			   int col_green_in_yellow = int(Markers[box_yellow].at(1).y + 0.5);
			   Green_in_yellow << cloud->at(row_green_in_yellow,col_green_in_yellow).x, cloud->at(row_green_in_yellow,col_green_in_yellow).y, cloud->at(row_green_in_yellow,col_green_in_yellow).z,1;
			   int row_centre_in_yellow = int(Markers[box_yellow].getCenter().x + 0.5);
			   int col_centre_in_yellow = int(Markers[box_yellow].getCenter().y + 0.5);
			   Centre_in_yellow << cloud->at(row_centre_in_yellow,col_centre_in_yellow).x, cloud->at(row_centre_in_yellow,col_centre_in_yellow).y, cloud->at(row_centre_in_yellow,col_centre_in_yellow).z,1;
			   Red_yellow_robot = temp_trans*Red_in_yellow;
			   Green_yellow_robot = temp_trans*Green_in_yellow;
			   Centre_yellow_robot = temp_trans*Centre_in_yellow;
			   Angle_vector_yellow << (Red_yellow_robot - Green_yellow_robot)(0), (Red_yellow_robot - Green_yellow_robot)(1);
			   norm_angle_vector_yellow = Angle_vector_yellow.normalized();
			   angle_yellow = atan2(norm_angle_vector_yellow(1),norm_angle_vector_yellow(0))* 180/PI;
		   }


		   if(box_white != 100)
		   {
			   white_flag = TRUE;
			   int row_blue_in_white = int(Markers[box_white].at(2).x + 0.5);
			   int col_blue_in_white = int(Markers[box_white].at(2).y + 0.5);
			   Blue_in_white << cloud->at(row_blue_in_white,col_blue_in_white).x, cloud->at(row_blue_in_white,col_blue_in_white).y, cloud->at(row_blue_in_white,col_blue_in_white).z,1;
			   int row_green_in_white = int(Markers[box_white].at(1).x + 0.5);
			   int col_green_in_white = int(Markers[box_white].at(1).y + 0.5);
			   Green_in_white << cloud->at(row_green_in_white,col_green_in_white).x, cloud->at(row_green_in_white,col_green_in_white).y, cloud->at(row_green_in_white,col_green_in_white).z,1;
			   int row_centre_in_white = int(Markers[box_white].getCenter().x + 0.5);
			   int col_centre_in_white = int(Markers[box_white].getCenter().y + 0.5);
			   Centre_in_white << cloud->at(row_centre_in_white,col_centre_in_white).x, cloud->at(row_centre_in_white,col_centre_in_white).y, cloud->at(row_centre_in_white,col_centre_in_white).z,1;
			   Blue_white_robot = temp_trans*Blue_in_white;
			   Green_white_robot = temp_trans*Green_in_white;
			   Centre_white_robot = temp_trans*Centre_in_white;
			   Angle_vector_white << (Blue_white_robot - Green_white_robot)(0), (Blue_white_robot - Green_white_robot)(1);
			   norm_angle_vector_white = Angle_vector_white.normalized();
			   angle_white = atan2(norm_angle_vector_white(1),norm_angle_vector_white(0))*180/PI;
		   }


		   if(box_pink != 100)
		   {
			   pink_flag = TRUE;
			   int row_blue_in_pink = int(Markers[box_pink].at(2).x + 0.5);
			   int col_blue_in_pink = int(Markers[box_pink].at(2).y + 0.5);
			   Blue_in_pink << cloud->at(row_blue_in_pink,col_blue_in_pink).x, cloud->at(row_blue_in_pink,col_blue_in_pink).y, cloud->at(row_blue_in_pink,col_blue_in_pink).z,1;
			   int row_green_in_pink = int(Markers[box_pink].at(1).x + 0.5);
			   int col_green_in_pink = int(Markers[box_pink].at(1).y + 0.5);
			   Green_in_pink << cloud->at(row_green_in_pink,col_green_in_pink).x, cloud->at(row_green_in_pink,col_green_in_pink).y, cloud->at(row_green_in_pink,col_green_in_pink).z,1;
			   int row_centre_in_pink = int(Markers[box_pink].getCenter().x + 0.5);
			   int col_centre_in_pink = int(Markers[box_pink].getCenter().y + 0.5);
			   Centre_in_pink << cloud->at(row_centre_in_pink,col_centre_in_pink).x, cloud->at(row_centre_in_pink,col_centre_in_pink).y, cloud->at(row_centre_in_pink,col_centre_in_pink).z,1;
			   Blue_pink_robot = temp_trans*Blue_in_pink;
			   Green_pink_robot = temp_trans*Green_in_pink;
			   Centre_pink_robot = temp_trans*Centre_in_pink;
			   Angle_vector_pink << (Blue_pink_robot - Green_pink_robot)(0), (Blue_pink_robot - Green_pink_robot)(1);
			   norm_angle_vector_pink = Angle_vector_pink.normalized();
			   angle_pink = atan2(norm_angle_vector_pink(1),norm_angle_vector_pink(0))*180/PI;
		   }

		   cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
		   cv::setMouseCallback("Display window", CallBackFunc, NULL);
		   cv::imshow( "Display window", image );

		 }
	 }




   void run ()
   {
	   pcl::Grabber* interface = new pcl::OpenNIGrabber();
	   boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
	   boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);
	   interface->registerCallback (f);
	   interface->start ();

		while (cv::waitKey(30) != 27)
		{
			if(end_flag == FLASE)
		{
		return;
		}
			boost::this_thread::sleep (boost::posix_time::seconds (5));
          	if(flag > 0)
	   	   {
	   		   return;
	   	   }
  		}
	   return;
	   interface->stop ();
   }
}
 v; //Creating object of class

int main (int argc, char **argv)
 {

	 std::ifstream infile("/home/niladri-64/module_heisenberg/data/translation_matrix.txt");
	 std::string line;
	 int i = 0;
	 while (std::getline(infile, line))
	 {
		  std::istringstream iss(line);
		  double a, b,c,d;
		  if (!(iss >> a >> b >> c >> d))
			{ break; } // error
		  temp_trans(i,0) =  a;
		  temp_trans(i,1) =  b;
		  temp_trans(i,2) =  c;
		  temp_trans(i,3) = d ;
		  i ++;
	  }
	 v.run ();
	 return 0;
 }
