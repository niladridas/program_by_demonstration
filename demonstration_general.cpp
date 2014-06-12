/*
 * demonstration.cpp
 *
 *  Created on: Jun 6, 2014
 *      Author: niladri-64
 */

/* With each key stroke it would save following data:
 * 1. Absolute position for the object after a demonstration step is completed
 * We will assume the initial condition to be the first demonstration step
 * 2. Absolute orientation for the object
 * The data will be used for creating the matrix
 * It is assumed that one objective in terms of either changing position or orientation is completed in one move
 * With a final key stroke it would stop learning
 * This is done only for learning either relative position change or absolute position change for only one object
*/

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

float zHeight = 0.13;
float xHeight = 0.03;

void CallBackFunc(int event, int x, int y, int flags, void* userdata);

int indicator = 0;
int flag = 0;

Eigen::Matrix4f temp_trans;

int max_obj_found = 0;
//std::vector<int> id_vector;
bool yellow_flag, white_flag, pink_flag;
Vector4f Red_in_yellow,Green_in_yellow,Centre_in_yellow, Red_yellow_robot, Green_yellow_robot, Centre_yellow_robot ;
Vector4f Blue_in_white,Green_in_white,Centre_in_white, Blue_white_robot, Green_white_robot, Centre_white_robot ;
Vector4f Blue_in_pink,Green_in_pink,Centre_in_pink, Blue_pink_robot, Green_pink_robot, Centre_pink_robot ;
Vector2f Angle_vector_yellow, Angle_vector_white, Angle_vector_pink,  norm_angle_vector_yellow, norm_angle_vector_white, norm_angle_vector_pink;
float angle_yellow, angle_white, angle_pink;

std::ofstream f1("/home/niladri-64/module_heisenberg/data/demonstration.txt");
std::ofstream f2("/home/niladri-64/module_heisenberg/data/end_effector_cartesian2.txt");
class SimpleOpenNIViewer
 {
   	public:

	MarkerDetector MDetector;
	vector<Marker> Markers;

   void save_cloud()
	{
	   if(Markers.size() > max_obj_found)
	   {
		   max_obj_found = Markers.size();
	   }


	   for(int k = 0 ; k < Markers.size(); k++)
	   {
		   if(Markers[k].id == 24)
		   { f1 << Markers[k].id << " " << Centre_yellow_robot(0) << " " << Centre_yellow_robot(1) << " " <<  Centre_yellow_robot(2) << " " << angle_yellow << " "   ;
		   f2 << Centre_yellow_robot(0) << " " << Centre_yellow_robot(1) << " " <<  ( Centre_yellow_robot(2) + zHeight )  ;
		   f2 << " 0 0 -1 0 1 0";
		   }
		   else if(Markers[k].id == 12)
			   f1 << Markers[k].id << " " << Centre_white_robot(0) << " " << Centre_white_robot(1) << " " <<  Centre_white_robot(2) << " " << angle_white << " " ;

		   else if(Markers[k].id == 18) // DUMMY VAriable
			   f1 << Markers[k].id << " " << Centre_pink_robot(0) << " " << Centre_pink_robot(1) << " " <<  Centre_pink_robot(2) << " " << angle_pink  << " " ;

	   }
	   f1 << endl;
	   f2<<endl;

	}

   void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
	 {
	   if (cv::waitKey(30) != 27)
	   {
		   if(flag > 0)
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
		   if(flag > 0)
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

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
   	   {

   		   if  ( event == EVENT_RBUTTONDOWN )
   		   {
   			   cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
   			   v.save_cloud();
   			f1 <<"0" << std::endl;
   			   //throw 10;


   		   }

   		   if  ( event == EVENT_LBUTTONDOWN )
   		  {
   			   cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
//   			   std::cout << "do something" << std::endl;
   			   v.save_cloud();
   		  }

   		   if ( event ==EVENT_MBUTTONDOWN)
   		   {
   			  flag  = flag++;

   			  f1 << max_obj_found;

   			  if(white_flag == TRUE)
   			  {
   				  f1 << " 12";
   			  }
   			  if(pink_flag == TRUE)
   			  {
   				  f1 << " 18";
   			  }
   			 if(yellow_flag == TRUE)
   			  {
   			      f1 << " 24";
   			  }
   			 f1 << endl;
   		   }

   	   }


// Main function
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
	f1.close();

	 return 0;
 }


