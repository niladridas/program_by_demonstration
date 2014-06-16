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
int time_stamp = 0;
float zHeight = 0.13;
float xHeight = 0.03;
int indicator = 0;
int flag = 0;
int max_obj_found = 0;
int  flag_obj[8] = {0,0,0,0,0,0,0,0} ;


//bool flag_obj1 = FALSE;
//bool flag_obj2 =  FALSE;
//bool flag_obj3 = FALSE;
//bool flag_obj4 = FALSE;
//bool flag_obj5 = FALSE;
//bool flag_obj6 = FALSE;
//bool flag_obj7 =FALSE;
//bool flag_obj8 = FALSE;

Vector4f Red_in_camera, Green_in_camera, Center_in_camera, Red_in_robot, Green_in_robot, Centre_in_robot ;
Vector2f Angle_vector, norm_angle_vector;

float angle;
const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA> );
void CallBackFunc(int event, int x, int y, int flags, void* userdata);

Eigen::Matrix4f temp_trans;
//std::ofstream f0, f1, f2, f3 ,f4, f5, f6, f7, f8,  f_tmp;
std::ofstream f_tmp;

std::ofstream f0("/home/niladri-64/module_heisenberg/data/demo_info.txt");

std::ofstream f1("/home/niladri-64/module_heisenberg/data/id10.txt");

//	 std::stringstream oss3;
//	 oss3<<"/home/niladri-64/module_heisenberg/data/id12.txt";
	 std::ofstream f2("/home/niladri-64/module_heisenberg/data/id12.txt");

//	 std::stringstream oss4;
//	 oss4<<"/home/niladri-64/module_heisenberg/data/id14.txt";
	 std::ofstream f3("/home/niladri-64/module_heisenberg/data/id14.txt");

//	 std::stringstream oss5;
//	 oss5<<"/home/niladri-64/module_heisenberg/data/id16.txt";
	 std::ofstream f4("/home/niladri-64/module_heisenberg/data/id16.txt");

//	 std::stringstream oss6;
//	 oss6<<"/home/niladri-64/module_heisenberg/data/id18.txt";
	 std::ofstream f5("/home/niladri-64/module_heisenberg/data/id18.txt");

//	 std::stringstream oss7;
//	 oss7<<"/home/niladri-64/module_heisenberg/data/id20.txt";
	 std::ofstream f6("/home/niladri-64/module_heisenberg/data/id20.txt");

//	 std::stringstream oss8;
//	 oss8<<"/home/niladri-64/module_heisenberg/data/id22.txt";
	 std::ofstream f7("/home/niladri-64/module_heisenberg/data/id36.txt");

//	 std::stringstream oss9;
//	 oss9<<"/home/niladri-64/module_heisenberg/data/id24.txt";
	 std::ofstream f8("/home/niladri-64/module_heisenberg/data/id44.txt");


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
		   int row_red = int(Markers[k].at(0).x + 0.5);
		   int col_red = int(Markers[k].at(0).y + 0.5);
		   Red_in_camera << cloud->at(row_red,col_red).x, cloud->at(row_red,col_red).y, cloud->at(row_red,col_red).z,1;
		   int row_green = int(Markers[k].at(1).x + 0.5);
		   int col_green = int(Markers[k].at(1).y + 0.5);
		   Green_in_camera << cloud->at(row_green,col_green).x, cloud->at(row_green,col_green).y, cloud->at(row_green,col_green).z,1;
		   int row_centre_x = int(Markers[k].getCenter().x + 0.5);
		   int row_centre_y = int(Markers[k].getCenter().y + 0.5);
		   Center_in_camera << cloud->at(row_centre_x,row_centre_y).x, cloud->at(row_centre_x,row_centre_y).y, cloud->at(row_centre_x,row_centre_y).z,1;
		   Red_in_robot = temp_trans*Red_in_camera;
		   Green_in_robot = temp_trans*Green_in_camera;
		   Centre_in_robot = temp_trans*Center_in_camera;
		   Angle_vector << (Red_in_robot - Green_in_robot)(0), (Red_in_robot - Green_in_robot)(1);
		   norm_angle_vector= Angle_vector.normalized();
		   angle = atan2(norm_angle_vector(1),norm_angle_vector(0))* 180/PI;


		   // marker Id, then centre X, centre Y, Centre Z and orientation
		   switch (Markers[k].id)
		   {

		   		case 10:
		   		 flag_obj[0] = 1;
		   		 f1 <<  time_stamp << " " << Centre_in_robot(0) << " " << Centre_in_robot(1) << " " <<  Centre_in_robot(2) << " " << angle << std::endl;
		   			break;

			case 12:
				 flag_obj[1] = 1;
			f2 <<  time_stamp << " " << Centre_in_robot(0) << " " << Centre_in_robot(1) << " " <<  Centre_in_robot(2) << " " << angle << std::endl;
					break;

			case 14:
				 flag_obj[2] = 1;
			f3 <<  time_stamp << " " << Centre_in_robot(0) << " " << Centre_in_robot(1) << " " <<  Centre_in_robot(2) << " " << angle << std::endl;
									break;

			case 16:
				 flag_obj[3] = 1;
			f4 <<  time_stamp << " " << Centre_in_robot(0) << " " << Centre_in_robot(1) << " " <<  Centre_in_robot(2) << " " << angle << std::endl;
									break;


			case 18:
				 flag_obj[4] = 1;
			f5 <<  time_stamp << " " << Centre_in_robot(0) << " " << Centre_in_robot(1) << " " <<  Centre_in_robot(2) << " " << angle << std::endl;
									break;

			case 20:
				 flag_obj[5] = 1;
			f6 <<  time_stamp << " " << Centre_in_robot(0) << " " << Centre_in_robot(1) << " " <<  Centre_in_robot(2) << " " << angle << std::endl;
									break;

			case 36:
				 flag_obj[6] = 1;
			f7 <<  time_stamp << " " << Centre_in_robot(0) << " " << Centre_in_robot(1) << " " <<  Centre_in_robot(2) << " " << angle << std::endl;
									break;

			case 44:
				 flag_obj[7] = 1;
			f8 <<  time_stamp << " " << Centre_in_robot(0) << " " << Centre_in_robot(1) << " " <<  Centre_in_robot(2) << " " << angle << std::endl;
									break;


		   }
//

		   f_tmp <<  time_stamp << " " << Centre_in_robot(0) << " " << Centre_in_robot(1) << " " <<  Centre_in_robot(2) << " " << angle << std::endl;

	   }


	}

   void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud2)
	 {

	   if (cv::waitKey(30) != 27)
	   {
		   if(flag > 0)
		   {
			   return;
		   }

		   cv::Mat image(cloud2->height, cloud2->width, CV_8UC3);
		   cv::Mat depth(cloud2->height, cloud2->width, CV_32FC1 );

		   for (int h=0; h<image.rows; h++)
		   {
			   for (int w=0; w<image.cols; w++)
			   {
					pcl::PointXYZRGBA point = cloud2->at(w, h);
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
		   pcl::copyPointCloud(*cloud2,*cloud);
		   for (unsigned int i=0;i<Markers.size();i++)
		   {
				cout<<Markers[i]<<endl;
				Markers[i].draw(image,Scalar(0,0,255),2);
		   }

//		   for(int i = 0; i < Markers.size() ; i++)
//		   {
//			   if(Markers[i]. id == 10) //yellow
//				   flag_obj1 = TRUE;
//			   else if(Markers[i]. id == 12) // White
//				   box_white = i;
//			   else if(Markers[i]. id == 18) // Pink "DUMMY VALUE" inserted
//				   box_pink = i;
//		   }

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
   			 time_stamp++;
   		   }

   		   if ( event ==EVENT_MBUTTONDOWN)
   		   {
   			  flag  = flag++;
//
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

////	 std::stringstream oss1;
////	 oss1<<"/home/niladri-64/module_heisenberg/data/Demo_number_"<<demo_count<<".txt";
////	 std::ofstream f0(oss1.str().c_str());
////
////	 std::stringstream oss2;
////	 oss2<<"/home/niladri-64/module_heisenberg/data/id10.txt";
//	 std::ofstream f1("/home/niladri-64/module_heisenberg/data/id10.txt");
//
////	 std::stringstream oss3;
////	 oss3<<"/home/niladri-64/module_heisenberg/data/id12.txt";
//	 std::ofstream f2("/home/niladri-64/module_heisenberg/data/id12.txt");
//
////	 std::stringstream oss4;
////	 oss4<<"/home/niladri-64/module_heisenberg/data/id14.txt";
//	 std::ofstream f3("/home/niladri-64/module_heisenberg/data/id14.txt");
//
////	 std::stringstream oss5;
////	 oss5<<"/home/niladri-64/module_heisenberg/data/id16.txt";
//	 std::ofstream f4("/home/niladri-64/module_heisenberg/data/id16.txt");
//
////	 std::stringstream oss6;
////	 oss6<<"/home/niladri-64/module_heisenberg/data/id18.txt";
//	 std::ofstream f5("/home/niladri-64/module_heisenberg/data/id18.txt");
//
////	 std::stringstream oss7;
////	 oss7<<"/home/niladri-64/module_heisenberg/data/id20.txt";
//	 std::ofstream f6("/home/niladri-64/module_heisenberg/data/id20.txt");
//
////	 std::stringstream oss8;
////	 oss8<<"/home/niladri-64/module_heisenberg/data/id22.txt";
//	 std::ofstream f7("/home/niladri-64/module_heisenberg/data/id22.txt");
//
////	 std::stringstream oss9;
////	 oss9<<"/home/niladri-64/module_heisenberg/data/id24.txt";
//	 std::ofstream f8("/home/niladri-64/module_heisenberg/data/id24.txt");
	 v.run ();

	 int sum_one = 0 ;

	 for(int i=0; i <8; i++)
	 {
		 sum_one = sum_one + flag_obj[i];
	 }

	 f0 << time_stamp << " " << sum_one << " " ;


		 if(flag_obj[0] == 1)
			 f0 << 10 << " ";
		 if(flag_obj[1] == 1)
		 			 f0 << 12 << " ";
		 if(flag_obj[2] == 1)
		 			 f0 << 14 << " ";
		 if(flag_obj[3] == 1)
		 			 f0 << 16 << " ";
		 if(flag_obj[4] == 1)
		 			 f0 << 18 << " ";
		 if(flag_obj[5] == 1)
		 			 f0 << 20 << " " ;
		 if(flag_obj[6] == 1)
		 			 f0 << 36 << " " ;
		 if(flag_obj[7] == 1)
		 			 f0 << 44 << " " ;



	f0.close();
	f1.close();
	f2.close();
	f3.close();
	f4.close();
	f5.close();
	f6.close();
	f7.close();
	f8.close();


	 return 0;
 }


