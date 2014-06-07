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
#include<stdlib.h>
#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <opencv2/highgui/highgui.hpp>

using namespace aruco;
using namespace cv;
using namespace Eigen;


//void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,void* viewer_void);
void CallBackFunc(int event, int x, int y, int flags, void* userdata);
//int count = 0;
int indicator = 0;
int flag = 0;
Eigen::Matrix4f temp_trans;
std::vector<std::vector<float> > centre_data;

std::vector<std::vector<float> > myvector;
Vector4f Red_in_yellow,Green_in_yellow,Centre_in_yellow, Red_yellow_robot, Green_yellow_robot, Centre_yellow_robot;
Vector2f Angle_vector, norm_angle_vector;


 class SimpleOpenNIViewer
 {
   public:
//	pcl::visualization::CloudViewer viewer;
//	SimpleOpenNIViewer(): viewer ("Simple Cloud Viewer") {}
	MarkerDetector MDetector;
	vector<Marker> Markers;

   void save_cloud()
	{ // count++;
	   std::vector<float> temp_centre(3) ;
	   temp_centre.push_back(Centre_yellow_robot(0));
	   temp_centre.push_back(Centre_yellow_robot(1));
	   temp_centre.push_back(Centre_yellow_robot(2));

	   myvector.push_back(temp_centre);
	}

   void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
	 {
	   if (cv::waitKey(30) != 27)
		 {
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

		   for (unsigned int i=0;i<Markers.size();i++)
		   {
				cout<<Markers[i]<<endl;
				Markers[i].draw(image,Scalar(0,0,255),2);
		   }


	//   	    For yellow box or ID=24 co-ordinates of green and red corners

		   int row_red_in_yellow = int(Markers[0].at(0).x + 0.5);
		   int col_red_in_yellow = int(Markers[0].at(0).y + 0.5);
		   Red_in_yellow << cloud->at(row_red_in_yellow,col_red_in_yellow).x, cloud->at(row_red_in_yellow,col_red_in_yellow).y, cloud->at(row_red_in_yellow,col_red_in_yellow).z,1;


		   int row_green_in_yellow = int(Markers[0].at(1).x + 0.5);
		   int col_green_in_yellow = int(Markers[0].at(1).y + 0.5);
		   Green_in_yellow << cloud->at(row_green_in_yellow,col_green_in_yellow).x, cloud->at(row_green_in_yellow,col_green_in_yellow).y, cloud->at(row_green_in_yellow,col_green_in_yellow).z,1;


		   int row_centre_in_yellow = int(Markers[0].getCenter().x + 0.5);
		   int col_centre_in_yellow = int(Markers[0].getCenter().y + 0.5);
		   Centre_in_yellow << cloud->at(row_centre_in_yellow,col_centre_in_yellow).x, cloud->at(row_centre_in_yellow,col_centre_in_yellow).y, cloud->at(row_centre_in_yellow,col_centre_in_yellow).z,1;


		   std::cout << "Difference" << (Red_in_yellow - Green_in_yellow).norm() << std::endl;

		   Red_yellow_robot = temp_trans*Red_in_yellow;
		   Green_yellow_robot = temp_trans*Green_in_yellow;
		   Centre_yellow_robot = temp_trans*Centre_in_yellow;

		   Angle_vector << (Red_yellow_robot - Green_yellow_robot)(0), (Red_yellow_robot - Green_yellow_robot)(1);
		   norm_angle_vector = Angle_vector.normalized();


		   cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
//		   cv::setMouseCallback("Display window", CallBackFunc, NULL);
		   cv::imshow( "Display window", image );
//		   waitKey(0);

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
	           boost::this_thread::sleep (boost::posix_time::seconds (5));
	         }
//
//	   while(cv::waitKey(30) != 27)
//	          {
//	            boost::this_thread::sleep (boost::posix_time::seconds (5));
//	   //         if (indicator > 0)
//
//	   //         {
//	   //         save_cloud();
//	   //         }
//	            std::cout << "The value of the flag is" << std::endl;
//	          }

	   if(flag >= 0)
	   	            	   	   {
	   	            	   		   return;
	   	            	   	   }


	   interface->stop ();
   }


}v; //Creating object of class

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
   	   {

   		   if  ( event == EVENT_RBUTTONDOWN )
   		   {
//   			   cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
   			   v.save_cloud();
   			   flag  = flag++;
//   			   throw 10;


   		   }

   		   if  ( event == EVENT_LBUTTONDOWN )
   		  {
//   			   cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
//   			   std::cout << "do something" << std::endl;
   			   v.save_cloud();
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
	 cv::setMouseCallback("Display window", CallBackFunc, NULL);
//	 try{
	 v.run ();
//	 }
//	 catch (int e){
		 std::cout << myvector[0][0] << myvector[0][1] << myvector[0][2] << myvector[0][3] << std::endl;
//	 }

	 return 0;
 }


