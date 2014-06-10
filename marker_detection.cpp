#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <boost/thread/thread.hpp>
#include <pcl/filters/passthrough.h>
#include <string>
#include <sstream>

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include<pcl/common/geometry.h>

#include <pcl/filters/statistical_outlier_removal.h>

#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>

// EuclIdean Clustering
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


// Opencv

#include "opencv2/highgui/highgui.hpp"
#include<opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include<cmath>
#include<stdlib.h>

// Aruco
#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <opencv2/highgui/highgui.hpp>

using namespace aruco;
using namespace cv;
using namespace Eigen;


void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,void* viewer_void);

int count = 0;
int indicator = 0;


 class SimpleOpenNIViewer
 {
   public:
	MarkerDetector MDetector;
	vector<Marker> Markers;

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

//    	   std::cout << Markers <<std::endl;
//    	   	std::cout << "Marker Size is " << Markers.size() << std::endl;
//
//    	   	// For only one marker
//    	   	{
//    	   	std::cout << "Red Co-ordinates are " << Markers[0].at(0).x << " " << Markers[0].at(0).y <<std::endl;
////    	   	std::cout << "Green Co-ordinates are " << Markers[0].at(1).x << " " << Markers[0].at(1).y <<std::endl;
////    	   	std::cout << "Blue Co-ordinates are " << Markers[0].at(2).x << " " << Markers[0].at(2).y <<std::endl;
////    	   	std::cout << "No-color Co-ordinates are " << Markers[0].at(3).x << " " << Markers[0].at(3).y <<std::endl;
//
////    	   	float del_Y = Markers[0].at(0).y - Markers[0].at(1).y;
////    	   	float del_X = Markers[0].at(0).x - Markers[0].at(1).x;
//
//    	   	// Unit vector along x axis of end effector in camera frame
//    	   	Eigen::Vector2f Cam_x_axis , Cam_x_axis_norm;
//    	   	Cam_x_axis(0) = (Markers[0].at(0).y - Markers[0].at(1).y);
//    	   	Cam_x_axis(1) = (Markers[0].at(0).x - Markers[0].at(1).x);
//
//    	   	Cam_x_axis_norm = Cam_x_axis.normalized();
//    	   	std::cout << "abc " <<Markers.empty()<< std::endl;
//    	   	std::cout << "efg " <<Markers[0].empty()<< std::endl;
//
//    	   	std::cout << "Direction of end-effector x axis in camera frame is " << Cam_x_axis_norm << std::endl;
//    	   	std::cout << "Centre of the box  in camera frame is " << Markers[0].getCenter() << std::endl;
//    	   	// Converting the x y co-ordinates from float to int
//    	   	int centre_x = int(Markers[0].getCenter().x);
//    	   	int centre_y = int(Markers[0].getCenter().y);
//
//    	   	std::cout << "Depth of the Centre of the box  in camera frame is " << cloud->at(centre_x,centre_y).z << std::endl;


 //   	    For yellow box or ID=24 co-ordinates of green and red corners
    	   Vector4f Red_in_yellow ;
    	   int row_red_in_yellow = int(Markers[0].at(0).x + 0.5);
    	   int col_red_in_yellow = int(Markers[0].at(0).y + 0.5);
    	   Red_in_yellow << cloud->at(row_red_in_yellow,col_red_in_yellow).x, cloud->at(row_red_in_yellow,col_red_in_yellow).y, cloud->at(row_red_in_yellow,col_red_in_yellow).z,1;

    	   Vector4f Green_in_yellow ;
    	   int row_green_in_yellow = int(Markers[0].at(1).x + 0.5);
    	   int col_green_in_yellow = int(Markers[0].at(1).y + 0.5);
    	   Green_in_yellow << cloud->at(row_green_in_yellow,col_green_in_yellow).x, cloud->at(row_green_in_yellow,col_green_in_yellow).y, cloud->at(row_green_in_yellow,col_green_in_yellow).z,1;

    	   Vector4f Centre_in_yellow ;
    	   int row_centre_in_yellow = int(Markers[0].getCenter().x + 0.5);
    	   int col_centre_in_yellow = int(Markers[0].getCenter().y + 0.5);
    	   Centre_in_yellow << cloud->at(row_centre_in_yellow,col_centre_in_yellow).x, cloud->at(row_centre_in_yellow,col_centre_in_yellow).y, cloud->at(row_centre_in_yellow,col_centre_in_yellow).z,1;


//    	   std::cout << "Difference" << (Red_in_yellow - Green_in_yellow).norm() << std::endl;
    	   	// Transforming from camera frame to robot frame
    	    Eigen::Matrix4f temp_trans;
    	   	std::ifstream infile("/home/niladri-64/module_heisenberg/data/translation_matrix.txt");

    	   	std::string line;

    	   	  int i = 0;
    	   	while (std::getline(infile, line))
    	   	      {
    	   	          std::istringstream iss(line);
    	   	          double a, b,c,d;
    	   	          if (!(iss >> a >> b >> c >> d))
    	   	            { break; } // error
    	   	          //  std::cout << a << " " << b << " " << c << " " << d <<std::endl;
    	   	          // process pair (a,b)

    	   	              temp_trans(i,0) =  a;
    	   	              temp_trans(i,1) =  b;
    	   	              temp_trans(i,2) =  c;
    	   	              temp_trans(i,3) = d ;


    	   	      i ++;
    	   	      }

   // 	   	std::cout << temp_trans << std::endl ;
//    	   	Vector4f Red_yellow_robot, Green_yellow_robot, Centre_yellow_robot;
//    	   	Red_yellow_robot = temp_trans*Red_in_yellow;
//    	   	Green_yellow_robot = temp_trans*Green_in_yellow;
//    	   	Centre_yellow_robot = temp_trans*Centre_in_yellow;
//
//    	   	std::cout << "Red spot " << Red_yellow_robot << std::endl;
//    	   	std::cout << "Green spot " << Green_yellow_robot << std::endl;
//    	   	std::cout << "Green Red diff" << (Red_yellow_robot - Green_yellow_robot) << std::endl;
//    	   	std::cout << "Difference" << (Red_yellow_robot - Green_yellow_robot).norm() << std::endl;
//
////
//    	   	Vector2f Angle_vector, norm_angle_vector;
//    	   	Angle_vector << (Red_yellow_robot - Green_yellow_robot)(0), (Red_yellow_robot - Green_yellow_robot)(1);
//    	   	norm_angle_vector = Angle_vector.normalized();



//    	   	// First transforming the centre
//    	   	Vector4f centre_box_robot, temp_centre_box_cam;
//    	   	temp_centre_box_cam << cloud->at(centre_x,centre_y).x,  cloud->at(centre_x,centre_y).y, cloud->at(centre_x,centre_y).z, 1;
//    	   	std::cout << "Fine upto this" << std::endl;
//    	   	centre_box_robot = temp_trans*temp_centre_box_cam;
//
//
//
//    	   	// Transforming the x axis of the end effector
//    	   	Vector4f tmp_x_axis_box_robot, temp_x_axis_box_cam, x_axis_box_robot;
//    	   	temp_x_axis_box_cam << Cam_x_axis_norm(0), Cam_x_axis_norm(1), 0, 1;
//
//    	   	tmp_x_axis_box_robot = temp_trans*temp_x_axis_box_cam;
//    	   	// normalizing taking only x and y
//    	   	Vector2f tmp_x_axis_robot ;
//    	   	tmp_x_axis_robot << tmp_x_axis_box_robot(0), tmp_x_axis_box_robot(1);
//
//
//    	   	x_axis_box_robot << (tmp_x_axis_robot.normalized())(0),(tmp_x_axis_robot.normalized())(1),0,1;
//
//
////    	   	// End effector co-ordinates
//    	    std::ofstream file_cartesian("/home/niladri-64/module_heisenberg/data/end_effector_cartesian.txt");
//    	    if (file_cartesian.is_open())
//    	      {
//    	        file_cartesian << Centre_yellow_robot(0) << " " << Centre_yellow_robot(1)  << " "<<  Centre_yellow_robot(2)+ 0.125 << " 0 0 -1 " <<  -norm_angle_vector(1) << " " << norm_angle_vector(0) << " " <<"0 " << std::endl  ;
//    	      }
//
//    	    file_cartesian.close();
////
////    		}

    	   cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
    	   cv::imshow( "Display window", image );

         }
     }

     void run ()
     {
       pcl::Grabber* interface = new pcl::OpenNIGrabber();

       boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
       boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

       interface->registerCallback (f);

//       viewer.registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);

       interface->start ();

//       while (!viewer.wasStopped())
       while(cv::waitKey(30) != 27)
       {
         boost::this_thread::sleep (boost::posix_time::seconds (5));
//         if (indicator > 0)
//         {
//         save_cloud();
//         }
       }

       interface->stop ();
     }


 }v; //Creating object of class

////Defining callback
//void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,void* viewer_void)
//{
//
//  if (event.getKeySym () == "s" && event.keyDown ())
//  {
//	  indicator++;
//    std::cout << "s was pressed : saving point cloud" << std::endl;
//    v.save_cloud();
//  }
//}



// Main function
 int main (int argc, char **argv)
 {
   v.run ();
   return 0;
 }
