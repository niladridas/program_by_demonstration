/*
* Created on: June 13, 2014
* Author: Ankit Pensia

A large number of functions are created that so that informaton exchange between robot and machine can be done in abstractive level.
It is grown on the existingf file pick_a_object.cpp

// This file reads the instructions created by visuo spatial.cpp.
 * The format is
 * ID CommandTypeIn3Characters DataRelatedToThat eg. 12 PAC 23.345 645.4
 * ID - id of the object to be moved.
 * CommandType :- Currently PAC - Positoin Absolutely changed
 * 							PRC : - Position Relatively Changed
 * 							OAC :- Orientation Absolutley Changed
* 					Proposed in Future POC :- Position Orientation Changed
* 					so format for that will be  "ID POC PAC x y z  OAC 34";
*
*____________________________
calculateJpForPickAndPlace
* It calculates the joint positions to to reach the desired location and also a JP in which j2 angle is 0.
* The input is an array which contains the data in this format:- " X Y Z phi1 phi2 phi3 theta1 theta2 theta3 "
the joint positions are updated in the arrays also passed as parameters.
Since multiple solutions are available to reach the same location. The one which causes the minimum change in angle j0 and j2  is chosen.
Some better method can used but it is not the focus right now.

______________________________

A socket is created to pass the commands to robot. codewords used for robot arm manipulation can be found in file hand_control_complete .

//objectId is a vector which transforms from the object indices in the program to Their MArker Id.
// objecctId[from 0 to no of object] = their marker id like 12,24,44.
objecctId[Pid] = MarkerId
locationOfId[MarkerId] = objectPId

*/

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/LU>
#include <vector>
#include <cmath>
#include <fstream>
#include <math.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/LU>
#include <Eigen/Geometry>
#include<fstream>
#include <string>
#include <boost/math/constants/constants.hpp>
#include <time.h>

#include <iostream>
#include <vector>
#include <math.h>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/LU>
#include <Eigen/Geometry>
#include<fstream>
#include <string>
#include <boost/math/constants/constants.hpp>


//socket files

#include <unistd.h>
#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <netdb.h>
#include <string>
#include<fstream>


#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <boost/thread/thread.hpp>
//#include <pcl/filters/passthrough.h>
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
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/filters/passthrough.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
//#include <pcl/filters/extract_indices.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/kdtree/kdtree.h>
//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>
//#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/segmentation/extract_clusters.h>
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




using namespace Eigen;
using namespace std;

using namespace aruco;
using namespace cv;
using namespace Eigen;

//#define PI 3.14159265;
#define PiValue 3.14159265;

Vector4f Red_in_camera, Green_in_camera, Center_in_camera, Red_in_robot, Green_in_robot, Centre_in_robot ;
Vector4f infoMarker;
Vector2f Angle_vector, norm_angle_vector;
int pcl_count= 0;
int markerMID;
float angle;
const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA> );
//void CallBackFunc(int event, int x, int y, int flags, void* userdata);

Eigen::Matrix4f temp_trans;
std::ofstream f_tmp;
//void CallBackFunc(int event, int x, int y, int flags, void* userdata);
class SimpleOpenNIViewer
 {
   	public:

	MarkerDetector MDetector;
	vector<Marker> Markers;

   void save_cloud(int markerID)
	{



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
		   angle = atan2(norm_angle_vector(1),norm_angle_vector(0))* 180/PiValue;
		   if(angle<0)
			   	   angle=-angle;
//   		 f1 <<  time_stamp << " " << Centre_in_robot(0) << " " << Centre_in_robot(1) << " " <<  Centre_in_robot(2) << " " << angle << std::endl;

//	   		  std::cout << "Marker found with id = " << markerID << " " << Markers[k].id << " " << pcl_count<< std::endl;

		   if(markerID==Markers[k].id)
		   {

			   infoMarker(0)= Centre_in_robot(0);
	   			infoMarker(1)= Centre_in_robot(1);
	   			infoMarker(2)= Centre_in_robot(2);
	   			infoMarker(3)= angle;
	   			pcl_count++;
	   		  std::cout << "Marker found with id = " << markerID  << " " << pcl_count<< std::endl;
	   		  return;
		   }

	   }


	}

   void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud2)
	 {

	   if (cv::waitKey(30) != 27)
	   {


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
		   if(pcl_count>0)
		   		   		   	   return;
		   pcl::copyPointCloud(*cloud2,*cloud);
		   for (unsigned int i=0;i<Markers.size();i++)
		   {
			//	cout<<Markers[i]<<endl;
				Markers[i].draw(image,Scalar(0,0,255),2);
		   }

//

		   cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
		   save_cloud(markerMID);

//		   cv::setMouseCallback("Display window", CallBackFunc, NULL);
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
		   	   if(pcl_count>0)
		   		   	   return;
	           boost::this_thread::sleep (boost::posix_time::seconds (5));

		   	   if(pcl_count>0)
		   		   	   return;
	         }

//	   return;



	   interface->stop ();
   }


}
 v; //Creating object of class

// void CallBackFunc(int event, int x, int y, int flags, void* userdata)
//    	   {
//
//;
//
//    	   }

Matrix4f frame_transformation(float A_f, float alpha_f, float D_f, float Theta_f)
{
	Matrix4f T;
	T << cos(Theta_f), -sin(Theta_f)*cos(alpha_f), sin(Theta_f)*sin(alpha_f), A_f*cos(Theta_f),
	     sin(Theta_f), cos(Theta_f)*cos(alpha_f), -cos(Theta_f)*sin(alpha_f), A_f*sin(Theta_f),
	     0,            sin(alpha_f),               cos(alpha_f),              D_f,
	     0,            0,                          0,                         1;
	return T;
}


#include "func_inverse_kine.h"
#include "func_multi_sol.h"

#define PI 3.141592653589793
float thresholdNearDistance = 0.14;
float thresholdNearDistance_obstacle = 0.21;
float zOffset = 0.13;
float zOffsetIntermediate = 0.10;
float thresholdHeightChange = 0.07;

void sendData(std::ostringstream *osstring);
void close_grasp();
void open_grasp();
void send_home();
void sendData(std::ostringstream *osstring);
void  socket(char *msg);
void trapezoidal_close();
void close_spread();
void trapezoidal_init();
void goToJointPosition(float *jointPosition);
void waitForEnter();
void calculateJpForPickAndPlace(float *initialDetails,float *currentJointPosition, float *jointPositionFinal,float *intermediateJointPosition,float *jointPositionj20);
bool isNear(float *firstXYZ, float *secondXYZ) ;//This function returns if the 2 given cartesian points are near or not.
bool isNearRand(float *firstXYZ, float *secondXYZ) ;
bool isObjectAt(float *finalXYZ, std::vector <int> objectId,MatrixXf observationMatrix, int *targetObjectPID);
bool isObjectAtRand(float *finalXYZ, std::vector <int> objectId,MatrixXf observationMatrix, int *targetObjectPID);
bool isAnotherObjectAt(float *finalXYZ, std::vector <int> objectId,MatrixXf observationMatrix, int *targetObjectPID, int currentPID);
int locationOfId(int targetObjectMarkerId,std::vector <int> objectId);
void createDummyPositon(float *dummyXYZ, std::vector <int> objectId,MatrixXf observationMatrix, int *targetObjectPID);
void updateObservationMatrix(MatrixXf& observationMatrix, int objectMID,int objectPID);
void createFinalVector(float *finalXYZ, float angle, float *output);
void pick_and_place (float *currentDetails, float *finalDetails,std::vector <int> objectId,MatrixXf& observationMatrix,int currentMarkerId,float *currentJointPosition);




void waitForEnter() {
	std::string line;
	std::getline(std::cin, line);
}


void updateCurrentJointPosition(float *prevJointPosition, float *finalPosition)
{
	prevJointPosition[0]=	finalPosition[0];
	prevJointPosition[1]=	finalPosition[1];
	prevJointPosition[2]=	finalPosition[2];
	prevJointPosition[3]=	finalPosition[3];
	prevJointPosition[4]=	finalPosition[4];
	prevJointPosition[5]=	finalPosition[5];
	prevJointPosition[6]=	finalPosition[6];

}

void calculateJpForPickAndPlace(float *initialDetails,float *currentJointPosition, float *jointPositionFinal,float *intermediateJointPosition,float *jointPositionj20)
{

float  cartesianAndOrientation[9];
cartesianAndOrientation[0] = initialDetails[0];
cartesianAndOrientation[1] = initialDetails[1];
cartesianAndOrientation[2] = initialDetails[2];
cartesianAndOrientation[3] = 0; //initialDetails[3]
cartesianAndOrientation[4] = 0;//initialDetails[4]
cartesianAndOrientation[5] = -1;//initialDetails[5]
cartesianAndOrientation[6] = initialDetails[6];
cartesianAndOrientation[7] = initialDetails[7];
cartesianAndOrientation[8] = initialDetails[8];

std::cout << "Calculating jp for  cartesian " << cartesianAndOrientation[0] << "   " << cartesianAndOrientation [1] <<"   "<<  cartesianAndOrientation[2] << std::endl;
//std::cout << "  4 to 6  " << cartesianAndOrientation[3] << "   " << cartesianAndOrientation [4] <<"   "<<  cartesianAndOrientation[5] << std::endl;
//std::cout << "  orien " << cartesianAndOrientation[6] << "   " << cartesianAndOrientation [7] <<"   "<<  cartesianAndOrientation[8] << std::endl;

float phi_search1[2];
int solutionCounter1[1];
char fileName[100];
	func_inverse_kine(cartesianAndOrientation,phi_search1);
//std::cout <<" phi search " << phi_search[0] << phi_search[1] << std::endl;
	func_multi_sol(cartesianAndOrientation,phi_search1,solutionCounter1,fileName);
std::cout << "Solution counter " << *solutionCounter1 << " " << fileName << std::endl;

float  cartesianAndOrientationIntermediate[9];
cartesianAndOrientationIntermediate[0] = initialDetails[0];
cartesianAndOrientationIntermediate[1] = initialDetails[1];
cartesianAndOrientationIntermediate[2] = initialDetails[2]+zOffsetIntermediate;
cartesianAndOrientationIntermediate[3] = 0; //initialDetails[3]
cartesianAndOrientationIntermediate[4] = 0;//initialDetails[4]
cartesianAndOrientationIntermediate[5] = -1;//initialDetails[5]
cartesianAndOrientationIntermediate[6] = initialDetails[6];
cartesianAndOrientationIntermediate[7] = initialDetails[7];
cartesianAndOrientationIntermediate[8] = initialDetails[8];



std::cout << "Calculating jp for  cartesian " << cartesianAndOrientationIntermediate[0] << "   " << cartesianAndOrientationIntermediate [1] <<"   "<<  cartesianAndOrientationIntermediate[2] << std::endl;
//std::cout << "  4 to 6  " << cartesianAndOrientation[3] << "   " << cartesianAndOrientation [4] <<"   "<<  cartesianAndOrientation[5] << std::endl;
//std::cout << "  orien " << cartesianAndOrientation[6] << "   " << cartesianAndOrientation [7] <<"   "<<  cartesianAndOrientation[8] << std::endl;

float phi_search2[2];
int solutionCounter2[1];
char fileNameIntermediate[100];
	func_inverse_kine(cartesianAndOrientationIntermediate,phi_search2);
//std::cout <<" phi search " << phi_search[0] << phi_search[1] << std::endl;
	func_multi_sol(cartesianAndOrientationIntermediate,phi_search2,solutionCounter2,fileNameIntermediate);
std::cout << "Solution counter " << *solutionCounter2 << " " << fileNameIntermediate << std::endl;


if(*solutionCounter1==0 || *solutionCounter2==0)
	{std::cout << "No solutions found" << std::endl;
	waitForEnter();
	open_grasp();
	close_spread();
	trapezoidal_close();
	send_home();
	}


std::ifstream infile(fileName);
MatrixXf solutionSpace (*solutionCounter1,7);
std::string line;

int i = 0;
while (std::getline(infile, line))
      {
          std::istringstream iss(line);
          double a, b,c,d,e,f,g;
          if (!(iss >> a >> b >> c >> d >> e >> f >> g))
            { break; } // error
//            std::cout << a << " " << b << " " << c << " " << d <<std::endl;
          // process pair (a,b)

              solutionSpace(i,0) =  a;
              solutionSpace(i,1) =  b;
              solutionSpace(i,2) =  c;
              solutionSpace(i,3) = d ;
              solutionSpace(i,4) =  e;
              solutionSpace(i,5) =  f;
              solutionSpace(i,6) = g;


      i ++;
      }

std::ifstream infileIntermediate(fileNameIntermediate);
MatrixXf solutionSpaceIntermediate (*solutionCounter2,7);
std::string lineIntermediate;

i = 0;
while (std::getline(infileIntermediate, lineIntermediate))
      {
          std::istringstream issInter(lineIntermediate);
          double a, b,c,d,e,f,g;
          if (!(issInter >> a >> b >> c >> d >> e >> f >> g))
            { break; } // error
//            std::cout << a << " " << b << " " << c << " " << d <<std::endl;
          // process pair (a,b)

          solutionSpaceIntermediate(i,0) =  a;
          solutionSpaceIntermediate(i,1) =  b;
          solutionSpaceIntermediate(i,2) =  c;
          solutionSpaceIntermediate(i,3) = d ;
          solutionSpaceIntermediate(i,4) =  e;
          solutionSpaceIntermediate(i,5) =  f;
          solutionSpaceIntermediate(i,6) = g;


      i ++;
      }

//std::cout << solutionSpace << std::endl;

float min = 99999999999;
float dummy0,dummy1,dummy2,dummy3,dummy4,dummy5,dummy6;
int index = 0;
float error1,error2,error3,error4,error5,error6,error0;
float min_error=99999,curr_error;
for(i =0;i < *solutionCounter1; i++)
	{
	//std::cout << i << "  " << solutionSpace(i,0) << " " << min << std::endl;
	 dummy0 = solutionSpace(i,0);
	 dummy1 = solutionSpace(i,1);
	 dummy2 = solutionSpace(i,2);
	 dummy3 = solutionSpace(i,3);
	 dummy4 = solutionSpace(i,4);
	 dummy5 = solutionSpace(i,5);
	 dummy6 = solutionSpace(i,6);

    error0 = dummy0- currentJointPosition[0];
	 error1 = dummy1- currentJointPosition[1];
	 error2 = dummy2- currentJointPosition[2];
	 error3 = dummy3- currentJointPosition[3];
	 error4 = dummy4- currentJointPosition[4];
	 error5 = dummy5- currentJointPosition[5];
	 error6 = dummy6- currentJointPosition[6];

	 if(error0<0)
		 error0=-error0;
	 if(error1<0)
		 error1=-error1;
	 if(error2<0)
		 error2=-error2;
	 if(error3<0)
		 error3=-error3;
	 if(error4<0)
		 error4=-error4;
	 if(error5<0)
		 error5=-error5;
	 if(error6<0)
		 error6=-error6;

	 curr_error = 0.3*error0 + error4 + 0.3*error2 + error6 ;//error3+error4+error5+error6;
	 //	 std::cout << "curr_error " << curr_error<< "and i = " << i << " " << error4 << " " << error6<< std::endl;
	 	 if(curr_error < min_error)
	 		 {
	 //		 std::cout << "curr_error " << curr_error<< "and i = " << i << std::endl;
	 //		std::cout<< "---------------" << error0 << " " << error2 << " " << error6 <<	 std::endl;

	 		 index= i;
	 			 min_error = curr_error;
	 		 }
//	float dummy = 0.4*dummy0 +dummy2 ; //+ 0.4*dummy6;
//	if(dummy < min)
//		{
//		index = i;
//		min = dummy;
//		}
	}

std::cout << "index = " << index << std::endl;



for(i=0;i<7;i++)
{
	jointPositionFinal[i]= solutionSpace(index,i);
	jointPositionj20[i] = solutionSpace(index,i);
	}

jointPositionj20[1] = 0;

min_error=99999;
int indexIntermediate;
for(i =0;i < *solutionCounter2; i++)
	{
	 dummy0 = solutionSpaceIntermediate(i,0);
	 dummy1 = solutionSpaceIntermediate(i,1);
	 dummy2 = solutionSpaceIntermediate(i,2);
	 dummy3 = solutionSpaceIntermediate(i,3);
	 dummy4 = solutionSpaceIntermediate(i,4);
	 dummy5 = solutionSpaceIntermediate(i,5);
	 dummy6 = solutionSpaceIntermediate(i,6);
	 error0 = dummy0- jointPositionFinal[0];
	 error1 = dummy1- jointPositionFinal[1];
	 error2 = dummy2- jointPositionFinal[2];
	 error3 = dummy3- jointPositionFinal[3];
	 error4 = dummy4- jointPositionFinal[4];
	 error5 = dummy5- jointPositionFinal[5];
	 error6 = dummy6- jointPositionFinal[6];
	 if(error0<0)
		 error0=-error0;
	 if(error1<0)
		 error1=-error1;
	 if(error2<0)
		 error2=-error2;
	 if(error3<0)
		 error3=-error3;
	 if(error4<0)
		 error4=-error4;
	 if(error5<0)
		 error5=-error5;
	 if(error6<0)
		 error6=-error6;

	 curr_error = error0 + 0.9*error4 + error2 + error6 ;//error3+error4+error5+error6;
//	 std::cout << "curr_error " << curr_error<< "and i = " << i << " " << error4 << " " << error6<< std::endl;
	 if(curr_error < min_error)
		 {
//		 std::cout << "curr_error " << curr_error<< "and i = " << i << std::endl;
//		std::cout<< "---------------" << error0 << " " << error2 << " " << error6 <<	 std::endl;

		 indexIntermediate= i;
			 min_error = curr_error;
		 }
	 }

std::cout << "index intermediate = " << indexIntermediate << std::endl;
for(i=0;i<7;i++)
{
	intermediateJointPosition[i]= solutionSpaceIntermediate(indexIntermediate,i);
}
std::cout <<"min error = "<< min_error << " "  << std::endl;


}



// Client socket include files

 int status;
int socketfd ;
int nil_counter = 0;
//struct addrinfo host_info;
 //struct addrinfo *host_info_list;

    void  socket(char *msg)
    {
    	std::cout << msg << std::endl;
    	if (nil_counter == 0)
    	     {
    	      struct addrinfo host_info;
    	      struct addrinfo *host_info_list;
    	      memset(&host_info, 0, sizeof host_info);
    	      host_info.ai_family = AF_UNSPEC;
    	      host_info.ai_socktype = SOCK_STREAM;
    	      status = getaddrinfo("192.168.0.110", "5555", &host_info, &host_info_list); //192.168.0.110 this is the wam(server) address
    	      if (status != 0)  std::cout << "getaddrinfo error" << gai_strerror(status) ;
    	      socketfd = socket(host_info_list->ai_family, host_info_list->ai_socktype, host_info_list->ai_protocol);
    	      status = connect(socketfd, host_info_list->ai_addr, host_info_list->ai_addrlen);
    	      if (status == -1)
    	      {
    	    	  std::cout << "connect error" ;
    	      }
    	      else
    	    	  std::cout << "connection established \n" << std::endl;
    	      }
    	      nil_counter = nil_counter + 1;

    		std::cout << "enter the command \n" << std::endl;
    	//      std::cin.getline( msg, 100 );

    	      // instead of this input from the user

    	//   	  std::getline(infile2,line_jp);

    	     // std::cin.getline( msg, 100 );

    	      ssize_t bytes_sent;
    	      int len = (int) strlen(msg);
    	      bytes_sent = send(socketfd, (char *)msg, len, 0);
    	      std::cout << "bytes sent = "<< bytes_sent << std::endl;


    	      ssize_t bytes_recieved;
    	           char incoming_data_buffer[1000];
    	           bytes_recieved = recv(socketfd, incoming_data_buffer,1000, 0);
    	           std::cout << "incoming :-"<<incoming_data_buffer << "\n" << std::endl;

    	 	  //    ssize_t bytes_recieved;
//    	      char incoming_data_buffer[1000];
//    	      bytes_recieved = recv(socketfd, incoming_data_buffer,1000, 0);
//    	      std::cout << incoming_data_buffer << "\n" << std::endl;
    	      //std::cout << "Choose the point from the image" << "\n" << std::endl;
     }


void sendData(std::ostringstream *osstring) // It sends the data via socket to Robot.
{
	std::string	msg1 = (*osstring).str();
	std::cout << msg1 << std::endl;
	socket(&msg1[0]);
	waitForEnter();
	(*osstring).clear();
	(*osstring).str("");
}
// ________________
//Folowing few functions are self-explainatory
//_________________
void initialise_hand()
{
	std::ostringstream ss;
	ss << "o" ;
	sendData(&ss);
}

void close_grasp()
{
	std::ostringstream ss;
	ss << "c" ;
	sendData(&ss);
}
void open_grasp()
{
	std::ostringstream ss;
	ss << "e" ;
	sendData(&ss);
}
void close_spread()
{
	std::ostringstream ss;
	ss << "a" ;
	sendData(&ss);
}

void trapezoidal_close()
{
	std::ostringstream ss;
	ss << "b" ;
	sendData(&ss);
}

void send_home()
{
	std::ostringstream ss;
	ss << "h" ;
	sendData(&ss);
}

void trapezoidal_init()
{

	std::ostringstream ss;
	ss << "m 40" ;
	sendData(&ss);
}
void goToJointPosition(float *jointPosition)
{

	std::ostringstream ss;
	ss<<  "j " << jointPosition[0] << " " << jointPosition[1] << " " << jointPosition[2] << " " << jointPosition[3] << " " << jointPosition[4] << " " << jointPosition[5] << " " << jointPosition[6];
	sendData(&ss);
}

bool isNear(float *firstXYZ, float *secondXYZ) //This function returns if the 2 given cartesian points are near or not.
// It doesn't take z in account (now but can be changed). thresholdNearDistance can be changed to change the parameter.
{
	Vector3f relativeXYZ;
	relativeXYZ(0)=firstXYZ[0]-secondXYZ[0];
	relativeXYZ(1)=firstXYZ[1]-secondXYZ[1];
	relativeXYZ(2)=firstXYZ[2]-zOffset-secondXYZ[2];
	std::cout << "relative vector  " << relativeXYZ(0) << " " << relativeXYZ(1) << " " << relativeXYZ(2)  <<  std::endl;
	if(relativeXYZ(2) >=thresholdHeightChange || relativeXYZ(2) <= -thresholdHeightChange  )
	{
		std::cout << "Far because of height " << relativeXYZ(2) << std::endl;
		return false;
	}

	if(relativeXYZ.norm() < thresholdNearDistance )
	{
		std::cout << " Yes,near Relative Distance is " << relativeXYZ.norm()  << "Realtive vector is " <<relativeXYZ(0) << " " << relativeXYZ(1) <<  std::endl;
		return true;

	}
	std::cout << "No, far  Relative Distance is " << relativeXYZ.norm()  << "Realtive vector is " <<relativeXYZ(0) << " " << relativeXYZ(1) << std::endl;
	return false;
}

bool isNearRand(float *firstXYZ, float *secondXYZ) // A dummy function made just because I didnt want to print all the distances when creating dummy loccation.
{
	Vector2f relativeXYZ;
	relativeXYZ(0)=firstXYZ[0]-secondXYZ[0];
	relativeXYZ(1)=firstXYZ[1]-secondXYZ[1];
	//relativeXYZ(2)=firstXYZ[2]-secondXYZ[2];
	if(relativeXYZ.norm() < thresholdNearDistance_obstacle )
	{
	//	std::cout << " Relative Distance is " << relativeXYZ.norm()  << "Realtive vector is " <<relativeXYZ(0) << " " << relativeXYZ(1) <<  std::endl;
		return true;

	}
	// std::cout << " Relative Distance is " << relativeXYZ.norm()  << "Realtive vector is " <<relativeXYZ(0) << " " << relativeXYZ(1) << std::endl;
	return false;
}


bool isObjectAt(float *finalXYZ, std::vector <int> objectId,MatrixXf observationMatrix, int *targetObjectPID)
//A function to check if there is any object At finalXYZ.
//objectId is a vector which transforms from the object indices in the program to Their MArker Id.
// objecctId[from 0 to no of object] = their marker id like 12,24,44.
{
	int alpha;
	float alphaXYZ[3];
		for(alpha=0;alpha<objectId.size(); alpha++)
		{

			alphaXYZ[0]=observationMatrix(alpha,1);
			alphaXYZ[1]=observationMatrix(alpha,2);
			alphaXYZ[2]=observationMatrix(alpha,3);
			if(isNear(finalXYZ,alphaXYZ))
				{
				*targetObjectPID = alpha;
				return true;
				}
				;
		}

return false;
}

bool isObjectAtRand(float *finalXYZ, std::vector <int> objectId,MatrixXf observationMatrix, int *targetObjectPID)
//Same function as before but calls isNearRandom so that it doesnt print every loop
{
	int alpha;
	float alphaXYZ[3];
		for(alpha=0;alpha<objectId.size(); alpha++)
		{

			alphaXYZ[0]=observationMatrix(alpha,1);
			alphaXYZ[1]=observationMatrix(alpha,2);
			alphaXYZ[2]=observationMatrix(alpha,3);
			if(isNearRand(finalXYZ,alphaXYZ))
				{
				*targetObjectPID = alpha;
				return true;
				}
				;
		}

return false;
}
bool isAnotherObjectAt(float *finalXYZ, std::vector <int> objectId,MatrixXf observationMatrix, int *targetObjectPID, int currentPID)
//Basically same as isObjectAt But it also ignores the case if the object found at particular location is currentId
{
	int alpha;
	float alphaXYZ[3];
		for(alpha=0;alpha<objectId.size(); alpha++)
		{
			std::cout << currentPID << "  " << alpha <<  std::endl;
			if(alpha == currentPID)
				continue;
			alphaXYZ[0]=observationMatrix(alpha,1);
			alphaXYZ[1]=observationMatrix(alpha,2);
			alphaXYZ[2]=observationMatrix(alpha,3);

			if(isNear(finalXYZ,alphaXYZ))
				{
				*targetObjectPID = alpha;
				return true;
				}
				;
		}

return false;
}



int locationOfId(int targetObjectMarkerId,std::vector <int> objectId)
// it returns the PID from the marker id.
{
	int alpha;
	for(alpha=0;alpha<objectId.size(); alpha++)
		if( objectId[alpha] == targetObjectMarkerId)
				return alpha;

	return -1;

}

void createDummyPositon(float *dummyXYZ, std::vector <int> objectId,MatrixXf observationMatrix, int *targetObjectPID)
{	int seed = 1;
	float r = ((float) rand()/ RAND_MAX) ;
	float min_x = 0.35;
	float max_x = 0.7;
	float min_y = -0.3;
	float max_y = 0.41;
	dummyXYZ[0]= min_x + (max_x - min_x) * r;
	dummyXYZ[1]= min_y+ (max_y - min_y) * r;
	dummyXYZ[2]= -0.23;
	int i;
	while(isObjectAtRand(dummyXYZ,objectId,observationMatrix,targetObjectPID))
	{
		srand(time(NULL) + seed);
		 r = ((float) rand()/ RAND_MAX);

	dummyXYZ[0]= min_x + (max_x - min_x) * r;
	dummyXYZ[1]= min_y+ (max_y - min_y) * r;
	dummyXYZ[2]= -0.35;
	seed = seed + 1000;
	i++;
	}
	 std::cout << "Random no. is " << r  << " total no. of loops is "<< i << std::endl;
	// make sure no object is at dummy XYZ using isObjectAt
//put dummXYZ in the array
	//dummyXYZ[0];
}

void updateObservationMatrix(MatrixXf& observationMatrix, int objectMID,int objectPID)
{
	pcl_count=0;
	markerMID=objectMID;
	v.run();
	//v.save_cloud(markerMID);
	observationMatrix(objectPID,1)= infoMarker(0);
	observationMatrix(objectPID,2)= infoMarker(1);
	observationMatrix(objectPID,3)= infoMarker(2) ;//- zOffset;
	observationMatrix(objectPID,4)= infoMarker(3);

}
void createFinalVector(float *finalXYZ, float angle, float *output)
{
	//if(angle <0)
//	if(angle<0)
//			angle = -angle;
//	float dum1 = angle;
//	float dum2 = 180-angle;
//	angle = dum1;
//	if(dum2<dum1)
//		angle = dum2;
//	int intAngle = int(angle);
//if(intAngle <30)
//	intAngle = 0;
//else if(intAngle < 60)
//	intAngle = 45;
//else
//	intAngle = 90;
//angle = intAngle;

	//angle = int(angle)%180;
	output[0]= finalXYZ[0];
	output[1]= finalXYZ[1];
	output[2]= finalXYZ[2] + zOffset;
	output[3]= 0;
	output[4]= 0;
	output[5]= -1;
	output[6]= cos((angle + 90)*PI/180);
	output[7]= sin((angle + 90)*PI/180);
	output[8]= 0;
}

void pick_and_place (float *currentDetails, float *finalDetails,std::vector <int> objectId,MatrixXf& observationMatrix,int currentMarkerId,float *currentJointPosition)
{
	float currentJp[7];
	float currentIntermediateJointPosition[7];
	float currentJpJ20[7];
	float finalJp[7];
	float finalJpJ20[7];
	float finalIntermediateJointPosition[7];
	int obstacleObjectPID[1];
	int flag = 1;
	while(isAnotherObjectAt(finalDetails,objectId,observationMatrix,obstacleObjectPID,locationOfId(currentMarkerId,objectId)))
		{
		std::cout << " Obstacle fount with id " <<  obstacleObjectPID[0] <<" "<< objectId[obstacleObjectPID[0]]  << std::endl;
		int obstaclePID= *obstacleObjectPID;
		float obstacleXYZ[3];
		obstacleXYZ[0]=observationMatrix(obstaclePID,1);
		obstacleXYZ[1]=observationMatrix(obstaclePID,2);
		obstacleXYZ[2]=observationMatrix(obstaclePID,3);
		float obstacleAngle;
		float obstacleDetails[9];
		createFinalVector(obstacleXYZ,obstacleAngle,obstacleDetails);
		float dummyXYZ[3];
		float dummyDetails[9];
		createDummyPositon(dummyXYZ,objectId,observationMatrix,obstacleObjectPID);
		// create Dummy position
		std::cout << "Putting it at dummy XYZ " << dummyXYZ[0]<< " " << dummyXYZ[1]<<  std::endl;
		createFinalVector(dummyXYZ,obstacleAngle,dummyDetails);
		pick_and_place(obstacleDetails,dummyDetails,objectId,observationMatrix,objectId[obstaclePID],currentJointPosition);
		// then put obstacle in the dummy XYZ

		}

	std::cout << " calculating JP for grasping" << std::endl;

	calculateJpForPickAndPlace(currentDetails,currentJointPosition,currentJp,currentIntermediateJointPosition,currentJpJ20);
	goToJointPosition(currentJpJ20);
	updateCurrentJointPosition(currentJointPosition,currentJpJ20);
	//	std::cout << "initialise hand" << std::endl;
//
	initialise_hand();
	trapezoidal_init();
	goToJointPosition(currentIntermediateJointPosition);

	updateCurrentJointPosition(currentJointPosition,currentIntermediateJointPosition);

	goToJointPosition(currentJp);
	updateCurrentJointPosition(currentJointPosition,currentJp);
	close_grasp();
	goToJointPosition(currentIntermediateJointPosition);
	updateCurrentJointPosition(currentJointPosition,currentIntermediateJointPosition);
	goToJointPosition(currentJpJ20);
	updateCurrentJointPosition(currentJointPosition,currentJpJ20);


	//_____________________________________________

	calculateJpForPickAndPlace(finalDetails,currentJointPosition,finalJp,finalIntermediateJointPosition,finalJpJ20);
	goToJointPosition(finalJpJ20);
	updateCurrentJointPosition(currentJointPosition,finalJpJ20);
	//initialise_hand();
	goToJointPosition(finalIntermediateJointPosition);
	updateCurrentJointPosition(currentJointPosition,finalIntermediateJointPosition);
	goToJointPosition(finalJp);
	updateCurrentJointPosition(currentJointPosition,finalJp);
	trapezoidal_init();
	//close_grasp();
	goToJointPosition(finalIntermediateJointPosition);
	updateCurrentJointPosition(currentJointPosition,finalIntermediateJointPosition);
	goToJointPosition(finalJpJ20);
	updateCurrentJointPosition(currentJointPosition,finalJpJ20);
	open_grasp();
	updateObservationMatrix(observationMatrix,currentMarkerId,locationOfId(currentMarkerId,objectId));
	std::cout << "updated matrix" << std::endl << observationMatrix << std::endl;
	//	close_spread();
//	trapezoidal_close();
//	send_home();
}


void initialiseMatrixWithZero(MatrixXf& observationMatrix, int rows, int cols)
{
	int i,j;
for (i=0;i<rows;i++)
		for (j =0;j<cols;j++)
			{
			observationMatrix(i,j)=0;

			}
}
     using namespace std;
     using namespace Eigen;
     int main()
     {
    	 std::ifstream trans_file("/home/niladri-64/module_heisenberg/data/translation_matrix.txt");
    	 	 std::string trans_line;
    	 	 int counter = 0;
    	 	 while (std::getline(trans_file, trans_line))
    	 	 {
    	 		  std::istringstream trans_iss(trans_line);
    	 		  double a, b,c,d;
    	 		  if (!(trans_iss >> a >> b >> c >> d))
    	 			{ break; } // error
    	 		  temp_trans(counter,0) =  a;
    	 		  temp_trans(counter,1) =  b;
    	 		  temp_trans(counter,2) =  c;
    	 		  temp_trans(counter,3) = d ;
    	 		 counter ++;
    	 	  }
    	 	//v.run ();
    	 float currentJointPosition[7] = {0.00076699, -1.97339, -0.0280971, 3.17994, 0.0176329, -0.0286238, -0.0207545 };


    		std::vector <int> objectId ;


    	 int totalObjects;
    	 int time_stamps;
    	 MatrixXf observationMatrix (8,5);
    	     	 		 //   	 observationVector.resize(8);
    	     	 		    	 initialiseMatrixWithZero(observationMatrix,8,5);

    	 std::ifstream demo_infile2("/home/niladri-64/module_heisenberg/data/demo_info.txt");
    	 		std::string demo_line2;
    	 		while (std::getline(demo_infile2, demo_line2))
    	 		{
    	 			int id;
    	 		    std::istringstream demo_iss2(demo_line2);
    	 		   demo_iss2 >> time_stamps >> totalObjects ;
    	 		    for(int i=0;i<totalObjects;i++)
    	 		    				{
    	 		    	demo_iss2 >> id;
    	 		    	observationMatrix(i,0) = id;
    	 		    				objectId.push_back(id);

    	 		    				}
    	 		}


    	 std::ifstream intialFile("/home/niladri-64/module_heisenberg/data/initial_condition.txt");
   			std::string file_line;
   			int index = 0;
    			while (std::getline(intialFile, file_line))
    			{
    				int id;
    				float xyzangle[4];
    			    std::istringstream iss2(file_line);
    			    iss2 >> id >> xyzangle[0] >> xyzangle[1] >> xyzangle[2] >> xyzangle[3] ;
    			    index = locationOfId(id,objectId);
    			    //       				objectId.push_back(id);
  //     				observationMatrix(index,0)=id;
       				observationMatrix(index,1)=xyzangle[0];
       				observationMatrix(index,2)=xyzangle[1];
       				observationMatrix(index,3)=xyzangle[2];
       				observationMatrix(index,4)=xyzangle[3]; // angle
       				//index++;
    			}





    	 std::ifstream myReadFile("/home/niladri-64/module_heisenberg/data/robot_instruct.txt");

    	 std::string line2;
    	// std::getline(myReadFile, line2);
    	 std::istringstream iss(line2);

    	 cout << "Obs matrix " << endl;
    	  cout << observationMatrix << endl;


    	 while (std::getline(myReadFile, line2))
    	 {
    	 	// std::cout << line2
    	     std::istringstream iss(line2);
    	    int id;
    	     if (!(iss >> id))
    	     	{
    	    	 std::cout << "Error in line loop";
    	    	 break;
    	     	} // error
    	     std::cout << "And id is " << id << "  " <<iss.str() << std::endl;

    	 	updateObservationMatrix(observationMatrix,id,locationOfId(id,objectId));

    	     iss.str().c_str();
    	     std::string msgStr = iss.str();

    	     char *msg = new char[(iss.str()).length() + 1];
    	     strcpy(msg, (iss.str()).c_str());
    	     //char *msg = &(iss.str())[0];
    	    std::cout << "command is "<<msg << std::endl;
			 //std::cout << observationMatrix << std::endl;

    	      std::stringstream ss1(msgStr.substr(6));
//
    	      if(msg[3]=='P' && msg[4]=='A' && msg[5]=='C')
    	      {

    	    	 float finalXYZ[3];
//

    	    	 ss1>> finalXYZ[0]  >> finalXYZ[1] >> finalXYZ[2];
//    	    	 finalXYZ[2] = observationMatrix(locationOfId(id,objectId),3);
    	    	 float currentXYZ[3];
    	    	 currentXYZ[0]=observationMatrix(locationOfId(id,objectId),1);
    	    	 currentXYZ[1]=observationMatrix(locationOfId(id,objectId),2);
    	    	 currentXYZ[2]=observationMatrix(locationOfId(id,objectId),3);

    	    	 float angle =observationMatrix(locationOfId(id,objectId),4) ;
    	    	 float currentOutput[9];
    	    	 float finalOutput[9];
    	    	 createFinalVector(finalXYZ,angle,finalOutput);
    	    	 createFinalVector(currentXYZ,angle,currentOutput);
    	    	 //pick_and_place(currentOutput,finalOutput);
    	    	 pick_and_place(currentOutput,finalOutput,objectId,observationMatrix,id,currentJointPosition);
    	    	// updateObservationMatrix(observationMatrix,locationOfId(id,objectId),finalOutput);
    	    	 //    	    	 std::cout << finalXYZ[0];
//    	    	float *finalXYZ ={} ;
    	      // pick and place

    	      }

    	      if(msg[3]=='P' && msg[4]=='R' && msg[5]=='C')
				  {
					// std::cout << msg[4] << std::endl;
					 int targetId;

					 float relativeXYZ[3];
					 ss1>> targetId >> relativeXYZ[0]  >> relativeXYZ[1] >> relativeXYZ[2];
//					 relativeXYZ[2] = 0;
					 float finalXYZ[3];
					 finalXYZ[0] = observationMatrix(locationOfId(targetId,objectId),1) + relativeXYZ[0];
					 finalXYZ[1] = observationMatrix(locationOfId(targetId,objectId),2) + relativeXYZ[1];
					 finalXYZ[2] = observationMatrix(locationOfId(targetId,objectId),3) + relativeXYZ[2];
					 //std::cout << observationMatrix(locationOfId(targetId,objectId),3) <<std::endl;
//					 std::cout << "hello ";
//					 std::cout << observationMatrix << std::endl;
	//				 finalXYZ[2] = observationMatrix(locationOfId(targetId,objectId),3) + relativeXYZ[2];
					 std::cout << "Final XYZ in Relative case is " <<    finalXYZ[0] <<" " <<   finalXYZ[1] << " " <<finalXYZ[2] << std::endl;;
					 float currentXYZ[3];
					 currentXYZ[0]=observationMatrix(locationOfId(id,objectId),1);
					 currentXYZ[1]=observationMatrix(locationOfId(id,objectId),2);
					 currentXYZ[2]=observationMatrix(locationOfId(id,objectId),3);
					 float angle =observationMatrix(locationOfId(id,objectId),4) ;
					 float currentOutput[9];
					 float finalOutput[9];
					 createFinalVector(finalXYZ,angle,finalOutput);
					 createFinalVector(currentXYZ,angle,currentOutput);
					 //pick_and_place(currentOutput,finalOutput);
					 pick_and_place(currentOutput,finalOutput,objectId,observationMatrix,id,currentJointPosition);
	    	    	 //updateObservationMatrix(observationMatrix,locationOfId(id,objectId),finalOutput);
	  //    	    	 std::cout << finalXYZ[0];
	  //    	    	float *finalXYZ ={} ;
				  // pick and place with respect to targetId

				  }
    	      if(msg[3]=='O' && msg[4]=='A' && msg[5]=='C')
				  {
					 //std::cout << msg[4] << std::endl;
//					 int targetId;

	//				 float relativeXYZ[3];
					 float targetAngle;
					 ss1>>targetAngle;
					 float currentXYZ[3];
					 currentXYZ[0]=observationMatrix(locationOfId(id,objectId),1);
					 currentXYZ[1]=observationMatrix(locationOfId(id,objectId),2);
					 currentXYZ[2]=observationMatrix(locationOfId(id,objectId),3);
					 float angle =observationMatrix(locationOfId(id,objectId),4) ;
					 float currentOutput[9];
					 float finalOutput[9];
					 createFinalVector(currentXYZ,angle,currentOutput);

					 createFinalVector(currentXYZ,targetAngle,finalOutput);
					 // pick_and_place(currentOutput,finalOutput);
					 pick_and_place(currentOutput,finalOutput,objectId,observationMatrix,id,currentJointPosition);
	    	    //	 updateObservationMatrix(observationMatrix,locationOfId(id,objectId),finalOutput);


	  //    	    	 std::cout << finalXYZ[0];
	  //    	    	float *finalXYZ ={} ;
				  // change the angle

				  }

    	 }


    	    	myReadFile.close();

    	    	cout << "Obs matrix " << endl;
    	    	cout << observationMatrix << endl;
    	    	close_spread();
    	    		trapezoidal_close();
    	    		send_home();
     }
