/*
* Created on: June 10, 2014
* Author: Ankit Pensia

The purpose of this file was to create the functions  grabAndRaiseCartesian and changeOrientation.

More functions can be found in file robot_final_instruct.cpp.
grabAndRaiseCartesian was also deprecated to make way for better functions with more flexibility and user-friendliness.
But it will always be alive in my hearts forever.

void grabAndRaiseCartesian

grabAndRaiseCartesian in this form takes the input , finalXYZ location and its orientation.
It then do inverse kinematics using func_inverse_kine and func_multi_sol, and get lots of solutions in a file.
It then searches for the optimum solution which in present case the one which doesn't make much changes in joint angle 0 and joint angle3 combined.
this can be easily changed in future.
It then updates the two float arrays jointPositionFinal and jointPositionj20, which are basically the same except in jointPositionj20, the 3 rd joint angle is 0
jointPositionj20 refers to the position in which robot approaches first and then open its hand to do further work.
then it goes in jointPositionFinal position to reach the object  and close the grasp and do further work.



void changeOrientation
this was created keeping in mind that it will just change the orientation of object, keeping the position same.
this function will not be developed as the same work can be done using  grabAndRaiseCartesian by just changing the orientation in the  9-sized array.
they basically do the same work.


A socket is created to pass the commands to robot. codewords used for robot arm manipulation can be found in file hand_control_complete .

// the main function just contains the manual code it check if it works or not and It was working absolutely fine.
 * Kudos!!
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




using namespace Eigen;
using namespace std;

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




void grabAndRaiseCartesian(float *finalXYZ, float* orientation,  float *jointPositionFinal,float *jointPositionj20)
{
	std::cout << " finalxyz  " << finalXYZ[0] << "    " << finalXYZ[1] << "   " << finalXYZ[2]<< std::endl;

float  cartesianAndOrientation[9];
cartesianAndOrientation[0] = finalXYZ[0];
cartesianAndOrientation[1] = finalXYZ[1];
cartesianAndOrientation[2] = finalXYZ[2];
cartesianAndOrientation[3] = 0;
cartesianAndOrientation[4] = 0;
cartesianAndOrientation[5] = -1;
cartesianAndOrientation[6] = orientation[0];
cartesianAndOrientation[7] = orientation[1];
cartesianAndOrientation[8] = orientation[2];

std::cout << "  cartesian " << cartesianAndOrientation[0] << "   " << cartesianAndOrientation [1] <<"   "<<  cartesianAndOrientation[2] << std::endl;
std::cout << "  4 to 6  " << cartesianAndOrientation[3] << "   " << cartesianAndOrientation [4] <<"   "<<  cartesianAndOrientation[5] << std::endl;
std::cout << "  orien " << cartesianAndOrientation[6] << "   " << cartesianAndOrientation [7] <<"   "<<  cartesianAndOrientation[8] << std::endl;

float phi_search[2];
int solutionCounter[1];
char fileName[100];
	func_inverse_kine(cartesianAndOrientation,phi_search);
std::cout <<" phi search " << phi_search[0] << phi_search[1] << std::endl;
	func_multi_sol(cartesianAndOrientation,phi_search,solutionCounter,fileName);
std::cout << "Solution counter " << *solutionCounter << fileName << std::endl;

std::ifstream infile(fileName);
MatrixXf solutionSpace (*solutionCounter,7);
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
//std::cout << solutionSpace << std::endl;

float min = 99999999999;
int index = 0;
for(i =0;i < *solutionCounter; i++)
	{
	//std::cout << i << "  " << solutionSpace(i,0) << " " << min << std::endl;
	float dummy1 = solutionSpace(i,0);
	float dummy2 = solutionSpace(i,2);
	if(dummy1<0)
		dummy1=-dummy1;
	if(dummy2<0)
		dummy2=-dummy2;
	float dummy = dummy1 +dummy2;
	if(dummy < min)
		{
		index = i;
		min = dummy;
		}
	}

std::cout << "index = " << index << std::endl;


for(i=0;i<7;i++)
{
	jointPositionFinal[i]= solutionSpace(index,i);
	jointPositionj20[i] = solutionSpace(index,i);
	}

jointPositionj20[1] = 0;
// calculate optimum jp and jp0
//go to jp0
// go to jp
// grab
//then go to jp0



}

void waitForEnter() {
	std::string line;
	std::getline(std::cin, line);
}

void changeOrientation(float *finalXYZ,  float finalOrientation, float *currentJointPosition, float *jointPositionj20, float *jointPositionFinal)
{

	float  cartesianAndOrientation[9];
	cartesianAndOrientation[0] = finalXYZ[0];
	cartesianAndOrientation[1] = finalXYZ[1];
	cartesianAndOrientation[2] = finalXYZ[2];
	cartesianAndOrientation[3] = 0;
	cartesianAndOrientation[4] = 0;
	cartesianAndOrientation[5] = -1;
	cartesianAndOrientation[6] = -sin(finalOrientation*PI/180);
	cartesianAndOrientation[7] = cos(finalOrientation*PI/180);
	cartesianAndOrientation[8] = 0;

	float phi_search[2];
	int solutionCounter[1];
	char fileName[100];
		func_inverse_kine(cartesianAndOrientation,phi_search);
		 func_multi_sol(cartesianAndOrientation,phi_search,solutionCounter,fileName);
	std::cout << "Dasaasas dasd  " << *solutionCounter << fileName << std::endl;

	std::ifstream infile(fileName);
	MatrixXf solutionSpace (*solutionCounter,7);
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
	//std::cout << solutionSpace << std::endl;

	float min = 99999999999;
	int index = 0;
	for(i =0;i < *solutionCounter; i++)
		{
		float dummy = solutionSpace(i,0) - currentJointPosition[0];
		if(solutionSpace(i,0) - currentJointPosition[0] <0)
			dummy = -dummy;
		if(dummy < min)
			{
			index = i;
			min = dummy;
			}
		}

	std::cout << "index = " << index << std::endl;


	for(i=0;i<7;i++)
	{
		jointPositionFinal[i]= solutionSpace(index,i);
		jointPositionj20[i] = solutionSpace(index,i);
		}

	jointPositionj20[1] = 0;
	//
	//go to jointPositionj20
	// go to jointPositionFinal
	// release
	//then go to jointPositionj20


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


void sendData(std::ostringstream *osstring)
{
std::string	msg1 = (*osstring).str();
socket(&msg1[0]);
waitForEnter();
(*osstring).clear();
(*osstring).str("");
}
     using namespace std;
     using namespace Eigen;
     int main()
     {
    	 std::ifstream myReadFile("/home/niladri-64/module_heisenberg/data/end_effector_cartesian.txt");

    	 std::string line2;
    	 std::getline(myReadFile, line2);
    	 std::istringstream iss(line2);
    	     //ifstream myReadFile;
    	    	// myReadFile.open("/home/niladri-64/module_heisenberg/data/end_effector_cartesian.txt");
    	    	 float finalxyz[3];
    	    	 float orientation[3];
    	    	 float output[9];


    	    		iss>> finalxyz[0] >> finalxyz[1] >> finalxyz[2] >> output[3] >> output[4] >> output[5] >> orientation[0] >> orientation[1] >> orientation[2]  ;


    	    	myReadFile.close();

    	    	float jp0[7] = {};
    	    	float jp[7]= {};
    	    	std::cout << "output   "<<  finalxyz[0] << finalxyz[1] << finalxyz[2] << std::endl;
    	    	std::cout << "before func "<<std::endl;
    	    	grabAndRaiseCartesian(finalxyz,orientation,jp,jp0) ;
    	    	std::cout << "after func " << std::endl;

//std::string msg1 ;
//{
		std::ostringstream ss;
		ss<<  "j " << jp0[0] << " " << jp0[1] << " " << jp0[2] << " " << jp0[3] << " " << jp0[4] << " " << jp0[5] << " " << jp0[6];
		sendData(&ss);
		ss << "o" ;
		sendData(&ss);
		ss<<  "j " << jp0[0] << " " << jp0[1] << " " << jp0[2] << " " << jp0[3] << " " << jp0[4] << " " << jp0[5] << " " << jp0[6];
		sendData(&ss);
		ss  << "j " << jp[0] << " " << jp[1] << " " << jp[2] << " " << jp[3] << " " << jp[4] << " " << jp[5] << " " << jp[6];
		sendData(&ss);
		ss  << "c ";
		sendData(&ss);
		ss<<  "j " << jp0[0] << " " << jp0[1] << " " << jp0[2] << " " << jp0[3] << " " << jp0[4] << " " << jp0[5] << " " << jp0[6];
		sendData(&ss);
		ss  << "j " << jp[0] << " " << jp[1] << " " << jp[2] << " " << jp[3] << " " << jp[4] << " " << jp[5] << " " << jp[6];
		sendData(&ss);
		ss  << "e ";
		sendData(&ss);
		ss<<  "j " << jp0[0] << " " << jp0[1] << " " << jp0[2] << " " << jp0[3] << " " << jp0[4] << " " << jp0[5] << " " << jp0[6];
		sendData(&ss);
		ss  << "a ";
		sendData(&ss);
		ss  << "b";
		sendData(&ss);
		ss  << "h";
		sendData(&ss);
		ss  << "q";
		sendData(&ss);
     }
