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
float thresholdNearDistance = 0.14;
float thresholdNearDistance_obstacle = 0.21;
float zOffset = 0.13;
float zOffsetIntermediate = 0.2;

void sendData(std::ostringstream *osstring);
void close_grasp();
void open_grasp();
void send_home();
void sendData(std::ostringstream *osstring);
void  socket(char *msg);
void trapezoidal_close();
void close_spread();
void trapezoidal_init();
void waitForEnter() {
	std::string line;
	std::getline(std::cin, line);
}



void calculateJpForPickAndPlace(float *initialDetails, float *jointPositionFinal,float *intermediateJointPosition,float *jointPositionj20)
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


	if(dummy0<0)
		dummy0=-dummy0;
	if(dummy2<0)
		dummy2=-dummy2;
	if(dummy4<0)
		dummy4=-dummy4;

	float dummy = dummy0 +dummy2 ; //+ 0.4*dummy6;
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
float error1,error2,error3,error4,error5,error6,error0;
float min_error=99999,curr_error;
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

	 curr_error= error0 + 0.2*error4 + error2 + error6 ;//error3+error4+error5+error6;
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
	ss << "m 50" ;
	sendData(&ss);
}

bool isNear(float *firstXYZ, float *secondXYZ) //This function returns if the 2 given cartesian points are near or not.
// It doesn't take z in account (now but can be changed). thresholdNearDistance can be changed to change the parameter.
{
	Vector2f relativeXYZ;
	relativeXYZ(0)=firstXYZ[0]-secondXYZ[0];
	relativeXYZ(1)=firstXYZ[1]-secondXYZ[1];
	//relativeXYZ(2)=firstXYZ[2]-secondXYZ[2];
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

void goToJointPosition(float *jointPosition)
{

	std::ostringstream ss;
	ss<<  "j " << jointPosition[0] << " " << jointPosition[1] << " " << jointPosition[2] << " " << jointPosition[3] << " " << jointPosition[4] << " " << jointPosition[5] << " " << jointPosition[6];
	sendData(&ss);
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
	dummyXYZ[2]= -0.35;
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

void updateObservationMatrix(MatrixXf& observationMatrix, int objectid, float *finalOutput)
{

	observationMatrix(objectid,1)= finalOutput[0];
	observationMatrix(objectid,2)= finalOutput[1];
	observationMatrix(objectid,3)= finalOutput[2] - zOffset;
	observationMatrix(objectid,4)= atan2(finalOutput[7],finalOutput[6]) * (180/PI) - 90;

}
void createFinalVector(float *finalXYZ, float angle, float *output)
{
	if(angle <0)
		angle=-angle;
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

void pick_and_place (float *currentDetails, float *finalDetails,std::vector <int> objectId,MatrixXf& observationMatrix,int currentMarkerId)
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
		pick_and_place(obstacleDetails,dummyDetails,objectId,observationMatrix,objectId[obstaclePID]);
		// then put obstacle in the dummy XYZ

		}

	std::cout << " calculating JP for grasping" << std::endl;

	calculateJpForPickAndPlace(currentDetails,currentJp,currentIntermediateJointPosition,currentJpJ20);	goToJointPosition(currentJpJ20);
//	std::cout << "initialise hand" << std::endl;
//
	initialise_hand();
	trapezoidal_init();
	goToJointPosition(currentIntermediateJointPosition);
	goToJointPosition(currentJp);
	close_grasp();
	goToJointPosition(currentIntermediateJointPosition);
	goToJointPosition(currentJpJ20);

	calculateJpForPickAndPlace(finalDetails,finalJp,finalIntermediateJointPosition,finalJpJ20);
	goToJointPosition(finalJpJ20);
	//initialise_hand();
	goToJointPosition(finalIntermediateJointPosition);
	goToJointPosition(finalJp);

	trapezoidal_init();
	//close_grasp();
	goToJointPosition(finalIntermediateJointPosition);
	goToJointPosition(finalJpJ20);
	open_grasp();
	updateObservationMatrix(observationMatrix,locationOfId(currentMarkerId,objectId),finalDetails);
	std::cout << "updated matrix" << std::endl << observationMatrix << std::endl;
	//	close_spread();
//	trapezoidal_close();
//	send_home();
}



     using namespace std;
     using namespace Eigen;
     int main()
     {
    		std::vector <int> objectId ;

    	 MatrixXf observationMatrix (8,5);
 //   	 observationVector.resize(8);
    		std::ifstream intialFile("/home/niladri-64/module_heisenberg/data/initial_condition.txt");
   			std::string file_line;
   			int index = 0;
    			while (std::getline(intialFile, file_line))
    			{
    				int id;
    				float xyzangle[4];
    			    std::istringstream iss2(file_line);
    			    iss2 >> id >> xyzangle[0] >> xyzangle[1] >> xyzangle[2] >> xyzangle[3] ;
       				objectId.push_back(id);
       				observationMatrix(index,0)=id;
       				observationMatrix(index,1)=xyzangle[0];
       				observationMatrix(index,2)=xyzangle[1];
       				observationMatrix(index,3)=xyzangle[2];
       				observationMatrix(index,4)=xyzangle[3]; // angle
       				index++;
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

    	    	 ss1>> finalXYZ[0]  >> finalXYZ[1];
    	    	 finalXYZ[2] = observationMatrix(locationOfId(id,objectId),3);
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
    	    	 pick_and_place(currentOutput,finalOutput,objectId,observationMatrix,id);
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
					 ss1>> targetId >> relativeXYZ[0]  >> relativeXYZ[1];
					 relativeXYZ[2] = 0;
					 float finalXYZ[3];
					 finalXYZ[0] = observationMatrix(locationOfId(targetId,objectId),1) + relativeXYZ[0];
					 finalXYZ[1] = observationMatrix(locationOfId(targetId,objectId),2) + relativeXYZ[1];
					 std::cout << observationMatrix(locationOfId(targetId,objectId),3) <<std::endl;
//					 std::cout << "hello ";
//					 std::cout << observationMatrix << std::endl;
					 finalXYZ[2] = observationMatrix(locationOfId(targetId,objectId),3) + relativeXYZ[2];
					 std::cout << finalXYZ[2];
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
					 pick_and_place(currentOutput,finalOutput,objectId,observationMatrix,id);
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
					 createFinalVector(currentXYZ,targetAngle,finalOutput);
					 createFinalVector(currentXYZ,angle,currentOutput);
					// pick_and_place(currentOutput,finalOutput);
					 pick_and_place(currentOutput,finalOutput,objectId,observationMatrix,id);
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
