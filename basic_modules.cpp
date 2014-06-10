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




void grabAndRaiseCartesian(float *finalXYZ, float orientationDegree,  float *jointPositionFinal,float *jointPositionj20)
{
float  cartesianAndOrientation[9];
cartesianAndOrientation[0] = finalXYZ[0];
cartesianAndOrientation[1] = finalXYZ[1];
cartesianAndOrientation[2] = finalXYZ[2];
cartesianAndOrientation[3] = 0;
cartesianAndOrientation[4] = 0;
cartesianAndOrientation[5] = -1;
cartesianAndOrientation[6] = -sin(orientationDegree*PI/180);
cartesianAndOrientation[7] = cos(orientationDegree*PI/180);
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
	//std::cout << i << "  " << solutionSpace(i,0) << " " << min << std::endl;
	float dummy = solutionSpace(i,0);
	if(solutionSpace(i,0)<0)
		dummy = -solutionSpace(i,0);
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


int main()
{

float finalPosition[3] = {0.65, 0.16, 0.05};
float angle = 1;
float jp0[7] = {};
float jp[7]= {};
float ojp0[7] = {};
float ojp[7]= {};
float finalPosition1[3] = {0.67, 0.17, 0.05};

grabAndRaiseCartesian(finalPosition,angle,jp,jp0) ;
	changeOrientation(finalPosition1,67,jp0,ojp,ojp0);
int i;
	for(i=0;i<7;i++)
	{
		std::cout <<jp0[i] << " 		" << jp[i] <<"			" << ojp0[i] << " 		" << ojp[i]  << std::endl ;
	}


}

