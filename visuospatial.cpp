/*
* Created on: June 12, 2014
* Author: Ankit Pensia


*/
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/LU>
#include <vector>
#include <cmath>
#include <fstream>
#include <math.h>

using namespace std;
using namespace Eigen;

float thresholdDistanceChange = 0.1;
float thresholdHeightChange = 0.30;
float thresholdAngleChange = 60;
float thresholdRelativeDistanceChange = 0.25;
//typedef int demo_num ;

bool isPositionChanged (int timeStamp, MatrixXf obsMatrix , float *distanceChange)
{
	float prev_x = obsMatrix(timeStamp-1,0) , prev_y= obsMatrix(timeStamp-1,1), prev_z = obsMatrix(timeStamp-1,2);
	float x = obsMatrix(timeStamp,0),y= obsMatrix(timeStamp,1),z = obsMatrix(timeStamp,2) ;

	Vector3f prev_loc ;
	Vector3f cur_loc ;
	Vector3f diff_loc;
	prev_loc(0) = prev_x;prev_loc(1) = prev_y; prev_loc(2) = prev_z;
	cur_loc(0) = x; cur_loc(1) = y; cur_loc(2) = z ;
	diff_loc = cur_loc - prev_loc;
	*distanceChange = diff_loc.norm();
	if(*distanceChange < thresholdDistanceChange )
		return false ;
	else
		return true;

}

bool isAngleChanged (int timeStamp, MatrixXf obsMatrix , float *angleChange)
{
	float prev_orien = obsMatrix(timeStamp-1,3);
	float curr_orien = obsMatrix(timeStamp,3);

	float	diff_orien = curr_orien - prev_orien;

	*angleChange = diff_orien;
	if(diff_orien< 0)
			diff_orien = - diff_orien;
	if(diff_orien < thresholdAngleChange )
		return false ;
	else
		return true;

}

bool isVisible(int timeStamp, MatrixXf obsMatrix )

{
	if(obsMatrix (timeStamp,0)==0 && obsMatrix (timeStamp,1)==0 && obsMatrix (timeStamp,2)==0 && obsMatrix (timeStamp,3)==0)
		return true;
	else
		return false;
}

bool isVisibilityChanged(int timeStamp,MatrixXf obsMatrix )
{
		return (isVisible(timeStamp-1,obsMatrix)!= isVisible(timeStamp,obsMatrix));
}

bool isHeightChnaged (int timeStamp,MatrixXf obsMatrix, float *heightChanged )
{
	float prev_height = obsMatrix(timeStamp-1,2);
		float curr_height = obsMatrix(timeStamp,2);
		float	diff_height = curr_height - prev_height;

		*heightChanged = diff_height;
		if(diff_height< 0)
			diff_height = - diff_height;
		if(diff_height < thresholdHeightChange )
			return false ;
		else
			return true;

}

float relativeDistanceMod (int timeStamp, int objectIndex,  int targetObject, std::vector<  MatrixXf > observationVector)
{
	float obj1_x = observationVector[objectIndex](timeStamp,0) , obj1_y=observationVector[objectIndex](timeStamp,1), obj1_z = observationVector[objectIndex](timeStamp,2);
		float obj2_x = observationVector[targetObject](timeStamp,0) , obj2_y= observationVector[targetObject](timeStamp,1) ,obj2_z = observationVector[targetObject](timeStamp,2) ;


		Vector3f prev_loc ;
		Vector3f cur_loc ;
		Vector3f diff_loc;
		prev_loc(0) = obj1_x;prev_loc(1) = obj1_y; prev_loc(2) = obj1_z;
		cur_loc(0) = obj2_x; cur_loc(1) = obj2_y; cur_loc(2) = obj2_z ;
		diff_loc = cur_loc - prev_loc;

		float relativeD = diff_loc.norm();
		return relativeD;
}

bool minimumInterDistance(int timeStamp, int objectIndex, std::vector<  MatrixXf > observationVector, int *targetObject, float *relativeDistance)
{
	int totalObjects = observationVector.size();
	int i;
	float minrelDistance =9999999999;
	int minIndex = -1;
	float relDistance;
	for (i=0;i<totalObjects;i++)
		{
		if (i==objectIndex)
				continue;
		relDistance = relativeDistanceMod(timeStamp,objectIndex,i,observationVector);
		if(relDistance < minrelDistance)
			{
				minrelDistance = relDistance;
				minIndex = i;
			}
		}

		*targetObject = minIndex;
		*relativeDistance = minrelDistance;
		if(minrelDistance < thresholdRelativeDistanceChange)
				return true;
		else
			return false;
}

int locationOfId(int targetObjectId,std::vector <int> objectId)
{int alpha;
	for(alpha=0;alpha<objectId.size(); alpha++)
		if( objectId[alpha] == targetObjectId)
				return alpha;

	return -1;

}


void displayVect4times(std::vector< std::vector< std::vector< std::vector<float> >  > > vect)
	{
	int i,j,k,l;
	for(i=0;i<vect.size();i++)
		for(j=0;j<vect[i].size();j++)
			{
				for(k=0;k <vect[i][j].size();k++)
					{
					for(l=0;l<vect[i][j][k].size();l++)
						std::cout << vect[i][j][k][l]  << "  ";
					std::cout<<"                      ";
					}
				std::cout<< std::endl;
			}

	}



int main()
{
	int time_num = 0 , i,j,l;
	int pos_type_num , demos_time ;
		int idAndTypeOfEvents = 4;
		int dataRelatedToEvent = 4;
	 int demos, time_stamps, no_of_events , events_observation_dim ;
		int id,totalObjects ;
		std::vector <int> objectId ;

int k=0;
	std::ifstream infile2("/home/niladri-64/module_heisenberg/data/demo_info.txt");
		std::string line2;
		while (std::getline(infile2, line2))
		{
		    std::istringstream iss2(line2);
		    iss2 >> time_stamps >> totalObjects ;
		    for(int i=0;i<totalObjects;i++)
		    				{
		    				iss2 >> id;
		    				objectId.push_back(id);
		    				}
		}


std::vector<  MatrixXf > observationVector;

observationVector.resize(totalObjects);
for (i=0;i<totalObjects;i++)
	{
	observationVector[i].resize(time_stamps,4);
	//MatrixXf observationVector[i](time_stamps,4);
		std::stringstream fileName;
		fileName << "/home/niladri-64/module_heisenberg/data/id" << objectId.at(i) << ".txt" ;
//		fileName << ".txt";
			std::ifstream infile(fileName.str().c_str());
			std::cout << "FileName is  "<< fileName.str().c_str() << std::endl;
			std::string line;
			while (std::getline(infile, line))
			      {
					std::istringstream iss(line);
			          int tStamp;
			          float x,y,z,orien;
			          if (!(iss >> tStamp >> x >> y >> z >>orien ))
			            { break; } // error

			          observationVector[i](tStamp,0) =  x;
		        	    observationVector[i](tStamp,1) =  y;
		        	    observationVector[i](tStamp,2) =  z;
		        	    observationVector[i](tStamp,3) =  orien;
			      }
		      }

for(i=0;i<observationVector.size();i++)
	std::cout << observationVector[i] << std::endl;

	// Uptil now I have a eigen matrix where row denotes time stamp and column denotes the absolute position of the centre of the object
	// The position attribute depends upon

	std::vector< std::vector< std::vector< std::vector<float> >  > > inference_matrix;
//demo  			// time 	// event (alll information in diff blocks of this vector) 			// data related to event


	// We know how many demonstrations have been done say demo_num
	int demo_num = 1 ;
//	int time_num = 0;
//		int idAndTypeOfEvents = 4;
//		int dataRelatedToEvent = 3;
//	 int demos, time_stamps, no_of_events , events_observation_dim ;
//
	inference_matrix.resize(demo_num);
	for(demos = 0; demos< demo_num ; demos++)
		 inference_matrix[demos].resize(time_stamps);
	for(demos = 0; demos< demo_num ; demos++)
		for(time_num = 0; time_num< time_stamps ; time_num++)
		 	inference_matrix[demos][time_num].resize(idAndTypeOfEvents);
	for(demos = 0; demos< demo_num ; demos++)
		for(time_num = 0; time_num< time_stamps ; time_num++)
			for(no_of_events = 0; no_of_events< idAndTypeOfEvents ; no_of_events++)
			 	inference_matrix[demos][time_num][no_of_events].resize(dataRelatedToEvent);

	std::cout << "dlasjdl n" << inference_matrix[demos-1][time_num-1][no_of_events-1].size() << std::endl;


	std::cout << "start "<< std::endl;

	for(int a1 = 0; a1 < demo_num; a1++)  // demo level
	{
		std::cout << " Demo Time" << time_stamps << std::endl ;
		for(int a2 = 1; a2 <time_stamps ; a2++) // time level
		{
			std::cout << "a2 =" << a2 << std::endl;
			for(int a3 = 0; a3 < totalObjects ; a3++) // checking all objects
			{
				std::cout << "Object id is " << objectId[a3] << std::endl;
				float positionChanged[1],angleChanged[1], heightChanged[1], relativeDistance[1];
				int targetObject[1];

				if (isPositionChanged(a2,observationVector[a3],positionChanged))
							{

					inference_matrix[a1][a2][0][0] = objectId[a3];

								k = 	minimumInterDistance(a2,a3, observationVector,targetObject, relativeDistance);
								if(k)
								{

									inference_matrix[a1][a2][1][0] = 1; // relatively
									inference_matrix[a1][a2][1][1] = objectId[*targetObject];
									inference_matrix[a1][a2][1][2] =  - observationVector[ (*targetObject)](a2,0) + observationVector[a3](a2,0);
									inference_matrix[a1][a2][1][3] = -observationVector[(*targetObject)](a2,1) + observationVector[a3](a2,1);
								}
								else
								{
									//std :: cout << "sajdbjaasd af g" << std::endl;
									inference_matrix[a1][a2][1][0] = 2;
									inference_matrix[a1][a2][1][1] = observationVector[a3](a2,0);
									inference_matrix[a1][a2][1][2] =  observationVector[a3](a2,1);

								}

							}
				if(isAngleChanged(a2,observationVector[a3],angleChanged))
				{
					inference_matrix[a1][a2][0][0] = objectId[a3];
					inference_matrix[a1][a2][2][0] = 1;
					inference_matrix[a1][a2][2][1]= observationVector[a3](a2,3);
				}


			}


			}
			i ++;
		}


	displayVect4times(inference_matrix);

int demo_count;
int time_count;
int event_count;
int event_data_count;
std::stringstream inferred;
std::stringstream inferred_code;

for (demo_count =0; demo_count<demo_num;demo_count++)
		for(time_count=1;time_count<time_stamps;time_count++)
		{

			int targetedObject = inference_matrix[demo_count][time_count][0][0];
			//std::cout << "TOid " << targetedObject << std::endl;
			inferred << targetedObject << " ";//<< std::endl;
			inferred_code << targetedObject << " ";
			//for(event_count=1;event_count<idAndTypeOfEvents ; event_count++)
			//{
				//if(inference_matrix[demo_count][time_count][event_count][0]!=0)
				//{
					if(inference_matrix[demo_count][time_count][1][0]!=0 && inference_matrix[demo_count][time_count][2][0]!=0) // position & angle
					{
						inferred << "'s position & angle was changed" ;
						inferred << ". POsition by " ;
						inferred_code << "POC ";
						if(inference_matrix[demo_count][time_count][1][0]==2)
							{
							inferred << " absolutively to " << inference_matrix[demo_count][time_count][1][1] << " " << inference_matrix[demo_count][time_count][1][2];
							inferred_code << "PAC " <<inference_matrix[demo_count][time_count][1][1] << " " << inference_matrix[demo_count][time_count][1][2];
							}
						if(inference_matrix[demo_count][time_count][1][0]==1)
							{
								inferred << " relatively to id of " <<inference_matrix[demo_count][time_count][1][1] ;
								inferred << " and x displacement of " << inference_matrix[demo_count][time_count][1][2];
								inferred << " and y displacement of " << inference_matrix[demo_count][time_count][1][3];
								inferred_code << "PRC " <<  inference_matrix[demo_count][time_count][1][1] << " " ;
								inferred_code << inference_matrix[demo_count][time_count][1][2] << " " << inference_matrix[demo_count][time_count][1][3] ;
							}

						inferred << "and angle was changed to " <<inference_matrix[demo_count][time_count][2][1] ;
						inferred_code << "OAC" << " " << inference_matrix[demo_count][time_count][2][1];

					}

					else if(inference_matrix[demo_count][time_count][1][0]!=0) //only position
					{
						inferred << "'s position was changed";
						if(inference_matrix[demo_count][time_count][1][0]==2)
							{
							inferred << " absolutively to " << inference_matrix[demo_count][time_count][1][1] << " " << inference_matrix[demo_count][time_count][1][2];
							inferred_code << "PAC " <<inference_matrix[demo_count][time_count][1][1] << " " << inference_matrix[demo_count][time_count][1][2];
							}
						if(inference_matrix[demo_count][time_count][1][0]==1)
							{
								inferred << " relatively to id of " <<inference_matrix[demo_count][time_count][1][1] ;
								inferred << " and x displacement of " << inference_matrix[demo_count][time_count][1][2];
								inferred << " and y displacement of " << inference_matrix[demo_count][time_count][1][3];
								inferred_code << "PRC " <<  inference_matrix[demo_count][time_count][1][1] << " " ;
								inferred_code << inference_matrix[demo_count][time_count][1][2] << " " << inference_matrix[demo_count][time_count][1][3] ;
							}

					}

					else if(inference_matrix[demo_count][time_count][2][0]!=0) //angle
					{
						inferred << "'s angle was changed to " <<inference_matrix[demo_count][time_count][2][1] ;
						inferred_code << "OAC" << " " << inference_matrix[demo_count][time_count][2][1];
					}


					inferred << std::endl;
					inferred_code << std::endl;

					std::ofstream myfile;
					  myfile.open ("/home/niladri-64/module_heisenberg/data/robot_instruct.txt");
					  myfile << inferred_code.str();

				//}
			//}

		}

std::cout <<"inferred is "  << std::endl<< inferred.str() << endl;
std::cout <<"inferred code is "  << std::endl<< inferred_code.str() << endl;
	///std::cout << "demo_num =  " << demo_num <<  std::endl;

	return 0;
}


