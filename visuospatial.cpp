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
float thresholdHeightChange = 0.07;
float thresholdAngleChange = 60;
float thresholdRelativeDistanceChange = 0.25;
float heightNoise = 0.00;
//typedef int demo_num ;

bool isPositionChanged (int timeStamp, MatrixXf obsMatrix , float *distanceChange);
bool isAngleChanged (int timeStamp, MatrixXf obsMatrix , float *angleChange);
bool isVisible(int timeStamp, MatrixXf obsMatrix );
bool isVisibilityChanged(int timeStamp,MatrixXf obsMatrix );
bool isHeightChanged (int timeStamp,MatrixXf obsMatrix, float *heightChanged );
float relativeDistanceMod (int timeStamp, int objectIndex,  int targetObject, std::vector<  MatrixXf > observationVector, int *heightChanged,int *lastTimeStamp);
bool minimumInterDistance(int timeStamp, int objectIndex, std::vector<  MatrixXf > observationVector, int *targetObject, float *relativeDistance, int *heightChanged, int *lastVisibleTimeStamp);
int locationOfId(int targetObjectId,std::vector <int> objectId);
void displayVect4times(std::vector< std::vector< std::vector< std::vector<float> >  > > vect);





bool isPositionChanged (int timeStamp, MatrixXf obsMatrix , float *distanceChange)
{

	int lastTime= timeStamp-1;
	for(;lastTime>=0;lastTime--)
		if(isVisible(lastTime,obsMatrix))
			break;
	if(lastTime==-1)
			return false;
	float prev_x = obsMatrix(lastTime,0) , prev_y= obsMatrix(lastTime,1), prev_z = obsMatrix(lastTime,2);
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
	int lastTime= timeStamp-1;
		for(;lastTime>=0;lastTime--)
			if(isVisible(lastTime,obsMatrix))
				break;
		if(lastTime==-1)
				return false;

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
		return false;
	else
		return true;
}

bool isVisibilityChanged(int timeStamp,MatrixXf obsMatrix )
{
		return (isVisible(timeStamp-1,obsMatrix)!= isVisible(timeStamp,obsMatrix));
}

bool isHeightChanged (int timeStamp,MatrixXf obsMatrix, float *heightChanged )
{
	float prev_height = obsMatrix(timeStamp-1,2);
		float curr_height = obsMatrix(timeStamp,2);
		float	diff_height = curr_height - prev_height;

		*heightChanged = diff_height;
		if(diff_height>0)
			diff_height+=heightNoise;
		else
			diff_height = - diff_height + heightNoise;
		*heightChanged = diff_height;
		std::cout << "Diff_height is " << diff_height << std::endl;
		if(diff_height < thresholdHeightChange )
			return false ;
		else
			return true;

}

float relativeDistanceMod (int timeStamp, int objectIndex,  int targetObject, std::vector<  MatrixXf > observationVector, int *heightChanged,int *lastTimeStamp)
{
	float obj1_x = observationVector[objectIndex](timeStamp,0) , obj1_y=observationVector[objectIndex](timeStamp,1), obj1_z = observationVector[objectIndex](timeStamp,2);
int lastVisibleTimeStamp = timeStamp;

float changedHeight[1];
std::cout <<" is visible for "<< objectIndex << " "<<targetObject<< " is "
		<< !isVisible(lastVisibleTimeStamp,observationVector[targetObject]) << std::endl;
if(targetObject==objectIndex)
		{
		std::cout << "changing time stamp" << std::endl;
		lastVisibleTimeStamp--;
		heightChanged[0] = 1;
		}
else
{
		while(isHeightChanged(timeStamp,observationVector[objectIndex],changedHeight)
				&&!isVisible(lastVisibleTimeStamp,observationVector[targetObject]))
			{
			std::cout << "changing time stamp" << std::endl;
				lastVisibleTimeStamp--;
				heightChanged[0] = 1;
				if(lastVisibleTimeStamp==-1)
					break;
			}

}

std::cout <<" is height changed for "<< objectIndex << " "<<isHeightChanged(timeStamp,observationVector[objectIndex],changedHeight)
		<< " is "  << heightChanged[0] << std::endl;
lastTimeStamp[0] = lastVisibleTimeStamp;
//		if(isHeightChanged(timeStamp,observationVector[objectIndex],heightChanged) )
	//		obj1_z
if(lastVisibleTimeStamp==-1)
		return 9999;

		float obj2_x = observationVector[targetObject](lastVisibleTimeStamp,0) , obj2_y= observationVector[targetObject](lastVisibleTimeStamp,1) ,obj2_z = observationVector[targetObject](lastVisibleTimeStamp,2) ;


		Vector3f prev_loc ;
		Vector3f cur_loc ;
		Vector3f diff_loc;
		prev_loc(0) = obj1_x;prev_loc(1) = obj1_y; prev_loc(2) =0; //obj1_z;
		cur_loc(0) = obj2_x; cur_loc(1) = obj2_y; cur_loc(2) = 0;//obj2_z ;
		diff_loc = cur_loc - prev_loc;

		float relativeD = diff_loc.norm();
		return relativeD;
}

bool minimumInterDistance(int timeStamp, int objectIndex, std::vector<  MatrixXf > observationVector, int *targetObject, float *relativeDistance, int *heightChanged, int *lastVisibleTimeStamp)
{
	int totalObjects = observationVector.size();
	int tempheightChanged[1];
	int templstVisibleTimeStamp[1];
	//float isH
	int i;
	float minrelDistance =9999999999;
	int minIndex = -1;
	float relDistance;
	float changedHeight[1];
	for (i=0;i<totalObjects;i++)
		{
		if (i==objectIndex&&!isHeightChanged(timeStamp,observationVector[objectIndex],changedHeight))
				continue;
		if(isVisible(timeStamp,observationVector[i])&& isVisibilityChanged(timeStamp,observationVector[i]))
			continue;
		relDistance = relativeDistanceMod(timeStamp,objectIndex,i,observationVector,tempheightChanged,templstVisibleTimeStamp);
		if(relDistance < minrelDistance)
			{
				minrelDistance = relDistance;
				minIndex = i;
				heightChanged[0]=tempheightChanged[0];
				lastVisibleTimeStamp[0]=templstVisibleTimeStamp[0];
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

void initialiseMatrixWithZero(MatrixXf& observationMatrix, int rows, int cols)
{
	int i,j;
for (i=0;i<rows;i++)
		for (j =0;j<cols;j++)
			{
			observationMatrix(i,j)=0;

			}
}


int main()
{
	int time_num = 0 , i,j,l;
	int pos_type_num , demos_time ;
		int idAndTypeOfEvents = 4;
		int dataRelatedToEvent = 5;
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

	initialiseMatrixWithZero(observationVector[i],time_stamps,4);
	//std::cout << observationVector[i] <<std::endl;
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

//	std::cout << "dlasjdl n" << inference_matrix[demos-1][time_num-1][no_of_events-1].size() << std::endl;


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
				if(!(isVisible(a2,observationVector[a3])))
				{
					std:: cout <<objectId[a3]  <<" is invisible now " << std::endl;
						continue;
				}
				float positionChanged[1],angleChanged[1],  relativeDistance[1];
				int targetObject[1],heightChanged[1],lastVisibleTimeStamp[1];

				if(!isPositionChanged(a2,observationVector[a3],positionChanged))
					std::cout << objectId[a3] << " position was not changed " << std::endl;
				if (isPositionChanged(a2,observationVector[a3],positionChanged))
							{

								inference_matrix[a1][a2][0][0] = objectId[a3];
								std::cout << "object id on which pos . event ocurred is  " << inference_matrix[a1][a2][0][0] << " a2 time is "<< a2 << std::endl;
								k = 	minimumInterDistance(a2,a3, observationVector,targetObject, relativeDistance,heightChanged,lastVisibleTimeStamp);
								std::cout << "Distance of "<< objectId[a3]<< "changed by " << *relativeDistance <<" w.r.to "<<objectId[*targetObject]  << std::endl;
								std::cout << "object id on which pos . event ocurred is  " << inference_matrix[a1][a2][0][0] << " and k is "<< k<<std::endl;
								if(k)
								{

									inference_matrix[a1][a2][1][0] = 1; // relatively
									inference_matrix[a1][a2][1][1] = objectId[*targetObject];
									inference_matrix[a1][a2][1][2] =  - observationVector[ (*targetObject)](*lastVisibleTimeStamp,0) + observationVector[a3](a2,0);
									inference_matrix[a1][a2][1][3] = -observationVector[(*targetObject)](*lastVisibleTimeStamp,1) + observationVector[a3](a2,1);
									inference_matrix[a1][a2][1][4] = -observationVector[(*targetObject)](*lastVisibleTimeStamp,2) + observationVector[a3](a2,2);

								}
								else
								{
									//std :: cout << "sajdbjaasd af g" << std::endl;
									inference_matrix[a1][a2][1][0] = 2;
									inference_matrix[a1][a2][1][1] = observationVector[a3](a2,0);
									inference_matrix[a1][a2][1][2] =  observationVector[a3](a2,1);
									inference_matrix[a1][a2][1][3] =  observationVector[a3](a2,2);

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
//					if(inference_matrix[demo_count][time_count][1][0]!=0 && inference_matrix[demo_count][time_count][2][0]!=0) // position & angle
//					{
//						inferred << "'s position & angle was changed" ;
//						inferred << ". POsition by " ;
//						inferred_code << "POC ";
//						if(inference_matrix[demo_count][time_count][1][0]==2)
//							{
//							inferred << " absolutively to " << inference_matrix[demo_count][time_count][1][1] << " " << inference_matrix[demo_count][time_count][1][2];
//							inferred_code << "PAC " <<inference_matrix[demo_count][time_count][1][1] << " " << inference_matrix[demo_count][time_count][1][2];
//							}
//						if(inference_matrix[demo_count][time_count][1][0]==1)
//							{
//								inferred << " relatively to id of " <<inference_matrix[demo_count][time_count][1][1] ;
//								inferred << " and x displacement of " << inference_matrix[demo_count][time_count][1][2];
//								inferred << " and y displacement of " << inference_matrix[demo_count][time_count][1][3];
//								inferred_code << "PRC " <<  inference_matrix[demo_count][time_count][1][1] << " " ;
//								inferred_code << inference_matrix[demo_count][time_count][1][2] << " " << inference_matrix[demo_count][time_count][1][3] ;
//							}
//
//						inferred << "and angle was changed to " <<inference_matrix[demo_count][time_count][2][1] ;
//						inferred_code << "OAC" << " " << inference_matrix[demo_count][time_count][2][1];
//
//					}
//
//					else
						if(inference_matrix[demo_count][time_count][1][0]!=0) //only position
					{
						inferred << "'s position was changed";
						if(inference_matrix[demo_count][time_count][1][0]==2)
							{
							inferred << " absolutively to " << inference_matrix[demo_count][time_count][1][1] << " " << inference_matrix[demo_count][time_count][1][2] << " " << inference_matrix[demo_count][time_count][1][3];
							inferred_code << "PAC " <<inference_matrix[demo_count][time_count][1][1] << " " << inference_matrix[demo_count][time_count][1][2] << " " <<inference_matrix[demo_count][time_count][1][3] ;
							}
						if(inference_matrix[demo_count][time_count][1][0]==1)
							{
								inferred << " relatively to id of " <<inference_matrix[demo_count][time_count][1][1] ;
								inferred << " and x displacement of " << inference_matrix[demo_count][time_count][1][2];
								inferred << " and y displacement of " << inference_matrix[demo_count][time_count][1][3];
								inferred << " and z displacement of " << inference_matrix[demo_count][time_count][1][4];
								inferred_code << "PRC " <<  inference_matrix[demo_count][time_count][1][1] << " " ;
								inferred_code << inference_matrix[demo_count][time_count][1][2] << " " << inference_matrix[demo_count][time_count][1][3] <<  " " << inference_matrix[demo_count][time_count][1][4];
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


