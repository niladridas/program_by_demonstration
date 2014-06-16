/*
 * create_inference_input.cpp
 *
 *  Created on: Jun 7, 2014
 *      Author: Niladri Das & Ankit Pensia
 It  makes the inference of the type of the operation and the intention of the user behind it from the set of observations.
It takes the input in the form of a txt file where each line denotes the time stamp and it contains x and y position of the object.
if a row of 0 0 is found. It means a new demonstration has begun. The location of that row is stored in a Vector and is used to keep the track of
no of time stamps in each demonstration.
That information of complete txt file is stored in a as it is, in eigen matrix.
and then a inference matrix is made(actually a vector of vector of vector of vector).

	std::vector< std::vector< std::vector< std::vector<float> >  > > inference_matrix;


	// The lowest level vector denotes the x y co-ordinates
	// The next outer level denotes the whether absolute or relative observation.
	// The next outer level denotes the time stamp in the given demonstration (instance stamp)
	// The next outer level denotes the demonstration stamp (which demonstration ?)
	 *
	 *
	 * It just stores the observation in different format than eigen matrix created earlier.
	 * It saves the same information twice in the format of absolute and relative recordings so that
	 * further methods can be used to identify which of them was intended.
It then creates the score of intention of the user in the scoring matrix. (using standard deviation) (i.e to identify absolute position or relative posiition
was required. The one with less standard deviation is the intention of user)

The further development of this file for multiple objects was left and another file was used using the ideas of this file.
 */

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/LU>
#include <vector>
#include <cmath>
#include <fstream>
#include <math.h>

//typedef int demo_num ;


using namespace std;
using namespace Eigen;
int main()
{
	int time_num = 0;
		int pos_type_num = 2;
		int pos_dim = 2;
	 int demos, time_stamps, no_of_events , events_observation_dim ;


	std::ifstream infile2("/home/niladri-64/module_heisenberg/data/demonstration.txt");

	std::string line2;

	  int k = 0;
	while (std::getline(infile2, line2))
	{
	    std::istringstream iss2(line2);

	k++;
	}


	MatrixXf temp_trans(k,pos_dim);
	std::vector<int> demos_time ;
	std::ifstream infile("/home/niladri-64/module_heisenberg/data/demonstration.txt");
	std::string line;
	int i = 0;
	  int j = 0;
	while (std::getline(infile, line))
	      { j ++;
	          std::istringstream iss(line);
	          float a, b;
	          if (!(iss >> a >> b ))
	            { break; } // error
	            std::cout << a << " " << b << std::endl;
	          // process pair (a,b)

	        	    temp_trans(i,0) =  a;
	              temp_trans(i,1) =  b;
	              if(a ==0 & b==0)
	              	        	{
	            	  	  	  	  demos_time.push_back(j-1);
	              	  	  	  	  j = 0;
	      	  	  	  	  	  	 }


	                     i ++;
	      }

	std::cout << "i (no. of lines) =  " << i <<  std::endl;
	// Uptil now I have a eigen matrix where row denotes time stamp and column denotes the absolute position of the centre of the object
	// The position attribute depends upon


	std::vector< std::vector< std::vector< std::vector<float> >  > > inference_matrix;


	// The lowest level vector denotes the x y co-ordinates
	// The next outer level denotes the whether absolute or relative
	// The next outer level denotes the time stamp (instance stamp)
	// The next outer level denotes the demonstration stamp (which demonstration ?)
//	std::ifstream infile("/home/niladri-64/module_heisenberg/data/demonstration.txt");
//
//	std::string line;
//	   int i = 0;

//	while (std::getline(infile, line))
//	{
//		j++;
//	    std::istringstream iss(line);
//	    double x , y;
//	        if (!(iss >> x >> y))
//	        	 break;  // error
//	        if(x ==0 & y==0)
//	        	demos_time.push_back(j-1), j =0;
//
//	i ++;
//
//	}

	// We know how many demonstrations have been done say demo_num
	int demo_num = demos_time.size() ;

	inference_matrix.resize(demo_num);
	for(demos = 0; demos< demo_num ; demos++)
		 inference_matrix[demos].resize(demos_time[demos]);
	for(demos = 0; demos< demo_num ; demos++)
		for(time_num = 0; time_num< demos_time[demos] ; time_num++)
		 	inference_matrix[demos][time_num].resize(pos_type_num);
	for(demos = 0; demos< demo_num ; demos++)
		for(time_num = 0; time_num< demos_time[demos] ; time_num++)
			for(no_of_events = 0; no_of_events< pos_type_num ; no_of_events++)
			 	inference_matrix[demos][time_num][no_of_events].resize(pos_dim);

	std::cout << "dlasjdl n" << inference_matrix[demos-1][time_num-1][no_of_events-1].size() << std::endl;

// inference_matrix[0].resize(2);
// 	inference_matrix[0][0].resize(2);
// 	inference_matrix[0][1].resize(2);
// //	inference_matrix.resize(1);
//
//	for(demos = 0; demos< demo_num ; demos++)
//		for(time_num = 0; time_num< demos_time[demos] ; time_num++)
//			for(no_of_events = 0; no_of_events< pos_type_num ; no_of_events++)
//				for(events_observation_dim = 0; events_observation_dim< pos_dim ; pos_dim++)
//
//
//	for (int i = 0; i < 10; i++)
//	{
//		inference_matrix.push_back(std::vector< std::vector<std::vector<float> > >()); // Add an empty row
//	}
	int zero_line = demos_time[0];
	std::cout << "start "<< std::endl;
	for(demos = 0; demos< demo_num ; demos++)
std :: cout  << demos_time[demos] <<std::endl;
	i = 0;
	for(int a1 = 0; a1 < demo_num; a1++)  // demo level
	{
		std::cout << " Demo Time" << demos_time[a1] << std::endl ;


		for(int a2 = 0; a2 < demos_time[a1]; a2++) // time level
		{
			std::cout << "a2 =" << a2 << std::endl;

			if (i== zero_line)
							{a2 = a2-1;
								if(a1 + 1 != demo_num)
									zero_line+= demos_time[a1+1];
								i++;
								continue ;
							}

			for(int a3 = 0; a3 < pos_type_num ; a3++) // position type level
			{
				std::cout << "a3 =" << a3 << std::endl;


				if(a3==0)
				{
//					inference_matrix[a1][a2][a3].push_back(std::vector<float>());
//				inference_matrix[a1][a2][a3].push_back(temp_trans(a2,0));
//					inference_matrix[0][0][0].push_back(0.0);
								//	inference_matrix[a1][a2][a3].push_back(0.0);


					inference_matrix[a1][a2][a3][0]=(temp_trans(i,0));
					inference_matrix[a1][a2][a3][1]=(temp_trans(i,1));
					std::cout << "Absolute  " << inference_matrix[a1][a2][a3][0] << inference_matrix[a1][a2][a3][1]   <<"Size " << inference_matrix[a1][a2][a3].size()<< std::endl;
					std::cout << "temp  " << temp_trans(i,0)<< temp_trans(i,1) << std::endl;
					std::cout << "Fine upto this " << a1 << a2 << a3<< std::endl;

				}


				else if(a3==1 && a2!= 0)
				{
					inference_matrix[a1][a2][a3][0]=(temp_trans(i,0) - temp_trans(i-1,0) );
					inference_matrix[a1][a2][a3][1]=(temp_trans(i,1) - temp_trans(i-1,1) );
					std::cout << "Fine upto this " << a1 << a2 << a3<< std::endl;

				}
				else if(a3 == 1 && a2==0)
				{
					std::cout << "Fine upto this " << a1 << a2 <<  a3<< std::endl;

					inference_matrix[a1][a2][a3][0]=0 ;//.push_back(0);
					inference_matrix[a1][a2][a3][1]= 0; //.push_back(0);
				}


			}
			i ++;
		}
	}

	std::cout << "demo_num =  " << demo_num <<  std::endl;
//
//	std::cout << "Matrix" << demo_num <<  std::endl;

	// Creating the scoring matrix

	// There is only one demonstration
	float sum_abs_x = 0, mean_abs_x= 0 ,sum_rel_x = 0, mean_rel_x= 0 , var_abs_x , var_rel_x, sum_abs_x_sq = 0, sum_rel_x_sq = 0 ;
	float sum_abs_y = 0, mean_abs_y= 0 ,sum_rel_y = 0, mean_rel_y= 0 , var_abs_y, var_rel_y,  sum_abs_y_sq = 0, sum_rel_y_sq = 0 ;
	float score_abs , score_rel;
	std::ofstream f1("/home/niladri-64/module_heisenberg/data/position_intention.txt");

	std::vector< std::vector< std::vector<float>  > > scoring_matrix;
	scoring_matrix.resize(demos_time[0]);
//		for(demos = 0; demos< demo_num ; demos++)
//			 inference_matrix[demos].resize(demos_time[demos]);
		//for(demos = 0; demos< demo_num ; demos++)
			for(time_num = 0; time_num< demos_time[0] ; time_num++)
				scoring_matrix[time_num].resize(pos_type_num);
			for(time_num = 0; time_num< demos_time[0] ; time_num++)
				for(no_of_events = 0; no_of_events< pos_type_num ; no_of_events++)
					scoring_matrix[time_num][no_of_events].resize(5);

			for(time_num = 1; time_num< demos_time[0] ; time_num++)
							{
								//for(no_of_events = 0; no_of_events< pos_type_num ; no_of_events++)

								{
//									std::cout << inference_matrix[0][time_num][no_of_events][0] <<  std::endl;
										for(int l =0; l < demo_num; l++)
										{
			//								std::cout << "Fine "  << inference_matrix[l][time_num][0][0] << std::endl;
											sum_abs_x = sum_abs_x + inference_matrix[l][time_num][0][0];
											sum_abs_y = sum_abs_y + inference_matrix[l][time_num][0][1];

		//									std::cout << "OK =  " <<  std::endl;

											sum_abs_x_sq +=  (inference_matrix[l][time_num][0][0])*(inference_matrix[l][time_num][0][0]);
											sum_abs_y_sq +=  (inference_matrix[l][time_num][0][1])*(inference_matrix[l][time_num][0][1]);
										}
								mean_abs_x = sum_abs_x / demo_num;
								mean_abs_y = sum_abs_y / demo_num;


										for(int l =0; l < demo_num; l++)
										{
											sum_rel_x = sum_rel_x + inference_matrix[l][time_num][1][0];
											sum_rel_y = sum_rel_y + inference_matrix[l][time_num][1][1];
											sum_rel_x_sq +=  (inference_matrix[l][time_num][1][0])*(inference_matrix[l][time_num][1][0]);
											sum_rel_y_sq +=  (inference_matrix[l][time_num][1][1])*(inference_matrix[l][time_num][1][1]);


										}
								mean_rel_x = sum_rel_x / demo_num;
								mean_rel_y = sum_rel_y / demo_num;

								var_abs_x = ((sum_abs_x_sq )/demo_num) - (mean_abs_x )*mean_abs_x ;
								var_abs_y = ((sum_abs_y_sq )/demo_num) - (mean_abs_y )*mean_abs_y ;

								var_rel_x = ((sum_rel_x_sq )/demo_num) - (mean_rel_x )*mean_rel_x ;
								var_rel_y = ((sum_rel_y_sq )/demo_num) - (mean_rel_y )*mean_rel_y ;
								score_abs = (sqrt (var_abs_x )/mean_abs_x) + (sqrt(var_abs_y)/mean_abs_y);
								score_rel = (sqrt (var_rel_x )/mean_rel_x) + (sqrt(var_rel_y)/mean_rel_y);
								score_abs = fabs(score_abs);
								score_rel=fabs(score_rel);
								std::cout <<" Score abs "<< (score_abs) << std::endl ;
								std::cout <<"Score rel " << (score_rel) << std::endl ;
								if((score_abs) <= (score_rel))
								{
									std::cout << "The user intended to have a absolute position of" << mean_abs_x << " " << mean_abs_y << std::endl;
									f1 << "abs " << mean_abs_x << " " << mean_abs_y << std::endl;
								}
								else
								{
									std::cout << "The user intended to have a relative position of" << mean_rel_x << " "<< mean_rel_y << std::endl;
									f1 << "rel " << mean_rel_x << " " << mean_rel_y << std::endl;

								}
								scoring_matrix[time_num][0].push_back(mean_abs_x);
								scoring_matrix[time_num][0].push_back(mean_abs_y);
								scoring_matrix[time_num][0].push_back(var_abs_x);
								scoring_matrix[time_num][0].push_back(var_abs_y);

								scoring_matrix[time_num][0].push_back(score_abs);

								scoring_matrix[time_num][0].push_back(mean_rel_x);
								scoring_matrix[time_num][0].push_back(mean_rel_y);
								scoring_matrix[time_num][0].push_back(var_rel_x);
								scoring_matrix[time_num][0].push_back(var_rel_y);

								scoring_matrix[time_num][0].push_back(score_rel);


								}


				}



			f1.close();
	return 0;
}


