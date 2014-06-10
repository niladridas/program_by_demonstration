
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

	int k = 0;
	int pos_dim = 2; // Number of types of position variables abs or relative


	std::ifstream infile2("/home/niladri-64/module_heisenberg/data/demonstration.txt");
	std::string unused;
	while (std::getline(infile2, unused)) // Counting the total number of lines
	{
		k++;
	}
	infile2.close();

	std::ifstream infile3("/home/niladri-64/module_heisenberg/data/demonstration.txt");
	for(int i = 0; i < k -1; ++i)
	       std::getline(infile3, unused);
	std::getline(infile3,unused); // Saving the last


	std::vector<std::vector<std::vector<std::vector<float> > > > Data_matrix;



	MatrixXf temp_trans(k,pos_dim);
	std::vector<int> demos_time ;
	std::ifstream infile("/home/niladri-64/module_heisenberg/data/demonstration.txt");
	std::string line;
	int i = 0;
	int j = 0;
	while (std::getline(infile, line))
	{

	}
	return 0;
}
