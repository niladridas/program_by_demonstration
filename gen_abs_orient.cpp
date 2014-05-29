/*
 * absolute_orientation.cpp
 *
 *  Created on: 26-Feb-2014
 *      Author: blue
 */
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/LU>
#include <vector>
#include <cmath>
#include <fstream>

using namespace std;
using namespace Eigen;
int main()
{

	// Using 4 non-colinear points
	//-------------------------------DATA------------------------------------------------------------------------------------------------------------------//
	// The data has to be read from a text file in /data/vector.txt
	// Data format:
	// Y co-ord the X co-ord. For example:
	// If the two points are (0,0,0.5) in robot (Y) co-ordinate and (-0.3,-0.2, 1.42) in camera (X) co-ordinate system them the entry in the text file will bw
	// 0 0 0.5 -0.3 -0.2 1.42
	// A new entry will come in the new line
	// For now we are assumin that there are exactly 4 such entries
	ifstream file;
	file.open( "/home/niladri-64/module_heisenberg/data/input_cordinates_for_trans.txt");
	MatrixXf Y(3,4);
	MatrixXf X(3,4);
	float dummy;

	Y << 0, 0, 0, 0,
		 0, 0, 0, 0,
		 0, 0, 0, 0;
	X << 0, 0, 0, 0,
		 0, 0, 0, 0,
		 0, 0, 0, 0;
	// There are exactly 6 entries in each line

	for(int i =0; i < 4; i++)  // four points
	{
//		file >> dummy;

		for(int j = 0; j < 3; j++) // 3-d points
		{
			file >> Y(j,i);
		}

		for(int k = 0; k < 3; k++) // 3-d points
		{
			file >> X(k,i);
		}
	}
	file.close();

	std::cout << X << "\n" <<std::endl;
	std::cout << Y << std::endl;

	//--------------------------------DATA ENDS-------------------------------------------------------------------------------------------------------------//

// Calculating mean vector of X
	Vector3f X_mean(0,0,0);
	int X_rows = X.rows();
	int X_cols = X.cols();
	for(int i = 0; i< X_rows; i++)
	{
		for(int j = 0; j < X_cols; j++)
		{
          		X_mean(i) = X(i,j) + X_mean(i);
		}
		X_mean(i) = X_mean(i)/X_cols;
	}

// Calculating mean vector of Y
	Vector3f Y_mean(0,0,0);
	int Y_rows = Y.rows();
	int Y_cols = Y.cols();
	for(int i = 0; i< Y_rows; i++)
	{
		for(int j = 0; j < Y_cols; j++)
		{
			Y_mean(i) = Y(i,j) + Y_mean(i);
		}
		Y_mean(i) = Y_mean(i)/Y_cols;
	}

// Calculating the variance of X
	Vector3f d_vec(0,0,0);
	float var_X;
	var_X = 0;
	for(int j = 0; j < X_cols; j++)
	{
		for(int i = 0; i< X_rows; i++)
		{
			d_vec(i) = X(i,j) - X_mean(i);
		}
		var_X = (d_vec.norm()*d_vec.norm())/X_cols + var_X;
	}

// Calculating the variance of Y
		Vector3f c_vec(0,0,0);
		float var_Y;
		var_Y = 0;
		for(int j = 0; j < Y_cols; j++)
		{
			for(int i = 0; i< Y_rows; i++)
			{
				c_vec(i) = Y(i,j) - Y_mean(i);
			}

			var_Y = (c_vec.norm()*c_vec.norm())/Y_cols + var_Y;
		}

// Covariance Matrix
		MatrixXf Covar_XY(3,3);
		MatrixXf tempCovar_XY(3,3);
		typedef Eigen::Matrix<float, 1, 3> RowVector;
		typedef Eigen::Matrix<float, 3, 1> ColVector;
		ColVector temp_Y ;
		RowVector temp_X;

		Covar_XY << 0,0,0,
					0,0,0,
					0,0,0;
		tempCovar_XY = Covar_XY;
		for(int i = 0; i < X_cols ; i++)   // X_cols = Y_cols
		{
			temp_X(0) = X(0,i) - X_mean(0);
			temp_X(1) = X(1,i) - X_mean(1);
			temp_X(2) = X(2,i) - X_mean(2);
			temp_Y(0) = Y(0,i) - Y_mean(0);
			temp_Y(1) = Y(1,i) - Y_mean(1);
			temp_Y(2) = Y(2,i) - Y_mean(2);

			tempCovar_XY = temp_Y*temp_X;
			Covar_XY = (tempCovar_XY/X_cols) + Covar_XY;
		}

// Singular Value Decomposition
		JacobiSVD<MatrixXf> svd(Covar_XY, ComputeThinU | ComputeThinV);
		std::cout << "Its singular values are:" << std::endl << svd.singularValues() << "\n" << std::endl;

		MatrixXf U(3,3);
		MatrixXf V(3,3);
		MatrixXf D(3,3);
		Vector3f D_Sing;

		U = svd.matrixU();
		V = svd.matrixV();

// Forming the D matrix
		D_Sing = svd.singularValues().cast<float>();                     // ---------------VERIFIED----------------------------//
		D  <<   0, 0, 0,
				0, 0, 0,
				0, 0, 0;
		D(0,0) = D_Sing(0);
		D(1,1) = D_Sing(1);
		D(2,2) = D_Sing(2);


// Calculating the rotation Matrix R

// forming the S matrix for R = U*S*trans(V)
		MatrixXf S(3,3);
		if(Covar_XY.determinant()>= 0)
		{
			S << 1, 0, 0,
				 0, 1, 0,
				 0, 0, 1;
		}
		else
		{
			S << 1, 0, 0,
				 0, 1, 0,
				 0, 0, -1;
		}
// R matrix
		MatrixXf R(3,3);
		R = U*S*V.transpose();

// Calculating scalar c

// Calculating trace of matrix D*S
		float trace;
		MatrixXf D_S;
		trace = 0 ;
		D_S = D*S;
//		std::cout << "D matrix = " << D << std::endl;
//		std::cout << "S matrix = " << S << std::endl;
		std::cout << "D*S matrix = " << D_S << "\n" << std::endl;
		for(int i = 0; i < 3; i++)
		{
			trace =  trace + D_S(i,i);
		}

		int c ;
		c = trace/ var_X;

// Calculating t the translation vector
		Vector3f t;
		t = Y_mean - c*R*X_mean;

		std::cout << "R c and t are = \n" << std::endl;
		std::cout << "R = \n" << R << std::endl;
		std::cout << "c = \n" << c << std::endl;
		std::cout << "t = \n" << t << std::endl;

MatrixXf trans_mat(4,4);
int  p ,q ;
for(p = 0;p <4 ; p++)
	for(q = 0;q <4;q++)
	{
		if(p==3)
		{
			if(q==3)
				trans_mat(p,q) = 1;
			else
				trans_mat(p,q) = 0;
		}
		
		else if (q==3)
			trans_mat(p,q) = t(p); 
		else
			trans_mat(p,q) = R(p,q);



	}

		std::cout << "4*4 translation matrix= \n" << trans_mat << std::endl;

std::ofstream file_to_save("/home/niladri-64/module_heisenberg/data/translation_matrix.txt");
if (file_to_save.is_open())
  {
    file_to_save << trans_mat  ;
  }

  file_to_save.close();
  
// Validation of the transformation parameters

// Calculating the error(MSE)
		Vector3f error_vec(0,0,0);
		ColVector Y_transf;
		ColVector X_vec;
		float mse_error;
		mse_error = 0;
		for(int j = 0; j < Y_cols; j++)
		{
			X_vec(0) = X(0,j);
			X_vec(1) = X(1,j);
			X_vec(2) = X(2,j);
			Y_transf = c*R*X_vec  + t;
			for(int i = 0; i< Y_rows; i++)
			{
				error_vec(i) = Y(i,j) - Y_transf(i);
			}
			mse_error = (error_vec.norm()*error_vec.norm())/Y_cols + mse_error;
		}
		std::cout << "\n" << "Enter a point in the camera frame " << std::endl;
		ColVector new_input;
		std::cin >> new_input(0);
		std::cin >> new_input(1);
		std::cin >> new_input(2);
		std::cout << "\n" << "Co-ordinate of the point in the robot frame \n" << c*R*new_input  + t <<  std::endl;
		
		return 0;
}




