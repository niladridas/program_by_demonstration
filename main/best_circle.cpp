#include <iostream>

#include <boost/thread/thread.hpp>

#include <pcl/filters/statistical_outlier_removal.h>

#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <pcl/point_types.h>

#include <Eigen/Dense>
// EuclIdean Clustering



#include <fstream>
#include <string>
#include <fstream>
#include <iostream>
#include<stdio.h>
#include<math.h>
#include <cmath> 
#include<iostream>


 /* Function Definitions */
   /* Function Definitions */
void circlefit3d(double p1[3], double p2[3], double p3[3],
                 double center_data[3], double rad_data[1])
{
	double v2[3];
  int i0;
  double dotp_data_idx_0;
  double v1[3];
  double l1;
  double l2;
  int i;
  double p3_2d[2];


  int center_size[2];
  int rad_size[1];
  double v1n_data[3];
  int v1n_size[2];
  double v2nb_data[3];
  int v2nb_size[2];
  for (i0 = 0; i0 < 3; i0++) {
    dotp_data_idx_0 = p2[i0] - p1[i0];
    v2[i0] = p3[i0] - p1[i0];
    v1n_data[i0] = dotp_data_idx_0;
    v1[i0] = dotp_data_idx_0;
  }

  l1 = sqrt((v1[0] * v1[0] + v1[1] * v1[1]) + v1[2] * v1[2]);
  l2 = sqrt((v2[0] * v2[0] + v2[1] * v2[1]) + v2[2] * v2[2]);
  v1n_size[0] = 1;
  v1n_size[1] = 3;
  for (i = 0; i < 3; i++) {
    v1n_data[i] /= l1;
  }
  for (i0 = 0; i0 < 3; i0++) {
    dotp_data_idx_0 = v2[i0] / l2;
    v2nb_data[i0] = dotp_data_idx_0;
    v1[i0] = dotp_data_idx_0;
  }

  dotp_data_idx_0 = (v1[0] * v1n_data[0] + v1[1] * v1n_data[1]) + v1[2] *
    v1n_data[2];
  v2nb_size[0] = 1;
  v2nb_size[1] = 3;
  for (i = 0; i < 3; i++) {
    v2nb_data[i] -= dotp_data_idx_0 * v1n_data[i];
  }

  /*  normalize v2nb */
  dotp_data_idx_0 = sqrt((v2nb_data[0] * v2nb_data[0] + v2nb_data[1] *
    v2nb_data[1]) + v2nb_data[2] * v2nb_data[2]);
  for (i = 0; i < 3; i++) {
    v2nb_data[i] /= dotp_data_idx_0;
  }
   for (i0 = 0; i0 < 2; i0++) {
    p3_2d[i0] = 0.0;
  }

  /*  has to be calculated */
  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      p3_2d[i0] += v2[i0 + i] * v1n_data[i0 + i];
      p3_2d[1 + i0] += v2[i0 + i] * v2nb_data[i0 + i];
    }
  }

  l2 = 0.5 * (l1 - p3_2d[0]) / p3_2d[1];
  dotp_data_idx_0 = p3_2d[0] / 2.0 + p3_2d[1] * l2;
  l2 = p3_2d[1] / 2.0 - p3_2d[0] * l2;

  /*  centers */
  center_size[0] = 1;
  center_size[1] = 3;
  for (i0 = 0; i0 < 3; i0++) {
    center_data[i0] = 0.0;
  }

  for (i = 0; i < 3; i++) {
    center_data[i] = (p1[i] + dotp_data_idx_0 * v1n_data[i]) + l2 * v2nb_data[i];
  }

  /*  radii */
  dotp_data_idx_0 = center_data[0] - p1[0];
  rad_data[0] = dotp_data_idx_0 * dotp_data_idx_0;
  dotp_data_idx_0 = center_data[1] - p1[1];
  l2 = dotp_data_idx_0 * dotp_data_idx_0;
  dotp_data_idx_0 = center_data[2] - p1[2];
  rad_size[0] = 1;
  rad_data[0] = (rad_data[0] + l2) + dotp_data_idx_0 * dotp_data_idx_0;
  rad_data[0] = sqrt(rad_data[0]);

}


 int main(int argc, char const *argv[])
{
	/* code */



std::ifstream infile("/home/niladri-64/module_heisenberg/data/camera_cordinates.txt");

std::string line;

  int i = 0;
while (std::getline(infile, line))
{
    std::istringstream iss(line);  			

i ++;
}
std::cout << i << std::endl;
int total_row = i;
 
Eigen::MatrixXd m(i,3);

std::vector<int> delimiter_indices ;
std::ifstream infile2("/home/niladri-64/module_heisenberg/data/camera_cordinates.txt");

std::string line2;
int j =0,k = 0 ;

while (std::getline(infile2, line2))
{
	// std::cout << line2
    std::istringstream iss(line2);
    double a, b,c;
    if (!(iss >> a >> b >> c)) 
    	{ break; } // error
    	std::cout << a << " " << b << " " << c <<std::endl;
    // process pair (a,b)
    	
  		m(j,0) =  a;
 			m(j,1) =  b;
 			m(j,2) =  c;
      	
	if(c==0)
		{
		delimiter_indices.push_back(j) ;
		}
	j ++;
}

std::cout << j << std::endl;
std::cout << "size " << delimiter_indices.size()<< std::endl ;



std::vector<std::vector<double> > centers_vec ;	

int q=0,r=0,s=0,p = 0;

for(p = 0; p< delimiter_indices.size() - 1;++p)
{
	std::cout << "p =  "<<  p << std::endl;
	std::cout <<  delimiter_indices.at(p)<< "  "<<  delimiter_indices.at(p+1) << std::endl;

	if( delimiter_indices.at(p+1)-delimiter_indices.at(p) <= 3)
	{
			std::cout << "Not enough points" << std::endl;
		double center[3] = {0,0,0};
		std::vector<double> center_vec(center, center+3);
		centers_vec.push_back(center_vec);
		continue ;
	}
	double center[3] = {0,0,0};
	double radius[1] = {0};
	double error = 1000000;
	double best_center[3] = {0,0,0};
	double best_radius[1] = {0};
	double min_error = 1000000;
	
	for(q =delimiter_indices.at(p)+1 ;q<=delimiter_indices.at(p+1) -3;q++ )
		for(r = q+1;r<=delimiter_indices.at(p+1) -2;r++ )	
			for(s = r+1;s<=delimiter_indices.at(p+1) -1;s++ )
				{
					//std::cout << "q = " << q<< " r = "<< r  << "s = " << s << std::endl;
				double point1[3] = {m(q,0),m(q,1),m(q,2)};
				double point2[3] = {m(r,0),m(r,1),m(r,2)};
				double point3[3] = {m(s,0),m(s,1),m(s,2)};

			circlefit3d(point1,point2,point3,center,radius);
				std::cout << "radius = "<< *radius << " " ;
				error = (0.045> *radius)?(0.045-*radius):(*radius-0.045); ;
				std::cout << "error = "<<  error << " " ;
				if (error<min_error)
					{
						for (i = 0; i < 3; i++)
							 best_center[i] = center[i];
								 
						min_error = error;
							 best_radius[0] = radius[0];
					}

				}	
		std::cout << std::endl ;
		std::vector<double> center_vec(best_center, best_center+3);
		centers_vec.push_back(center_vec);
		std::cout << " radius  "<< *best_radius << std::endl ;
		std::cout << " min_error  "<< min_error << std::endl ;
}
std::cout << "Completed"<< std::endl ;

std::cout << centers_vec.size()<< std::endl ;


  std::ofstream myfile;
  myfile.open ("/home/niladri-64/plc_kinect_workspace/robot/camera_centers.txt");

for(j = 0; j < centers_vec.size() ; j++ )
{

	for( std::vector<double>::const_iterator i = (centers_vec.at(j)).begin(); i != (centers_vec.at(j)).end(); ++i)
	   
	   {
	    std::cout << *i << ' ';

		myfile << *i << " ";
	  }

	  myfile<< std::endl ;
	  std::cout << std::endl ;

}

myfile.close();

return 0;
}
