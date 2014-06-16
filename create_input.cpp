/*
* Created on: May 22, 2014
* Author: Ankit Pensia
It does nothing except the fact that it takes two files and concatenate their lines in a new text file.
it is required so that transformation matrix can be calculated from the newly created file.

4 lines of observations
first robot frame co-ordinates and then camera frame(created from best_circle.cpp) co-ordinates of the same point.
*/
#include <string>
#include <sstream>
#include <fstream>

#include <iostream>


int main()
{
std::ifstream camera_center("/home/niladri-64/module_heisenberg/data/camera_centers.txt");
std::string line_camera;
std::ifstream robot_center("/home/niladri-64/module_heisenberg/data/cal_cp.txt");
std::string line_robot;

  std::ofstream myfile;
  myfile.open ("/home/niladri-64/module_heisenberg/data/input_cordinates_for_trans.txt");

while (std::getline(camera_center, line_camera) &&  std::getline(robot_center, line_robot) )
{
 std::istringstream coordinates_rob(line_robot);
  std::istringstream coordinates_cam(line_camera);
    double cam_a, cam_b,cam_c;
    double rob_a,rob_b,rob_c;
    if (!(coordinates_rob >> rob_a >> rob_b >> rob_c)) 
      { break; } // error

    if (!(coordinates_cam >> cam_a >> cam_b >> cam_c)) 
      { break; }
          std::cout << rob_a << " " << rob_b << " " << rob_c << " " << cam_a << " " << cam_b << " " << cam_c <<std::endl;
    // process pair (a,b)  

  myfile << rob_a << " " << rob_b << " " << rob_c << " " << cam_a << " " << cam_b << " " << cam_c << std::endl;


}

myfile.close();

	return 0;
}
