
#include <string>
#include <sstream>
#include <fstream>

#include <iostream>


int main()
{
std::ifstream camera_center("/home/niladri-64/plc_kinect_workspace/robot/camera_centers.txt");
std::string line_camera;
std::ifstream robot_center("/home/niladri-64/plc_kinect_workspace/robot/cal_cp.txt");
std::string line_robot;

  std::ofstream myfile;
  myfile.open ("/home/niladri-64/plc_kinect_workspace/robot/input_cordinates_for_trans.txt");

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
