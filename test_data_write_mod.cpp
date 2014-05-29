// OpenNI Grabber
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <boost/thread/thread.hpp>
#include <pcl/filters/passthrough.h>
#include <string>
#include <sstream>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <vector>

// Client socket include files
#include <unistd.h>
#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <netdb.h>
#include <string>
#include<fstream>

// Inputs to this program are:
// 3 non co-linear points in frame A (ie camera frame) say x
// corresponding 3 non co-linear points in frame B (ie robot frame) say y
void pointpickingEventOccurred (const pcl::visualization::PointPickingEvent &event, void* viewer_void);
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,void* viewer_void);
//Global Variables
pcl::PointXYZRGBA first,second;
int point_count =1;
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
int nil_counter = 0;
ofstream file;
ofstream file_camera;
//std::ifstream infile2("/home/niladri-64/plc_kinect_wokspace/cal_joint_pos.txt");
//std::string line_jp;

class SimpleOpenNIViewer
 {
   public:
     pcl::PointCloud<pcl::PointXYZRGBA> s_cloud;
     pcl::visualization::CloudViewer viewer;

     SimpleOpenNIViewer(): viewer ("Simple Cloud Viewer") {}
     int status;
     int socketfd ;
     struct addrinfo host_info;
     struct addrinfo *host_info_list;

void save_cloud()
     {
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
//      char msg[100] ;
      char msg[100] ;
      std::cout << "enter the command \n" << std::endl;
//      std::cin.getline( msg, 100 );

      // instead of this input from the user

//   	  std::getline(infile2,line_jp);

      std::cin.getline( msg, 100 );

      ssize_t bytes_sent;
      int len = (int) strlen(msg);
      bytes_sent = send(socketfd, (char *)msg, len, 0);
  	  std::cout << "correct" << std::endl;

      if(msg[0] == 'p')
         	  {
//    	  	  for(int j = 2; j < len; j++)
//    	  	  {
//    	  		  msg_tmp[j-2] = msg[j];
//    	  	  }
    	  	  std::cout << "in P" << std::endl;
         	  file << ((msg)+2*sizeof(char)) << std::endl ; //<< std::endl;
    	  	  std::cout << ((msg)+2*sizeof(char)) << std::endl;

         	  }
      ssize_t bytes_recieved;
      char incoming_data_buffer[1000];
      bytes_recieved = recv(socketfd, incoming_data_buffer,1000, 0);
      std::cout << incoming_data_buffer << "\n" << std::endl;
      std::cout << "Choose the point from the image" << "\n" << std::endl;
     	}

void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
     {
      if (!viewer.wasStopped())
     {
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr result (new pcl::PointCloud<pcl::PointXYZRGBA>);
      pcl::PassThrough<pcl::PointXYZRGBA> pass;
      pass.setFilterFieldName ("z");
      pass.setFilterLimits (0.0, 3.0);
      pass.setInputCloud (cloud);
      pass.filter (*result);
      s_cloud = *result;
      viewer.showCloud (result);
     }
     }

void run ()
     {
      pcl::Grabber* interface = new pcl::OpenNIGrabber();
      boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
      boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);
      interface->registerCallback (f);
      viewer.registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
      viewer.registerPointPickingCallback (pointpickingEventOccurred, (void*)&viewer);
      interface->start ();
      while (!viewer.wasStopped())
      {
        boost::this_thread::sleep (boost::posix_time::seconds (1));
      }
      interface->stop ();
      }
}v;

 //Defining callback
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,void* viewer_void)
     {
     if (event.getKeySym () == "s" && event.keyDown ())
     {
     std::cout << "s was pressed : starting" << std::endl;
     file_camera << "0 0 0"<<std::endl;
     v.save_cloud();
      }
      }

void pointpickingEventOccurred (const pcl::visualization::PointPickingEvent &event, void* viewer_void)
     {
 	 int indx;
 	 float x,y,z;
     indx = event.getPointIndex();
 	 if(indx == -1)
     return;
 //get co-ordinates of point clicked
     event.getPoint(x,y,z);
     first.x = x;
 	 first.y = y;
 	 first.z = z;
 	 std::cout<<"Co-ordinate points are : "<<first.x<<" , "<<first.y<<" , "<<first.z<<std::endl;
 	 point_count++;
 	 file_camera  << first.x <<" " << first.y <<" "<< first.z << std::endl;
 	 std::cout<<"\n Now again press s on the image to enter commands for moving or go on setecting points"<<std::endl;

     }


using namespace std;
using namespace Eigen;
int main()
{
	std::cout << "Start by pressing s when the picture comes \n" << std::endl;
//	ofstream file; // out file stream
	file.open("/home/niladri-64/module_heisenberg/data/robot_cordinates1.txt");
	file_camera.open("/home/niladri-64/module_heisenberg/data/camera_cordinates.txt");

//	SimpleOpenNIViewer v;
	v.run ();
	file.close();
  file_camera << "0 0 0";
	file_camera.close();
	return 0;
}
