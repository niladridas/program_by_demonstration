#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <boost/thread/thread.hpp>
#include <pcl/filters/passthrough.h>
#include <string>
#include <sstream>

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,void* viewer_void);

int count = 0;
int flag = 0;

 class SimpleOpenNIViewer
 {
   public:

     pcl::PointCloud<pcl::PointXYZRGBA> s_cloud;
     pcl::visualization::CloudViewer viewer;

     SimpleOpenNIViewer(): viewer ("Simple Cloud Viewer") {}

     void save_cloud()
	{  count++;
	   std::stringstream oss;
	   oss<<"saved_file"<<count<<".pcd";
	   pcl::io::savePCDFileASCII (oss.str(), s_cloud);
	}

     void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
     {
       if (!viewer.wasStopped())
         {
//           pcl::PointCloud<pcl::PointXYZRGBA>::Ptr result (new pcl::PointCloud<pcl::PointXYZRGBA>);
//
//           pcl::PassThrough<pcl::PointXYZRGBA> pass;
//   	      pass.setFilterFieldName ("z");
//    	   pass.setFilterLimits (0.0, 3.0);
//    	   pass.setInputCloud (cloud);
//    	   pass.filter (*result);
//
//           s_cloud = *result;
//           viewer.showCloud (result);

    	   s_cloud = *cloud;
    	   viewer.showCloud(cloud);
         }
     }

     void run ()
     {
       pcl::Grabber* interface = new pcl::OpenNIGrabber();

       boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
         boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

       interface->registerCallback (f);

       viewer.registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);

       interface->start ();

       while (!viewer.wasStopped())
       {
         boost::this_thread::sleep (boost::posix_time::seconds (10));
         if(flag > 0)
         {
         save_cloud();
//         count++;
         }
         if (count >= 5)
        		 {
        	 return;
        		 }
       }

       interface->stop ();
     }


 }v; //Creating object of class


//Defining callback
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,void* viewer_void)
{

  if (event.getKeySym () == "s" && event.keyDown ())
  {
    std::cout << "s was pressed : saving point cloud" << std::endl;
//    v.save_cloud();
    flag = 1;
  }
}

 int main ()
 {

   v.run ();
   return 0;
 }
