/*
 * pcd_rgb_d.cpp
 *
 *  Created on: May 23, 2014
 *      Author: niladri-64
 */

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include<pcl/common/geometry.h>

#include <pcl/filters/statistical_outlier_removal.h>

#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>

// EuclIdean Clustering
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>


// Opencv

#include "opencv2/highgui/highgui.hpp"
#include<opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include<cmath>
#include<stdlib.h>



//template< class IntType >
//IntType mod( IntType a, IntType b )
//{
//    IntType const r = a%b;
//    return (r < 0? r + b : r);
//}

int main (int argc, char **argv)
	{
	   pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA> );
	   pcl::PCDReader reader;
	   reader.read<pcl::PointXYZRGBA> ("/home/niladri-64/module_heisenberg/build/saved_file_box.pcd",*cloud);
	   std::cout << cloud->height << " " << cloud->width << std::endl;
	   cv::Mat image(cloud->height, cloud->width, CV_8UC3);
	   cv::Mat depth(cloud->height, cloud->width, CV_32FC1 );

	   for (int h=0; h<image.rows; h++)
	   {
	       for (int w=0; w<image.cols; w++)
	       {
	               pcl::PointXYZRGBA point = cloud->at(w, h);

	               Eigen::Vector3i rgb = point.getRGBVector3i();

	               image.at<cv::Vec3b>(h,w)[0] = rgb[2];
	               image.at<cv::Vec3b>(h,w)[1] = rgb[1];
	               image.at<cv::Vec3b>(h,w)[2] = rgb[0];

	               depth.at<float>(h,w) = point.z;

	       }

	   }

	   cv::imwrite("/home/niladri-64/module_heisenberg/build/Image.jpg", image );
//	   cv::imwrite("Depth_Image.jpg", depth);

// Imshow a modulo depth map of the point cloud
	   cv::Mat depth_norm(cloud->height, cloud->width, CV_32FC1);

//	   std::cout << *std::max_element(depth.begin<float>(), depth.end<float>()) << " " << (25.2 % 2) << std::endl;
	   depth_norm = 255*depth / (*std::max_element(depth.begin<float>(), depth.end<float>()));
	   cv::imwrite("Depth_Image.jpg", depth_norm);

	   cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
	   cv::imshow( "Display window", depth_norm );

	   cv::Mat depth_mod(cloud->height, cloud->width, CV_32FC1);


	   for(int a = 0; a < depth_norm.rows; a++)
	   {
		   for(int b = 0; b < depth.cols; b++)
		   {
			   depth_mod.at<float>(a,b) = int(ceil(depth_norm.at<float>(a,b))) % 2;
			   depth_mod.at<float>(a,b) = depth_mod.at<float>(a,b)*(255/1);
		   }
	   }

	   cv::namedWindow( "Modulo Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
	   cv::imshow( "Modulo Display window", depth_mod );

	   cv::waitKey(0);

	return 0;
}











