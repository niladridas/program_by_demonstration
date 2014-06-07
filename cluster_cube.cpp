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


boost::shared_ptr<pcl::visualization::PCLVisualizer> shapesVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, double tx, double ty, double tz, double width
	, double height, double depth)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();

  //------------------------------------
  //-----Add shapes at cloud points----
  //------------------------------------
 // viewer->addLine<pcl::PointXYZRGB> (cloud->points[0],
//                                     cloud->points[cloud->size() - 1], "line");
//  viewer->addSphere (cloud->points[0], 0.2, 0.5, 0.5, 0.0, "sphere");

  //---------------------------------------
  //-----Add shapes at other locations-----
  //---------------------------------------
  pcl::ModelCoefficients coeffs;

double qx, qy,qz,qw;
qx=1;qy = 0; qz =0; qw = 0;
coeffs.values.push_back (tx);
coeffs.values.push_back (ty);
coeffs.values.push_back (tz);
coeffs.values.push_back (qx);
coeffs.values.push_back (qy);
coeffs.values.push_back (qz);
coeffs.values.push_back (qw);
coeffs.values.push_back (width);
coeffs.values.push_back (height);
coeffs.values.push_back (depth);
viewer->addCube (coeffs, "cube");

  return (viewer);
}



boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}




boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis (
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
{
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals, 10, 0.05, "normals");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}



boost::shared_ptr<pcl::visualization::PCLVisualizer> viewportsVis (
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud1,pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud2)
{
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->initCameraParameters ();

  int v1(0);
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer->setBackgroundColor (0.3, 0.3, 0.3, v1+1);
  viewer->addText("Radius: 0.01", 10, 10, "v1 text", v1);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb1(cloud1);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud1, rgb1, "sample cloud1", v1);

  int v2(0);
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  viewer->setBackgroundColor (0.3, 0.3, 0.3, v2+1);
  viewer->addText("Radius: 0.1", 10, 10, "v2 text", v2);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb2(cloud2);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud2, rgb2, "sample cloud2", v2);

  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud1");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud2");
  viewer->addCoordinateSystem (1.0);


  return (viewer);
}





int main (int argc, char** argv)
{

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_original(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr result_pass_filter0(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr result_pass_filter1(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr result_pass_filter2(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2_planar(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud3_non_planar(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud4_cylinder(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud4_cylinder_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud5_non_planar_non_cylinder(new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::PCDReader reader;
  //reader.read<pcl::PointXYZRGB> ("/home/niladri-64/plc_kinect_workspace/robot/build/two_objects/saved_file4.pcd",*cloud_original);
  reader.read<pcl::PointXYZRGB> ("/home/niladri-64/module_heisenberg/build/saved_file1.pcd",*cloud_original);

 // TRansforming
std::cout << cloud_original->points.size() << std::endl;
  Eigen::Matrix4f temp_trans;
{

  std::ifstream infile("/home/niladri-64/module_heisenberg/data/translation_matrix.txt");

std::string line;

  int i = 0;
while (std::getline(infile, line))
      {
          std::istringstream iss(line);
          double a, b,c,d;
          if (!(iss >> a >> b >> c >> d)) 
            { break; } // error
            std::cout << a << " " << b << " " << c << " " << d <<std::endl;
          // process pair (a,b)
            
              temp_trans(i,0) =  a;
              temp_trans(i,1) =  b;
              temp_trans(i,2) =  c;
              temp_trans(i,3) = d ;
              

      i ++;
      }

std::cout << temp_trans << std::endl ;

}

 pcl::transformPointCloud(*cloud_original, *cloud, temp_trans );
 std::stringstream oss;
 oss<<"saved_file.pcd";
 pcl::io::savePCDFileASCII (oss.str(), *cloud);

  {
       boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
        viewer = rgbVis(cloud);

      while (!viewer->wasStopped())
            {
             viewer->spinOnce(1000);
            }
  }

  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-0.16, 0.5);
  pass.setInputCloud (cloud);
  pass.filter (*result_pass_filter0);

  pass.setFilterFieldName ("x");
  pass.setFilterLimits (0.57, 1.0);
  pass.setInputCloud (result_pass_filter0);
  pass.filter (*result_pass_filter1);

  pass.setFilterFieldName ("y");
  pass.setFilterLimits (-0.36, 0.25);
  pass.setInputCloud (result_pass_filter1);
  pass.filter (*result_pass_filter2);

  {

      boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

      viewer = rgbVis(result_pass_filter2);
      while (!viewer->wasStopped())
             {
              viewer->spinOnce(1000);
              }
  }

  // Planar Segmentation

  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());

  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
  seg.setOptimizeCoefficients (true);
    // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.01);

    // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;

  int i = 0, nr_points = (int) result_pass_filter2->points.size ();
    // While 30% of the original cloud is still there

std::cout << nr_points << std::endl;
  while (result_pass_filter2->points.size () > 0.5 * nr_points)

  {

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (result_pass_filter2);
    seg.segment (*inliers, *coefficients);

//    std::cerr << "Model coefficients: " << coefficients->values[0] << " "
//                                          << coefficients->values[1] << " "
//                                          << coefficients->values[2] << " "
//                                          << coefficients->values[3] << std::endl;


    if (inliers->indices.size () == 0)
    {
          std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
          break;
    }

    // Extract the inliers
    extract.setInputCloud (result_pass_filter2);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud2_planar);
    std::cerr << "PointCloud representing the planar component: " << cloud2_planar->width * cloud2_planar->height << " data points." << std::endl;


    extract.setNegative (true);
    extract.filter (*cloud3_non_planar);


    std::stringstream ss;
    ss << "table_scene_lms400_plane_" << i << ".pcd";
    //writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);

    {

        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

        viewer = viewportsVis(cloud3_non_planar, result_pass_filter2);
        while (!viewer->wasStopped())
              {
                viewer->spinOnce (1000);
              }

    }


    i++;

    result_pass_filter2.swap (cloud3_non_planar);

  } // while loop


  result_pass_filter2.swap (cloud3_non_planar);

// cloud3_non_planar is again the non-planar component
double cyl_radius;
  { // cylinder segmentation


	  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals3 (new pcl::PointCloud<pcl::Normal>);
	  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
	  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	  // Estimate point normals
	  ne.setSearchMethod (tree);
	  ne.setInputCloud (cloud3_non_planar);
	  ne.setKSearch (50);
	  ne.compute (*cloud_normals3);


    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());

//  pcl::SACSegmentation<pcl::PointXYZRGB> seg2;
    pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg2;
  // Optional
    seg2.setOptimizeCoefficients (true);
    seg2.setModelType (pcl::SACMODEL_CYLINDER);
    seg2.setMethodType (pcl::SAC_RANSAC);
    seg2.setNormalDistanceWeight (0.1);
    seg2.setMaxIterations (10000);
    seg2.setDistanceThreshold (0.07);
    seg2.setRadiusLimits (0, 0.11);
    seg2.setInputCloud (cloud3_non_planar);
    seg2.setInputNormals (cloud_normals3);


    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;

    int i = 0, nr_points = (int) cloud3_non_planar->points.size ();
    // While 30% of the original cloud is still there
//    while (result2->points.size () > 0.5 * nr_points)
//    {

      // Segment the largest planar component from the remaining cloud
    seg2.setInputCloud (cloud3_non_planar);
    seg2.segment (*inliers, *coefficients);

    std::cerr << "Model coefficients of Cylinder : " << coefficients->values[0] << " "
                                                << coefficients->values[1] << " "
                                                << coefficients->values[2] << " "
                                                << coefficients->values[3] << " "
                                                << coefficients->values[4] << " "
                                                << coefficients->values[5] << " "
                                                << coefficients->values[6] << " " <<std::endl;

    std::cout << "Orientation :" << coefficients->values[3] << " " << coefficients->values[4] << " "<< coefficients->values[5] << std::endl;
    cyl_radius = coefficients->values[6];
    if (inliers->indices.size () == 0)
      {
        std::cerr << "Could not estimate a cylinder model for the given dataset." << std::endl;
     //   break;
      }

      // Extract the inliers
    extract.setInputCloud (cloud3_non_planar);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud4_cylinder);
    std::cerr << "PointCloud representing the Cylinder component: " << cloud4_cylinder->width * cloud4_cylinder->height << " data points." << std::endl;

    std::stringstream ss;
    ss << "table_scene_lms400_plane_" << i << ".pcd";
      //writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);

    { // visualizer

	      boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

        viewer = viewportsVis(cloud4_cylinder, cloud3_non_planar);
        while (!viewer->wasStopped())
		    {
			      viewer->spinOnce (1000);
		    }

    }



    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud5_non_planar_non_cylinder);
  //  cloud3_non_planar.swap (cloud5_non_planar_non_cylinder); we are not running a loop

    //i++;



  } // cylinder segmentation ()

  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud (cloud4_cylinder);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*cloud4_cylinder_filtered);
  {
      boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

        viewer = viewportsVis(cloud4_cylinder_filtered, cloud4_cylinder);
        while (!viewer->wasStopped())
        {
            viewer->spinOnce (1000);
        }
  }


  pcl::PointXYZRGB minPt, maxPt;
  pcl::getMinMax3D (*cloud4_cylinder_filtered, minPt, maxPt);
  std::cout << "Max x: " << maxPt.x << std::endl;
  std::cout << "Max y: " << maxPt.y << std::endl;
  std::cout << "Max z: " << maxPt.z << std::endl;
  std::cout << "Min x: " << minPt.x << std::endl;
  std::cout << "Min y: " << minPt.y << std::endl;
  std::cout << "Min z: " << minPt.z << std::endl;

double height =maxPt.y - minPt.y;
double depth = maxPt.z - minPt.z;
double width = maxPt.x - minPt.x;
  std::cout << "Position :" << (minPt.x -0.09) << " " << (maxPt.y + minPt.y)/2 << " "<< (maxPt.z + minPt.z)/2 << std::endl;
 //std::cout << "Orientation :" << coefficients->values[4] << " " << coefficients->values[5] << " "<< coefficients->values[6] << std::endl;

  {
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

          viewer = shapesVis(cloud4_cylinder_filtered,(maxPt.x + minPt.x)/2,(maxPt.y + minPt.y)/2,(maxPt.z + minPt.z)/2,width,height,depth);
          while (!viewer->wasStopped())
          {
              viewer->spinOnce (1000);
          }
    }

  {
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

          viewer = shapesVis(cloud,(maxPt.x + minPt.x)/2,(maxPt.y + minPt.y)/2,(maxPt.z + minPt.z)/2,width,height,depth);
          while (!viewer->wasStopped())
          {
              viewer->spinOnce (1000);
          }
    }
std::cout << coefficients->values[6] << "  "<< cyl_radius << std::endl;
  double x = minPt.x - 0.09 ;//minPt.x + 0.042;//////minPt.x + cyl_radius;//(maxPt.y + minPt.y)/2 ;//minPt.x - 0.09 ;
  double avgy =(maxPt.y + minPt.y)/2 ;// maxPt.y + 0.13;//(maxPt.y + minPt.y)/2 ;
  double avgz = (maxPt.z + minPt.z)/2 + 0.03;//maxPt.z + 0.11;//(maxPt.z + minPt.z)/2 +0.04;//// //maxPt.z + 0.14;   //

  std::ofstream file_cartesian("/home/niladri-64/module_heisenberg/data/end_effector_cartesian.txt");
  if (file_cartesian.is_open())
    {
      file_cartesian << x << " " << avgy  << " "<<avgz << " 1 0 0 0 1 0"  ; // hard coded...should not be done like this
    }

  file_cartesian.close();


//std::cout << "The end effector position is : "<<

return 0;
}
