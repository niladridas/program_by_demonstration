cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(robot)
set(FLANN_ROOT "/usr/include/flann")

#find_package(Eigen REQUIRED)
find_package(PCL 1.7.1 REQUIRED)
#find_package(Eigen REQUIRED)
find_package( OpenCV REQUIRED )
find_package(aruco REQUIRED )



add_executable( test_opencv test_opencv.cpp )
target_link_libraries( test_opencv ${OpenCV_LIBS} )


find_library(EIGEN_LIB eigen $ENV{EIGEN_LIB_PATH})

#include_directories(Eigen_INCLUDE_DIRS)


#include_directories(EIGEN3_INCLUDE_DIR)
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${HDF5_INCLUDE_DIR})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable (flann_example flann_example.cpp)
target_link_libraries (flann_example ${HDF5_LIBRARIES})

add_executable (test test.cpp)
target_link_libraries (test ${PCL_LIBRARIES})

add_executable (test_data_write test_data_write.cpp)
target_link_libraries (test_data_write ${PCL_LIBRARIES})

add_executable (test_data_write_mod test_data_write_mod.cpp)
target_link_libraries (test_data_write_mod ${PCL_LIBRARIES})
 
#add_executable (kinect_grabber_multi_thread kinect_grabber_multi_thread.cpp)
#target_link_libraries (kinect_grabber_multi_thread ${PCL_LIBRARIES})

#add_executable (green_detection green_detection.cpp)
#target_link_libraries (green_detection ${PCL_LIBRARIES})

#add_executable (voxel_grid.cpp voxel_grid.cpp)
#target_link_libraries (voxel_grid ${PCL_LIBRARIES})

#add_executable (deep_network.cpp deep_network.cpp)
#target_link_libraries (deep_network ${PCL_LIBRARIES})

add_executable (pc_cam_robo_trans pc_cam_robo_trans.cpp)
target_link_libraries (pc_cam_robo_trans ${PCL_LIBRARIES})

add_executable (pass_through pass_through.cpp)
target_link_libraries (pass_through ${PCL_LIBRARIES})

add_executable (cluster_cube cluster_cube.cpp)
target_link_libraries (cluster_cube ${PCL_LIBRARIES})


add_executable (absolute_orientation absolute_orientation.cpp)
add_executable (gen_abs_orient gen_abs_orient.cpp)
add_executable (theta_jacobian_xyz theta_jacobian_xyz.cpp)
add_executable (rotation_matrix rotation_matrix.cpp)
add_executable (Inverse_Kinematics Inverse_Kinematics.cpp)
add_executable (multiple_solutions multiple_solutions.cpp)
#add_executable (special_rrt special_rrt.cpp)
add_executable (circlefit3d circlefit3d.cpp)
add_executable (create_input create_input.cpp)


add_executable( pcd_rgb_d pcd_rgb_d.cpp )
target_link_libraries( pcd_rgb_d ${OpenCV_LIBS} ${PCL_LIBRARIES} )

add_executable( pcd_rgb_depth_viewer pcd_rgb_depth_viewer.cpp )
target_link_libraries( pcd_rgb_depth_viewer ${OpenCV_LIBS} ${PCL_LIBRARIES} ${aruco_LIBS})



add_executable( pass_through_edit1 pass_through_edit1.cpp )
target_link_libraries( pass_through_edit1 ${PCL_LIBRARIES} )

add_executable( saving_private_cloud saving_private_cloud.cpp )
target_link_libraries( saving_private_cloud ${PCL_LIBRARIES} )

add_executable( best_circle best_circle.cpp )
target_link_libraries( best_circle ${PCL_LIBRARIES} )



add_executable( extract_table extract_table.cpp )
target_link_libraries( extract_table ${PCL_LIBRARIES} )

add_executable( rakesh_track rakesh_track.cpp )
target_link_libraries( rakesh_track ${OpenCV_LIBS})










