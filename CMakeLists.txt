cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(module_heisenberg)

find_package(PCL 1.7.1 REQUIRED)
find_package( OpenCV REQUIRED )
find_package(aruco REQUIRED )

find_library(EIGEN_LIB eigen $ENV{EIGEN_LIB_PATH})

include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (test_data_write_mod test_data_write_mod.cpp)
target_link_libraries (test_data_write_mod ${PCL_LIBRARIES})

add_executable (cluster_cube cluster_cube.cpp)
target_link_libraries (cluster_cube ${PCL_LIBRARIES})

add_executable (gen_abs_orient gen_abs_orient.cpp)

add_executable (Inverse_Kinematics Inverse_Kinematics.cpp)
add_executable (multiple_solutions multiple_solutions.cpp)

add_executable (create_input create_input.cpp)

add_executable( saving_private_cloud saving_private_cloud.cpp )
target_link_libraries( saving_private_cloud ${PCL_LIBRARIES} )

add_executable( best_circle best_circle.cpp )
target_link_libraries( best_circle ${PCL_LIBRARIES} )

add_executable( pcd_rgb_depth pcd_rgb_depth.cpp )
target_link_libraries( pcd_rgb_depth ${OpenCV_LIBS} ${PCL_LIBRARIES} )

add_executable( marker_detection marker_detection.cpp )
target_link_libraries( marker_detection ${OpenCV_LIBS} ${PCL_LIBRARIES} ${aruco_LIBS})

#add_executable( demonstration demonstration.cpp )
#target_link_libraries( demonstration ${OpenCV_LIBS} ${PCL_LIBRARIES} ${aruco_LIBS})

#add_executable( demonstration_mod demonstration_mod.cpp )
#target_link_libraries( demonstration_mod ${OpenCV_LIBS} ${PCL_LIBRARIES} ${aruco_LIBS})

#add_executable( demonstration_general demonstration_general.cpp )
#target_link_libraries( demonstration_general ${OpenCV_LIBS} ${PCL_LIBRARIES} ${aruco_LIBS})

add_executable( demonstration_general_v3 demonstration_general_v3.cpp )
target_link_libraries( demonstration_general_v3 ${OpenCV_LIBS} ${PCL_LIBRARIES} ${aruco_LIBS})




add_executable( start_execution start_execution.cpp )
target_link_libraries( start_execution ${OpenCV_LIBS} ${PCL_LIBRARIES} ${aruco_LIBS})




add_executable (create_inference_input create_inference_input.cpp)
add_executable (inference_final inference_final.cpp)

add_executable (pick_a_object pick_a_object.cpp)


set(HEADER_FILES /home/niladri-64/module_heisenberg/func_inverse_kine.h /home/niladri-64/module_heisenberg/func_multi_sol.h)
add_executable (basic_modules basic_modules.cpp ${HEADER_FILES})

add_executable (visuospatial visuospatial.cpp)

add_executable (robot_final_instruct robot_final_instruct.cpp)
target_link_libraries( robot_final_instruct ${OpenCV_LIBS} ${PCL_LIBRARIES} ${aruco_LIBS})





