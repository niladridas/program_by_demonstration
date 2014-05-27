cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(module_heisenberg)

set(MY_CODES_DIRS /home/niladri-64/module_heisenberg/)

## libbarrett
find_package(Barrett REQUIRED)
include_directories(${BARRETT_INCLUDE_DIRS} ${MY_CODES_DIRS}/include)
link_directories(${BARRETT_LIBRARY_DIRS} ${MY_CODES_DIRS}/builds)
add_definitions(${BARRETT_DEFINITIONS})
add_definitions(-o2 -g)

## curses
find_package(Curses REQUIRED)
include_directories(${CURSES_INCLUDE_DIR})

## Point Cloud Library
find_package(PCL 1.7.1 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


## Opencv
find_package( OpenCV REQUIRED )

## Eigen
find_library(EIGEN_LIB eigen $ENV{EIGEN_LIB_PATH})

## AR Marker
find_package(aruco REQUIRED )

function(add_programs)
	foreach (prog ${ARGV})
		add_executable(${prog} main/${prog}.cpp)
		target_link_libraries(${prog} ${BARRETT_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${aruco_LIBS}  )
	endforeach()
endfunction()

add_programs
(

)


