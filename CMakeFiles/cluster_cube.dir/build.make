# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/cmake-gui

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/niladri-64/module_heisenberg

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/niladri-64/module_heisenberg

# Include any dependencies generated for this target.
include CMakeFiles/cluster_cube.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/cluster_cube.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/cluster_cube.dir/flags.make

CMakeFiles/cluster_cube.dir/cluster_cube.cpp.o: CMakeFiles/cluster_cube.dir/flags.make
CMakeFiles/cluster_cube.dir/cluster_cube.cpp.o: cluster_cube.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/niladri-64/module_heisenberg/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/cluster_cube.dir/cluster_cube.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/cluster_cube.dir/cluster_cube.cpp.o -c /home/niladri-64/module_heisenberg/cluster_cube.cpp

CMakeFiles/cluster_cube.dir/cluster_cube.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cluster_cube.dir/cluster_cube.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/niladri-64/module_heisenberg/cluster_cube.cpp > CMakeFiles/cluster_cube.dir/cluster_cube.cpp.i

CMakeFiles/cluster_cube.dir/cluster_cube.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cluster_cube.dir/cluster_cube.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/niladri-64/module_heisenberg/cluster_cube.cpp -o CMakeFiles/cluster_cube.dir/cluster_cube.cpp.s

CMakeFiles/cluster_cube.dir/cluster_cube.cpp.o.requires:
.PHONY : CMakeFiles/cluster_cube.dir/cluster_cube.cpp.o.requires

CMakeFiles/cluster_cube.dir/cluster_cube.cpp.o.provides: CMakeFiles/cluster_cube.dir/cluster_cube.cpp.o.requires
	$(MAKE) -f CMakeFiles/cluster_cube.dir/build.make CMakeFiles/cluster_cube.dir/cluster_cube.cpp.o.provides.build
.PHONY : CMakeFiles/cluster_cube.dir/cluster_cube.cpp.o.provides

CMakeFiles/cluster_cube.dir/cluster_cube.cpp.o.provides.build: CMakeFiles/cluster_cube.dir/cluster_cube.cpp.o

# Object files for target cluster_cube
cluster_cube_OBJECTS = \
"CMakeFiles/cluster_cube.dir/cluster_cube.cpp.o"

# External object files for target cluster_cube
cluster_cube_EXTERNAL_OBJECTS =

cluster_cube: CMakeFiles/cluster_cube.dir/cluster_cube.cpp.o
cluster_cube: /usr/lib/libboost_system-mt.so
cluster_cube: /usr/lib/libboost_filesystem-mt.so
cluster_cube: /usr/lib/libboost_thread-mt.so
cluster_cube: /usr/lib/libboost_date_time-mt.so
cluster_cube: /usr/lib/libboost_iostreams-mt.so
cluster_cube: /usr/lib/libboost_serialization-mt.so
cluster_cube: /usr/lib/libpcl_common.so
cluster_cube: /usr/lib/libflann_cpp_s.a
cluster_cube: /usr/lib/libpcl_kdtree.so
cluster_cube: /usr/lib/libpcl_octree.so
cluster_cube: /usr/lib/libpcl_search.so
cluster_cube: /usr/lib/libpcl_sample_consensus.so
cluster_cube: /usr/lib/libpcl_filters.so
cluster_cube: /usr/lib/libpcl_features.so
cluster_cube: /usr/lib/libpcl_keypoints.so
cluster_cube: /usr/lib/libOpenNI.so
cluster_cube: /usr/lib/libvtkCommon.so.5.8.0
cluster_cube: /usr/lib/libvtkRendering.so.5.8.0
cluster_cube: /usr/lib/libvtkHybrid.so.5.8.0
cluster_cube: /usr/lib/libvtkCharts.so.5.8.0
cluster_cube: /usr/lib/libpcl_io.so
cluster_cube: /usr/lib/libpcl_segmentation.so
cluster_cube: /usr/lib/libqhull.so
cluster_cube: /usr/lib/libpcl_surface.so
cluster_cube: /usr/lib/libpcl_registration.so
cluster_cube: /usr/lib/libpcl_recognition.so
cluster_cube: /usr/lib/libpcl_visualization.so
cluster_cube: /usr/lib/libpcl_outofcore.so
cluster_cube: /usr/lib/libpcl_people.so
cluster_cube: /usr/lib/libpcl_tracking.so
cluster_cube: /usr/lib/libpcl_apps.so
cluster_cube: /usr/lib/libboost_system-mt.so
cluster_cube: /usr/lib/libboost_filesystem-mt.so
cluster_cube: /usr/lib/libboost_thread-mt.so
cluster_cube: /usr/lib/libboost_date_time-mt.so
cluster_cube: /usr/lib/libboost_iostreams-mt.so
cluster_cube: /usr/lib/libboost_serialization-mt.so
cluster_cube: /usr/lib/libqhull.so
cluster_cube: /usr/lib/libOpenNI.so
cluster_cube: /usr/lib/libflann_cpp_s.a
cluster_cube: /usr/lib/libvtkCommon.so.5.8.0
cluster_cube: /usr/lib/libvtkRendering.so.5.8.0
cluster_cube: /usr/lib/libvtkHybrid.so.5.8.0
cluster_cube: /usr/lib/libvtkCharts.so.5.8.0
cluster_cube: /usr/lib/libpcl_common.so
cluster_cube: /usr/lib/libpcl_kdtree.so
cluster_cube: /usr/lib/libpcl_octree.so
cluster_cube: /usr/lib/libpcl_search.so
cluster_cube: /usr/lib/libpcl_sample_consensus.so
cluster_cube: /usr/lib/libpcl_filters.so
cluster_cube: /usr/lib/libpcl_features.so
cluster_cube: /usr/lib/libpcl_keypoints.so
cluster_cube: /usr/lib/libpcl_io.so
cluster_cube: /usr/lib/libpcl_segmentation.so
cluster_cube: /usr/lib/libpcl_surface.so
cluster_cube: /usr/lib/libpcl_registration.so
cluster_cube: /usr/lib/libpcl_recognition.so
cluster_cube: /usr/lib/libpcl_visualization.so
cluster_cube: /usr/lib/libpcl_outofcore.so
cluster_cube: /usr/lib/libpcl_people.so
cluster_cube: /usr/lib/libpcl_tracking.so
cluster_cube: /usr/lib/libpcl_apps.so
cluster_cube: /usr/lib/libvtkViews.so.5.8.0
cluster_cube: /usr/lib/libvtkInfovis.so.5.8.0
cluster_cube: /usr/lib/libvtkWidgets.so.5.8.0
cluster_cube: /usr/lib/libvtkHybrid.so.5.8.0
cluster_cube: /usr/lib/libvtkParallel.so.5.8.0
cluster_cube: /usr/lib/libvtkVolumeRendering.so.5.8.0
cluster_cube: /usr/lib/libvtkRendering.so.5.8.0
cluster_cube: /usr/lib/libvtkGraphics.so.5.8.0
cluster_cube: /usr/lib/libvtkImaging.so.5.8.0
cluster_cube: /usr/lib/libvtkIO.so.5.8.0
cluster_cube: /usr/lib/libvtkFiltering.so.5.8.0
cluster_cube: /usr/lib/libvtkCommon.so.5.8.0
cluster_cube: /usr/lib/libvtksys.so.5.8.0
cluster_cube: CMakeFiles/cluster_cube.dir/build.make
cluster_cube: CMakeFiles/cluster_cube.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable cluster_cube"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/cluster_cube.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/cluster_cube.dir/build: cluster_cube
.PHONY : CMakeFiles/cluster_cube.dir/build

CMakeFiles/cluster_cube.dir/requires: CMakeFiles/cluster_cube.dir/cluster_cube.cpp.o.requires
.PHONY : CMakeFiles/cluster_cube.dir/requires

CMakeFiles/cluster_cube.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/cluster_cube.dir/cmake_clean.cmake
.PHONY : CMakeFiles/cluster_cube.dir/clean

CMakeFiles/cluster_cube.dir/depend:
	cd /home/niladri-64/module_heisenberg && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/niladri-64/module_heisenberg /home/niladri-64/module_heisenberg /home/niladri-64/module_heisenberg /home/niladri-64/module_heisenberg /home/niladri-64/module_heisenberg/CMakeFiles/cluster_cube.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/cluster_cube.dir/depend

