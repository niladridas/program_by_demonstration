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
CMAKE_BINARY_DIR = /home/niladri-64/module_heisenberg/build

# Include any dependencies generated for this target.
include CMakeFiles/saving_private_cloud.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/saving_private_cloud.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/saving_private_cloud.dir/flags.make

CMakeFiles/saving_private_cloud.dir/saving_private_cloud.cpp.o: CMakeFiles/saving_private_cloud.dir/flags.make
CMakeFiles/saving_private_cloud.dir/saving_private_cloud.cpp.o: ../saving_private_cloud.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/niladri-64/module_heisenberg/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/saving_private_cloud.dir/saving_private_cloud.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/saving_private_cloud.dir/saving_private_cloud.cpp.o -c /home/niladri-64/module_heisenberg/saving_private_cloud.cpp

CMakeFiles/saving_private_cloud.dir/saving_private_cloud.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/saving_private_cloud.dir/saving_private_cloud.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/niladri-64/module_heisenberg/saving_private_cloud.cpp > CMakeFiles/saving_private_cloud.dir/saving_private_cloud.cpp.i

CMakeFiles/saving_private_cloud.dir/saving_private_cloud.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/saving_private_cloud.dir/saving_private_cloud.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/niladri-64/module_heisenberg/saving_private_cloud.cpp -o CMakeFiles/saving_private_cloud.dir/saving_private_cloud.cpp.s

CMakeFiles/saving_private_cloud.dir/saving_private_cloud.cpp.o.requires:
.PHONY : CMakeFiles/saving_private_cloud.dir/saving_private_cloud.cpp.o.requires

CMakeFiles/saving_private_cloud.dir/saving_private_cloud.cpp.o.provides: CMakeFiles/saving_private_cloud.dir/saving_private_cloud.cpp.o.requires
	$(MAKE) -f CMakeFiles/saving_private_cloud.dir/build.make CMakeFiles/saving_private_cloud.dir/saving_private_cloud.cpp.o.provides.build
.PHONY : CMakeFiles/saving_private_cloud.dir/saving_private_cloud.cpp.o.provides

CMakeFiles/saving_private_cloud.dir/saving_private_cloud.cpp.o.provides.build: CMakeFiles/saving_private_cloud.dir/saving_private_cloud.cpp.o

# Object files for target saving_private_cloud
saving_private_cloud_OBJECTS = \
"CMakeFiles/saving_private_cloud.dir/saving_private_cloud.cpp.o"

# External object files for target saving_private_cloud
saving_private_cloud_EXTERNAL_OBJECTS =

saving_private_cloud: CMakeFiles/saving_private_cloud.dir/saving_private_cloud.cpp.o
saving_private_cloud: /usr/lib/libboost_system-mt.so
saving_private_cloud: /usr/lib/libboost_filesystem-mt.so
saving_private_cloud: /usr/lib/libboost_thread-mt.so
saving_private_cloud: /usr/lib/libboost_date_time-mt.so
saving_private_cloud: /usr/lib/libboost_iostreams-mt.so
saving_private_cloud: /usr/lib/libboost_serialization-mt.so
saving_private_cloud: /usr/lib/libpcl_common.so
saving_private_cloud: /usr/lib/libflann_cpp_s.a
saving_private_cloud: /usr/lib/libpcl_kdtree.so
saving_private_cloud: /usr/lib/libpcl_octree.so
saving_private_cloud: /usr/lib/libpcl_search.so
saving_private_cloud: /usr/lib/libpcl_sample_consensus.so
saving_private_cloud: /usr/lib/libpcl_filters.so
saving_private_cloud: /usr/lib/libpcl_features.so
saving_private_cloud: /usr/lib/libpcl_keypoints.so
saving_private_cloud: /usr/lib/libOpenNI.so
saving_private_cloud: /usr/lib/libvtkCommon.so.5.8.0
saving_private_cloud: /usr/lib/libvtkRendering.so.5.8.0
saving_private_cloud: /usr/lib/libvtkHybrid.so.5.8.0
saving_private_cloud: /usr/lib/libvtkCharts.so.5.8.0
saving_private_cloud: /usr/lib/libpcl_io.so
saving_private_cloud: /usr/lib/libpcl_segmentation.so
saving_private_cloud: /usr/lib/libqhull.so
saving_private_cloud: /usr/lib/libpcl_surface.so
saving_private_cloud: /usr/lib/libpcl_registration.so
saving_private_cloud: /usr/lib/libpcl_recognition.so
saving_private_cloud: /usr/lib/libpcl_visualization.so
saving_private_cloud: /usr/lib/libpcl_outofcore.so
saving_private_cloud: /usr/lib/libpcl_people.so
saving_private_cloud: /usr/lib/libpcl_tracking.so
saving_private_cloud: /usr/lib/libpcl_apps.so
saving_private_cloud: /usr/lib/libboost_system-mt.so
saving_private_cloud: /usr/lib/libboost_filesystem-mt.so
saving_private_cloud: /usr/lib/libboost_thread-mt.so
saving_private_cloud: /usr/lib/libboost_date_time-mt.so
saving_private_cloud: /usr/lib/libboost_iostreams-mt.so
saving_private_cloud: /usr/lib/libboost_serialization-mt.so
saving_private_cloud: /usr/lib/libqhull.so
saving_private_cloud: /usr/lib/libOpenNI.so
saving_private_cloud: /usr/lib/libflann_cpp_s.a
saving_private_cloud: /usr/lib/libvtkCommon.so.5.8.0
saving_private_cloud: /usr/lib/libvtkRendering.so.5.8.0
saving_private_cloud: /usr/lib/libvtkHybrid.so.5.8.0
saving_private_cloud: /usr/lib/libvtkCharts.so.5.8.0
saving_private_cloud: /usr/lib/libpcl_common.so
saving_private_cloud: /usr/lib/libpcl_kdtree.so
saving_private_cloud: /usr/lib/libpcl_octree.so
saving_private_cloud: /usr/lib/libpcl_search.so
saving_private_cloud: /usr/lib/libpcl_sample_consensus.so
saving_private_cloud: /usr/lib/libpcl_filters.so
saving_private_cloud: /usr/lib/libpcl_features.so
saving_private_cloud: /usr/lib/libpcl_keypoints.so
saving_private_cloud: /usr/lib/libpcl_io.so
saving_private_cloud: /usr/lib/libpcl_segmentation.so
saving_private_cloud: /usr/lib/libpcl_surface.so
saving_private_cloud: /usr/lib/libpcl_registration.so
saving_private_cloud: /usr/lib/libpcl_recognition.so
saving_private_cloud: /usr/lib/libpcl_visualization.so
saving_private_cloud: /usr/lib/libpcl_outofcore.so
saving_private_cloud: /usr/lib/libpcl_people.so
saving_private_cloud: /usr/lib/libpcl_tracking.so
saving_private_cloud: /usr/lib/libpcl_apps.so
saving_private_cloud: /usr/lib/libvtkViews.so.5.8.0
saving_private_cloud: /usr/lib/libvtkInfovis.so.5.8.0
saving_private_cloud: /usr/lib/libvtkWidgets.so.5.8.0
saving_private_cloud: /usr/lib/libvtkHybrid.so.5.8.0
saving_private_cloud: /usr/lib/libvtkParallel.so.5.8.0
saving_private_cloud: /usr/lib/libvtkVolumeRendering.so.5.8.0
saving_private_cloud: /usr/lib/libvtkRendering.so.5.8.0
saving_private_cloud: /usr/lib/libvtkGraphics.so.5.8.0
saving_private_cloud: /usr/lib/libvtkImaging.so.5.8.0
saving_private_cloud: /usr/lib/libvtkIO.so.5.8.0
saving_private_cloud: /usr/lib/libvtkFiltering.so.5.8.0
saving_private_cloud: /usr/lib/libvtkCommon.so.5.8.0
saving_private_cloud: /usr/lib/libvtksys.so.5.8.0
saving_private_cloud: CMakeFiles/saving_private_cloud.dir/build.make
saving_private_cloud: CMakeFiles/saving_private_cloud.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable saving_private_cloud"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/saving_private_cloud.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/saving_private_cloud.dir/build: saving_private_cloud
.PHONY : CMakeFiles/saving_private_cloud.dir/build

CMakeFiles/saving_private_cloud.dir/requires: CMakeFiles/saving_private_cloud.dir/saving_private_cloud.cpp.o.requires
.PHONY : CMakeFiles/saving_private_cloud.dir/requires

CMakeFiles/saving_private_cloud.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/saving_private_cloud.dir/cmake_clean.cmake
.PHONY : CMakeFiles/saving_private_cloud.dir/clean

CMakeFiles/saving_private_cloud.dir/depend:
	cd /home/niladri-64/module_heisenberg/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/niladri-64/module_heisenberg /home/niladri-64/module_heisenberg /home/niladri-64/module_heisenberg/build /home/niladri-64/module_heisenberg/build /home/niladri-64/module_heisenberg/build/CMakeFiles/saving_private_cloud.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/saving_private_cloud.dir/depend

