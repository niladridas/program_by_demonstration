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
include CMakeFiles/visuospatial.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/visuospatial.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/visuospatial.dir/flags.make

CMakeFiles/visuospatial.dir/visuospatial.cpp.o: CMakeFiles/visuospatial.dir/flags.make
CMakeFiles/visuospatial.dir/visuospatial.cpp.o: ../visuospatial.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/niladri-64/module_heisenberg/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/visuospatial.dir/visuospatial.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/visuospatial.dir/visuospatial.cpp.o -c /home/niladri-64/module_heisenberg/visuospatial.cpp

CMakeFiles/visuospatial.dir/visuospatial.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/visuospatial.dir/visuospatial.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/niladri-64/module_heisenberg/visuospatial.cpp > CMakeFiles/visuospatial.dir/visuospatial.cpp.i

CMakeFiles/visuospatial.dir/visuospatial.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/visuospatial.dir/visuospatial.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/niladri-64/module_heisenberg/visuospatial.cpp -o CMakeFiles/visuospatial.dir/visuospatial.cpp.s

CMakeFiles/visuospatial.dir/visuospatial.cpp.o.requires:
.PHONY : CMakeFiles/visuospatial.dir/visuospatial.cpp.o.requires

CMakeFiles/visuospatial.dir/visuospatial.cpp.o.provides: CMakeFiles/visuospatial.dir/visuospatial.cpp.o.requires
	$(MAKE) -f CMakeFiles/visuospatial.dir/build.make CMakeFiles/visuospatial.dir/visuospatial.cpp.o.provides.build
.PHONY : CMakeFiles/visuospatial.dir/visuospatial.cpp.o.provides

CMakeFiles/visuospatial.dir/visuospatial.cpp.o.provides.build: CMakeFiles/visuospatial.dir/visuospatial.cpp.o

# Object files for target visuospatial
visuospatial_OBJECTS = \
"CMakeFiles/visuospatial.dir/visuospatial.cpp.o"

# External object files for target visuospatial
visuospatial_EXTERNAL_OBJECTS =

visuospatial: CMakeFiles/visuospatial.dir/visuospatial.cpp.o
visuospatial: CMakeFiles/visuospatial.dir/build.make
visuospatial: CMakeFiles/visuospatial.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable visuospatial"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/visuospatial.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/visuospatial.dir/build: visuospatial
.PHONY : CMakeFiles/visuospatial.dir/build

CMakeFiles/visuospatial.dir/requires: CMakeFiles/visuospatial.dir/visuospatial.cpp.o.requires
.PHONY : CMakeFiles/visuospatial.dir/requires

CMakeFiles/visuospatial.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/visuospatial.dir/cmake_clean.cmake
.PHONY : CMakeFiles/visuospatial.dir/clean

CMakeFiles/visuospatial.dir/depend:
	cd /home/niladri-64/module_heisenberg/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/niladri-64/module_heisenberg /home/niladri-64/module_heisenberg /home/niladri-64/module_heisenberg/build /home/niladri-64/module_heisenberg/build /home/niladri-64/module_heisenberg/build/CMakeFiles/visuospatial.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/visuospatial.dir/depend
