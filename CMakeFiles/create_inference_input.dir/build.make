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
include CMakeFiles/create_inference_input.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/create_inference_input.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/create_inference_input.dir/flags.make

CMakeFiles/create_inference_input.dir/create_inference_input.cpp.o: CMakeFiles/create_inference_input.dir/flags.make
CMakeFiles/create_inference_input.dir/create_inference_input.cpp.o: create_inference_input.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/niladri-64/module_heisenberg/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/create_inference_input.dir/create_inference_input.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/create_inference_input.dir/create_inference_input.cpp.o -c /home/niladri-64/module_heisenberg/create_inference_input.cpp

CMakeFiles/create_inference_input.dir/create_inference_input.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/create_inference_input.dir/create_inference_input.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/niladri-64/module_heisenberg/create_inference_input.cpp > CMakeFiles/create_inference_input.dir/create_inference_input.cpp.i

CMakeFiles/create_inference_input.dir/create_inference_input.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/create_inference_input.dir/create_inference_input.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/niladri-64/module_heisenberg/create_inference_input.cpp -o CMakeFiles/create_inference_input.dir/create_inference_input.cpp.s

CMakeFiles/create_inference_input.dir/create_inference_input.cpp.o.requires:
.PHONY : CMakeFiles/create_inference_input.dir/create_inference_input.cpp.o.requires

CMakeFiles/create_inference_input.dir/create_inference_input.cpp.o.provides: CMakeFiles/create_inference_input.dir/create_inference_input.cpp.o.requires
	$(MAKE) -f CMakeFiles/create_inference_input.dir/build.make CMakeFiles/create_inference_input.dir/create_inference_input.cpp.o.provides.build
.PHONY : CMakeFiles/create_inference_input.dir/create_inference_input.cpp.o.provides

CMakeFiles/create_inference_input.dir/create_inference_input.cpp.o.provides.build: CMakeFiles/create_inference_input.dir/create_inference_input.cpp.o

# Object files for target create_inference_input
create_inference_input_OBJECTS = \
"CMakeFiles/create_inference_input.dir/create_inference_input.cpp.o"

# External object files for target create_inference_input
create_inference_input_EXTERNAL_OBJECTS =

create_inference_input: CMakeFiles/create_inference_input.dir/create_inference_input.cpp.o
create_inference_input: CMakeFiles/create_inference_input.dir/build.make
create_inference_input: CMakeFiles/create_inference_input.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable create_inference_input"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/create_inference_input.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/create_inference_input.dir/build: create_inference_input
.PHONY : CMakeFiles/create_inference_input.dir/build

CMakeFiles/create_inference_input.dir/requires: CMakeFiles/create_inference_input.dir/create_inference_input.cpp.o.requires
.PHONY : CMakeFiles/create_inference_input.dir/requires

CMakeFiles/create_inference_input.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/create_inference_input.dir/cmake_clean.cmake
.PHONY : CMakeFiles/create_inference_input.dir/clean

CMakeFiles/create_inference_input.dir/depend:
	cd /home/niladri-64/module_heisenberg && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/niladri-64/module_heisenberg /home/niladri-64/module_heisenberg /home/niladri-64/module_heisenberg /home/niladri-64/module_heisenberg /home/niladri-64/module_heisenberg/CMakeFiles/create_inference_input.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/create_inference_input.dir/depend

