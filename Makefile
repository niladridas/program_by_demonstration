# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

# Default target executed when no arguments are given to make.
default_target: all
.PHONY : default_target

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

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target edit_cache
edit_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake cache editor..."
	/usr/bin/cmake-gui -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : edit_cache

# Special rule for the target edit_cache
edit_cache/fast: edit_cache
.PHONY : edit_cache/fast

# Special rule for the target rebuild_cache
rebuild_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake to regenerate build system..."
	/usr/bin/cmake -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : rebuild_cache

# Special rule for the target rebuild_cache
rebuild_cache/fast: rebuild_cache
.PHONY : rebuild_cache/fast

# The main all target
all: cmake_check_build_system
	$(CMAKE_COMMAND) -E cmake_progress_start /home/niladri-64/module_heisenberg/CMakeFiles /home/niladri-64/module_heisenberg/CMakeFiles/progress.marks
	$(MAKE) -f CMakeFiles/Makefile2 all
	$(CMAKE_COMMAND) -E cmake_progress_start /home/niladri-64/module_heisenberg/CMakeFiles 0
.PHONY : all

# The main clean target
clean:
	$(MAKE) -f CMakeFiles/Makefile2 clean
.PHONY : clean

# The main clean target
clean/fast: clean
.PHONY : clean/fast

# Prepare targets for installation.
preinstall: all
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall

# Prepare targets for installation.
preinstall/fast:
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall/fast

# clear depends
depend:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 1
.PHONY : depend

#=============================================================================
# Target rules for targets named Inverse_Kinematics

# Build rule for target.
Inverse_Kinematics: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 Inverse_Kinematics
.PHONY : Inverse_Kinematics

# fast build rule for target.
Inverse_Kinematics/fast:
	$(MAKE) -f CMakeFiles/Inverse_Kinematics.dir/build.make CMakeFiles/Inverse_Kinematics.dir/build
.PHONY : Inverse_Kinematics/fast

#=============================================================================
# Target rules for targets named best_circle

# Build rule for target.
best_circle: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 best_circle
.PHONY : best_circle

# fast build rule for target.
best_circle/fast:
	$(MAKE) -f CMakeFiles/best_circle.dir/build.make CMakeFiles/best_circle.dir/build
.PHONY : best_circle/fast

#=============================================================================
# Target rules for targets named cluster_cube

# Build rule for target.
cluster_cube: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 cluster_cube
.PHONY : cluster_cube

# fast build rule for target.
cluster_cube/fast:
	$(MAKE) -f CMakeFiles/cluster_cube.dir/build.make CMakeFiles/cluster_cube.dir/build
.PHONY : cluster_cube/fast

#=============================================================================
# Target rules for targets named create_input

# Build rule for target.
create_input: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 create_input
.PHONY : create_input

# fast build rule for target.
create_input/fast:
	$(MAKE) -f CMakeFiles/create_input.dir/build.make CMakeFiles/create_input.dir/build
.PHONY : create_input/fast

#=============================================================================
# Target rules for targets named demonstration

# Build rule for target.
demonstration: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 demonstration
.PHONY : demonstration

# fast build rule for target.
demonstration/fast:
	$(MAKE) -f CMakeFiles/demonstration.dir/build.make CMakeFiles/demonstration.dir/build
.PHONY : demonstration/fast

#=============================================================================
# Target rules for targets named demonstration_mod

# Build rule for target.
demonstration_mod: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 demonstration_mod
.PHONY : demonstration_mod

# fast build rule for target.
demonstration_mod/fast:
	$(MAKE) -f CMakeFiles/demonstration_mod.dir/build.make CMakeFiles/demonstration_mod.dir/build
.PHONY : demonstration_mod/fast

#=============================================================================
# Target rules for targets named gen_abs_orient

# Build rule for target.
gen_abs_orient: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 gen_abs_orient
.PHONY : gen_abs_orient

# fast build rule for target.
gen_abs_orient/fast:
	$(MAKE) -f CMakeFiles/gen_abs_orient.dir/build.make CMakeFiles/gen_abs_orient.dir/build
.PHONY : gen_abs_orient/fast

#=============================================================================
# Target rules for targets named marker_detection

# Build rule for target.
marker_detection: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 marker_detection
.PHONY : marker_detection

# fast build rule for target.
marker_detection/fast:
	$(MAKE) -f CMakeFiles/marker_detection.dir/build.make CMakeFiles/marker_detection.dir/build
.PHONY : marker_detection/fast

#=============================================================================
# Target rules for targets named multiple_solutions

# Build rule for target.
multiple_solutions: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 multiple_solutions
.PHONY : multiple_solutions

# fast build rule for target.
multiple_solutions/fast:
	$(MAKE) -f CMakeFiles/multiple_solutions.dir/build.make CMakeFiles/multiple_solutions.dir/build
.PHONY : multiple_solutions/fast

#=============================================================================
# Target rules for targets named pcd_rgb_depth

# Build rule for target.
pcd_rgb_depth: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 pcd_rgb_depth
.PHONY : pcd_rgb_depth

# fast build rule for target.
pcd_rgb_depth/fast:
	$(MAKE) -f CMakeFiles/pcd_rgb_depth.dir/build.make CMakeFiles/pcd_rgb_depth.dir/build
.PHONY : pcd_rgb_depth/fast

#=============================================================================
# Target rules for targets named saving_private_cloud

# Build rule for target.
saving_private_cloud: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 saving_private_cloud
.PHONY : saving_private_cloud

# fast build rule for target.
saving_private_cloud/fast:
	$(MAKE) -f CMakeFiles/saving_private_cloud.dir/build.make CMakeFiles/saving_private_cloud.dir/build
.PHONY : saving_private_cloud/fast

#=============================================================================
# Target rules for targets named test_data_write_mod

# Build rule for target.
test_data_write_mod: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 test_data_write_mod
.PHONY : test_data_write_mod

# fast build rule for target.
test_data_write_mod/fast:
	$(MAKE) -f CMakeFiles/test_data_write_mod.dir/build.make CMakeFiles/test_data_write_mod.dir/build
.PHONY : test_data_write_mod/fast

Inverse_Kinematics.o: Inverse_Kinematics.cpp.o
.PHONY : Inverse_Kinematics.o

# target to build an object file
Inverse_Kinematics.cpp.o:
	$(MAKE) -f CMakeFiles/Inverse_Kinematics.dir/build.make CMakeFiles/Inverse_Kinematics.dir/Inverse_Kinematics.cpp.o
.PHONY : Inverse_Kinematics.cpp.o

Inverse_Kinematics.i: Inverse_Kinematics.cpp.i
.PHONY : Inverse_Kinematics.i

# target to preprocess a source file
Inverse_Kinematics.cpp.i:
	$(MAKE) -f CMakeFiles/Inverse_Kinematics.dir/build.make CMakeFiles/Inverse_Kinematics.dir/Inverse_Kinematics.cpp.i
.PHONY : Inverse_Kinematics.cpp.i

Inverse_Kinematics.s: Inverse_Kinematics.cpp.s
.PHONY : Inverse_Kinematics.s

# target to generate assembly for a file
Inverse_Kinematics.cpp.s:
	$(MAKE) -f CMakeFiles/Inverse_Kinematics.dir/build.make CMakeFiles/Inverse_Kinematics.dir/Inverse_Kinematics.cpp.s
.PHONY : Inverse_Kinematics.cpp.s

best_circle.o: best_circle.cpp.o
.PHONY : best_circle.o

# target to build an object file
best_circle.cpp.o:
	$(MAKE) -f CMakeFiles/best_circle.dir/build.make CMakeFiles/best_circle.dir/best_circle.cpp.o
.PHONY : best_circle.cpp.o

best_circle.i: best_circle.cpp.i
.PHONY : best_circle.i

# target to preprocess a source file
best_circle.cpp.i:
	$(MAKE) -f CMakeFiles/best_circle.dir/build.make CMakeFiles/best_circle.dir/best_circle.cpp.i
.PHONY : best_circle.cpp.i

best_circle.s: best_circle.cpp.s
.PHONY : best_circle.s

# target to generate assembly for a file
best_circle.cpp.s:
	$(MAKE) -f CMakeFiles/best_circle.dir/build.make CMakeFiles/best_circle.dir/best_circle.cpp.s
.PHONY : best_circle.cpp.s

cluster_cube.o: cluster_cube.cpp.o
.PHONY : cluster_cube.o

# target to build an object file
cluster_cube.cpp.o:
	$(MAKE) -f CMakeFiles/cluster_cube.dir/build.make CMakeFiles/cluster_cube.dir/cluster_cube.cpp.o
.PHONY : cluster_cube.cpp.o

cluster_cube.i: cluster_cube.cpp.i
.PHONY : cluster_cube.i

# target to preprocess a source file
cluster_cube.cpp.i:
	$(MAKE) -f CMakeFiles/cluster_cube.dir/build.make CMakeFiles/cluster_cube.dir/cluster_cube.cpp.i
.PHONY : cluster_cube.cpp.i

cluster_cube.s: cluster_cube.cpp.s
.PHONY : cluster_cube.s

# target to generate assembly for a file
cluster_cube.cpp.s:
	$(MAKE) -f CMakeFiles/cluster_cube.dir/build.make CMakeFiles/cluster_cube.dir/cluster_cube.cpp.s
.PHONY : cluster_cube.cpp.s

create_input.o: create_input.cpp.o
.PHONY : create_input.o

# target to build an object file
create_input.cpp.o:
	$(MAKE) -f CMakeFiles/create_input.dir/build.make CMakeFiles/create_input.dir/create_input.cpp.o
.PHONY : create_input.cpp.o

create_input.i: create_input.cpp.i
.PHONY : create_input.i

# target to preprocess a source file
create_input.cpp.i:
	$(MAKE) -f CMakeFiles/create_input.dir/build.make CMakeFiles/create_input.dir/create_input.cpp.i
.PHONY : create_input.cpp.i

create_input.s: create_input.cpp.s
.PHONY : create_input.s

# target to generate assembly for a file
create_input.cpp.s:
	$(MAKE) -f CMakeFiles/create_input.dir/build.make CMakeFiles/create_input.dir/create_input.cpp.s
.PHONY : create_input.cpp.s

demonstration.o: demonstration.cpp.o
.PHONY : demonstration.o

# target to build an object file
demonstration.cpp.o:
	$(MAKE) -f CMakeFiles/demonstration.dir/build.make CMakeFiles/demonstration.dir/demonstration.cpp.o
.PHONY : demonstration.cpp.o

demonstration.i: demonstration.cpp.i
.PHONY : demonstration.i

# target to preprocess a source file
demonstration.cpp.i:
	$(MAKE) -f CMakeFiles/demonstration.dir/build.make CMakeFiles/demonstration.dir/demonstration.cpp.i
.PHONY : demonstration.cpp.i

demonstration.s: demonstration.cpp.s
.PHONY : demonstration.s

# target to generate assembly for a file
demonstration.cpp.s:
	$(MAKE) -f CMakeFiles/demonstration.dir/build.make CMakeFiles/demonstration.dir/demonstration.cpp.s
.PHONY : demonstration.cpp.s

demonstration_mod.o: demonstration_mod.cpp.o
.PHONY : demonstration_mod.o

# target to build an object file
demonstration_mod.cpp.o:
	$(MAKE) -f CMakeFiles/demonstration_mod.dir/build.make CMakeFiles/demonstration_mod.dir/demonstration_mod.cpp.o
.PHONY : demonstration_mod.cpp.o

demonstration_mod.i: demonstration_mod.cpp.i
.PHONY : demonstration_mod.i

# target to preprocess a source file
demonstration_mod.cpp.i:
	$(MAKE) -f CMakeFiles/demonstration_mod.dir/build.make CMakeFiles/demonstration_mod.dir/demonstration_mod.cpp.i
.PHONY : demonstration_mod.cpp.i

demonstration_mod.s: demonstration_mod.cpp.s
.PHONY : demonstration_mod.s

# target to generate assembly for a file
demonstration_mod.cpp.s:
	$(MAKE) -f CMakeFiles/demonstration_mod.dir/build.make CMakeFiles/demonstration_mod.dir/demonstration_mod.cpp.s
.PHONY : demonstration_mod.cpp.s

gen_abs_orient.o: gen_abs_orient.cpp.o
.PHONY : gen_abs_orient.o

# target to build an object file
gen_abs_orient.cpp.o:
	$(MAKE) -f CMakeFiles/gen_abs_orient.dir/build.make CMakeFiles/gen_abs_orient.dir/gen_abs_orient.cpp.o
.PHONY : gen_abs_orient.cpp.o

gen_abs_orient.i: gen_abs_orient.cpp.i
.PHONY : gen_abs_orient.i

# target to preprocess a source file
gen_abs_orient.cpp.i:
	$(MAKE) -f CMakeFiles/gen_abs_orient.dir/build.make CMakeFiles/gen_abs_orient.dir/gen_abs_orient.cpp.i
.PHONY : gen_abs_orient.cpp.i

gen_abs_orient.s: gen_abs_orient.cpp.s
.PHONY : gen_abs_orient.s

# target to generate assembly for a file
gen_abs_orient.cpp.s:
	$(MAKE) -f CMakeFiles/gen_abs_orient.dir/build.make CMakeFiles/gen_abs_orient.dir/gen_abs_orient.cpp.s
.PHONY : gen_abs_orient.cpp.s

marker_detection.o: marker_detection.cpp.o
.PHONY : marker_detection.o

# target to build an object file
marker_detection.cpp.o:
	$(MAKE) -f CMakeFiles/marker_detection.dir/build.make CMakeFiles/marker_detection.dir/marker_detection.cpp.o
.PHONY : marker_detection.cpp.o

marker_detection.i: marker_detection.cpp.i
.PHONY : marker_detection.i

# target to preprocess a source file
marker_detection.cpp.i:
	$(MAKE) -f CMakeFiles/marker_detection.dir/build.make CMakeFiles/marker_detection.dir/marker_detection.cpp.i
.PHONY : marker_detection.cpp.i

marker_detection.s: marker_detection.cpp.s
.PHONY : marker_detection.s

# target to generate assembly for a file
marker_detection.cpp.s:
	$(MAKE) -f CMakeFiles/marker_detection.dir/build.make CMakeFiles/marker_detection.dir/marker_detection.cpp.s
.PHONY : marker_detection.cpp.s

multiple_solutions.o: multiple_solutions.cpp.o
.PHONY : multiple_solutions.o

# target to build an object file
multiple_solutions.cpp.o:
	$(MAKE) -f CMakeFiles/multiple_solutions.dir/build.make CMakeFiles/multiple_solutions.dir/multiple_solutions.cpp.o
.PHONY : multiple_solutions.cpp.o

multiple_solutions.i: multiple_solutions.cpp.i
.PHONY : multiple_solutions.i

# target to preprocess a source file
multiple_solutions.cpp.i:
	$(MAKE) -f CMakeFiles/multiple_solutions.dir/build.make CMakeFiles/multiple_solutions.dir/multiple_solutions.cpp.i
.PHONY : multiple_solutions.cpp.i

multiple_solutions.s: multiple_solutions.cpp.s
.PHONY : multiple_solutions.s

# target to generate assembly for a file
multiple_solutions.cpp.s:
	$(MAKE) -f CMakeFiles/multiple_solutions.dir/build.make CMakeFiles/multiple_solutions.dir/multiple_solutions.cpp.s
.PHONY : multiple_solutions.cpp.s

pcd_rgb_depth.o: pcd_rgb_depth.cpp.o
.PHONY : pcd_rgb_depth.o

# target to build an object file
pcd_rgb_depth.cpp.o:
	$(MAKE) -f CMakeFiles/pcd_rgb_depth.dir/build.make CMakeFiles/pcd_rgb_depth.dir/pcd_rgb_depth.cpp.o
.PHONY : pcd_rgb_depth.cpp.o

pcd_rgb_depth.i: pcd_rgb_depth.cpp.i
.PHONY : pcd_rgb_depth.i

# target to preprocess a source file
pcd_rgb_depth.cpp.i:
	$(MAKE) -f CMakeFiles/pcd_rgb_depth.dir/build.make CMakeFiles/pcd_rgb_depth.dir/pcd_rgb_depth.cpp.i
.PHONY : pcd_rgb_depth.cpp.i

pcd_rgb_depth.s: pcd_rgb_depth.cpp.s
.PHONY : pcd_rgb_depth.s

# target to generate assembly for a file
pcd_rgb_depth.cpp.s:
	$(MAKE) -f CMakeFiles/pcd_rgb_depth.dir/build.make CMakeFiles/pcd_rgb_depth.dir/pcd_rgb_depth.cpp.s
.PHONY : pcd_rgb_depth.cpp.s

saving_private_cloud.o: saving_private_cloud.cpp.o
.PHONY : saving_private_cloud.o

# target to build an object file
saving_private_cloud.cpp.o:
	$(MAKE) -f CMakeFiles/saving_private_cloud.dir/build.make CMakeFiles/saving_private_cloud.dir/saving_private_cloud.cpp.o
.PHONY : saving_private_cloud.cpp.o

saving_private_cloud.i: saving_private_cloud.cpp.i
.PHONY : saving_private_cloud.i

# target to preprocess a source file
saving_private_cloud.cpp.i:
	$(MAKE) -f CMakeFiles/saving_private_cloud.dir/build.make CMakeFiles/saving_private_cloud.dir/saving_private_cloud.cpp.i
.PHONY : saving_private_cloud.cpp.i

saving_private_cloud.s: saving_private_cloud.cpp.s
.PHONY : saving_private_cloud.s

# target to generate assembly for a file
saving_private_cloud.cpp.s:
	$(MAKE) -f CMakeFiles/saving_private_cloud.dir/build.make CMakeFiles/saving_private_cloud.dir/saving_private_cloud.cpp.s
.PHONY : saving_private_cloud.cpp.s

test_data_write_mod.o: test_data_write_mod.cpp.o
.PHONY : test_data_write_mod.o

# target to build an object file
test_data_write_mod.cpp.o:
	$(MAKE) -f CMakeFiles/test_data_write_mod.dir/build.make CMakeFiles/test_data_write_mod.dir/test_data_write_mod.cpp.o
.PHONY : test_data_write_mod.cpp.o

test_data_write_mod.i: test_data_write_mod.cpp.i
.PHONY : test_data_write_mod.i

# target to preprocess a source file
test_data_write_mod.cpp.i:
	$(MAKE) -f CMakeFiles/test_data_write_mod.dir/build.make CMakeFiles/test_data_write_mod.dir/test_data_write_mod.cpp.i
.PHONY : test_data_write_mod.cpp.i

test_data_write_mod.s: test_data_write_mod.cpp.s
.PHONY : test_data_write_mod.s

# target to generate assembly for a file
test_data_write_mod.cpp.s:
	$(MAKE) -f CMakeFiles/test_data_write_mod.dir/build.make CMakeFiles/test_data_write_mod.dir/test_data_write_mod.cpp.s
.PHONY : test_data_write_mod.cpp.s

# Help Target
help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... all (the default if no target is provided)"
	@echo "... clean"
	@echo "... depend"
	@echo "... Inverse_Kinematics"
	@echo "... best_circle"
	@echo "... cluster_cube"
	@echo "... create_input"
	@echo "... demonstration"
	@echo "... demonstration_mod"
	@echo "... edit_cache"
	@echo "... gen_abs_orient"
	@echo "... marker_detection"
	@echo "... multiple_solutions"
	@echo "... pcd_rgb_depth"
	@echo "... rebuild_cache"
	@echo "... saving_private_cloud"
	@echo "... test_data_write_mod"
	@echo "... Inverse_Kinematics.o"
	@echo "... Inverse_Kinematics.i"
	@echo "... Inverse_Kinematics.s"
	@echo "... best_circle.o"
	@echo "... best_circle.i"
	@echo "... best_circle.s"
	@echo "... cluster_cube.o"
	@echo "... cluster_cube.i"
	@echo "... cluster_cube.s"
	@echo "... create_input.o"
	@echo "... create_input.i"
	@echo "... create_input.s"
	@echo "... demonstration.o"
	@echo "... demonstration.i"
	@echo "... demonstration.s"
	@echo "... demonstration_mod.o"
	@echo "... demonstration_mod.i"
	@echo "... demonstration_mod.s"
	@echo "... gen_abs_orient.o"
	@echo "... gen_abs_orient.i"
	@echo "... gen_abs_orient.s"
	@echo "... marker_detection.o"
	@echo "... marker_detection.i"
	@echo "... marker_detection.s"
	@echo "... multiple_solutions.o"
	@echo "... multiple_solutions.i"
	@echo "... multiple_solutions.s"
	@echo "... pcd_rgb_depth.o"
	@echo "... pcd_rgb_depth.i"
	@echo "... pcd_rgb_depth.s"
	@echo "... saving_private_cloud.o"
	@echo "... saving_private_cloud.i"
	@echo "... saving_private_cloud.s"
	@echo "... test_data_write_mod.o"
	@echo "... test_data_write_mod.i"
	@echo "... test_data_write_mod.s"
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 0
.PHONY : cmake_check_build_system

