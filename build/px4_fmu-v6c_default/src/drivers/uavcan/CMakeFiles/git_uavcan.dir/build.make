# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


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

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/andrey/Documents/ap/PX4-Autopilot

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default

# Utility rule file for git_uavcan.

# Include the progress variables for this target.
include src/drivers/uavcan/CMakeFiles/git_uavcan.dir/progress.make

src/drivers/uavcan/CMakeFiles/git_uavcan: src/drivers/uavcan/git_init__home_andrey_Documents_ap_PX4-Autopilot_src_drivers_uavcan_libuavcan.stamp


src/drivers/uavcan/git_init__home_andrey_Documents_ap_PX4-Autopilot_src_drivers_uavcan_libuavcan.stamp: ../../.gitmodules
src/drivers/uavcan/git_init__home_andrey_Documents_ap_PX4-Autopilot_src_drivers_uavcan_libuavcan.stamp: ../../src/drivers/uavcan/libuavcan/.git
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "git submodule src/drivers/uavcan/libuavcan"
	cd /home/andrey/Documents/ap/PX4-Autopilot && Tools/check_submodules.sh src/drivers/uavcan/libuavcan
	cd /home/andrey/Documents/ap/PX4-Autopilot && /usr/bin/cmake -E touch /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/uavcan/git_init__home_andrey_Documents_ap_PX4-Autopilot_src_drivers_uavcan_libuavcan.stamp

git_uavcan: src/drivers/uavcan/CMakeFiles/git_uavcan
git_uavcan: src/drivers/uavcan/git_init__home_andrey_Documents_ap_PX4-Autopilot_src_drivers_uavcan_libuavcan.stamp
git_uavcan: src/drivers/uavcan/CMakeFiles/git_uavcan.dir/build.make

.PHONY : git_uavcan

# Rule to build all files generated by this target.
src/drivers/uavcan/CMakeFiles/git_uavcan.dir/build: git_uavcan

.PHONY : src/drivers/uavcan/CMakeFiles/git_uavcan.dir/build

src/drivers/uavcan/CMakeFiles/git_uavcan.dir/clean:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/uavcan && $(CMAKE_COMMAND) -P CMakeFiles/git_uavcan.dir/cmake_clean.cmake
.PHONY : src/drivers/uavcan/CMakeFiles/git_uavcan.dir/clean

src/drivers/uavcan/CMakeFiles/git_uavcan.dir/depend:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/andrey/Documents/ap/PX4-Autopilot /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/uavcan /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/uavcan /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/uavcan/CMakeFiles/git_uavcan.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/drivers/uavcan/CMakeFiles/git_uavcan.dir/depend

