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

# Utility rule file for qconfig.

# Include the progress variables for this target.
include NuttX/CMakeFiles/qconfig.dir/progress.make

NuttX/CMakeFiles/qconfig:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Running make qconfig then savedefconfig for nsh"
	cd /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/nuttx && make --no-print-directory --silent savedefconfig
	cd /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/nuttx && /usr/bin/cmake -E copy /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/nuttx/defconfig /home/andrey/Documents/ap/PX4-Autopilot/boards/px4/fmu-v6c/nuttx-config/nsh/defconfig
	cd /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/nuttx && /usr/bin/cmake -E remove -f /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/nuttx/.config

qconfig: NuttX/CMakeFiles/qconfig
qconfig: NuttX/CMakeFiles/qconfig.dir/build.make

.PHONY : qconfig

# Rule to build all files generated by this target.
NuttX/CMakeFiles/qconfig.dir/build: qconfig

.PHONY : NuttX/CMakeFiles/qconfig.dir/build

NuttX/CMakeFiles/qconfig.dir/clean:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/NuttX && $(CMAKE_COMMAND) -P CMakeFiles/qconfig.dir/cmake_clean.cmake
.PHONY : NuttX/CMakeFiles/qconfig.dir/clean

NuttX/CMakeFiles/qconfig.dir/depend:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/andrey/Documents/ap/PX4-Autopilot /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/NuttX /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/NuttX/CMakeFiles/qconfig.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : NuttX/CMakeFiles/qconfig.dir/depend

