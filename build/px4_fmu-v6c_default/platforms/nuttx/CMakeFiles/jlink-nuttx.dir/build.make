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

# Utility rule file for jlink-nuttx.

# Include the progress variables for this target.
include platforms/nuttx/CMakeFiles/jlink-nuttx.dir/progress.make

platforms/nuttx/CMakeFiles/jlink-nuttx: ../../platforms/nuttx/NuttX/nuttx/tools/jlink-nuttx.so


../../platforms/nuttx/NuttX/nuttx/tools/jlink-nuttx.so: ../../platforms/nuttx/NuttX/nuttx/tools/jlink-nuttx.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating ../../../../platforms/nuttx/NuttX/nuttx/tools/jlink-nuttx.so"
	cd /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/nuttx/tools && make --no-print-directory --silent -f Makefile.host jlink-nuttx

jlink-nuttx: platforms/nuttx/CMakeFiles/jlink-nuttx
jlink-nuttx: ../../platforms/nuttx/NuttX/nuttx/tools/jlink-nuttx.so
jlink-nuttx: platforms/nuttx/CMakeFiles/jlink-nuttx.dir/build.make

.PHONY : jlink-nuttx

# Rule to build all files generated by this target.
platforms/nuttx/CMakeFiles/jlink-nuttx.dir/build: jlink-nuttx

.PHONY : platforms/nuttx/CMakeFiles/jlink-nuttx.dir/build

platforms/nuttx/CMakeFiles/jlink-nuttx.dir/clean:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/platforms/nuttx && $(CMAKE_COMMAND) -P CMakeFiles/jlink-nuttx.dir/cmake_clean.cmake
.PHONY : platforms/nuttx/CMakeFiles/jlink-nuttx.dir/clean

platforms/nuttx/CMakeFiles/jlink-nuttx.dir/depend:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/andrey/Documents/ap/PX4-Autopilot /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/platforms/nuttx /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/platforms/nuttx/CMakeFiles/jlink-nuttx.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : platforms/nuttx/CMakeFiles/jlink-nuttx.dir/depend

