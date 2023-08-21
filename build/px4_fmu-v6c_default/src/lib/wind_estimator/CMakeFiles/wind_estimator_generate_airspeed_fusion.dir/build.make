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

# Utility rule file for wind_estimator_generate_airspeed_fusion.

# Include the progress variables for this target.
include src/lib/wind_estimator/CMakeFiles/wind_estimator_generate_airspeed_fusion.dir/progress.make

src/lib/wind_estimator/CMakeFiles/wind_estimator_generate_airspeed_fusion:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating fuse_airspeed"
	cd /home/andrey/Documents/ap/PX4-Autopilot/src/lib/wind_estimator/python && /usr/bin/python3 derivation.py

wind_estimator_generate_airspeed_fusion: src/lib/wind_estimator/CMakeFiles/wind_estimator_generate_airspeed_fusion
wind_estimator_generate_airspeed_fusion: src/lib/wind_estimator/CMakeFiles/wind_estimator_generate_airspeed_fusion.dir/build.make

.PHONY : wind_estimator_generate_airspeed_fusion

# Rule to build all files generated by this target.
src/lib/wind_estimator/CMakeFiles/wind_estimator_generate_airspeed_fusion.dir/build: wind_estimator_generate_airspeed_fusion

.PHONY : src/lib/wind_estimator/CMakeFiles/wind_estimator_generate_airspeed_fusion.dir/build

src/lib/wind_estimator/CMakeFiles/wind_estimator_generate_airspeed_fusion.dir/clean:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/lib/wind_estimator && $(CMAKE_COMMAND) -P CMakeFiles/wind_estimator_generate_airspeed_fusion.dir/cmake_clean.cmake
.PHONY : src/lib/wind_estimator/CMakeFiles/wind_estimator_generate_airspeed_fusion.dir/clean

src/lib/wind_estimator/CMakeFiles/wind_estimator_generate_airspeed_fusion.dir/depend:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/andrey/Documents/ap/PX4-Autopilot /home/andrey/Documents/ap/PX4-Autopilot/src/lib/wind_estimator /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/lib/wind_estimator /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/lib/wind_estimator/CMakeFiles/wind_estimator_generate_airspeed_fusion.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/lib/wind_estimator/CMakeFiles/wind_estimator_generate_airspeed_fusion.dir/depend

