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

# Utility rule file for world_magnetic_model_update.

# Include the progress variables for this target.
include src/lib/world_magnetic_model/CMakeFiles/world_magnetic_model_update.dir/progress.make

src/lib/world_magnetic_model/CMakeFiles/world_magnetic_model_update: ../../src/lib/world_magnetic_model/fetch_noaa_table.py
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Updating world magnetic model from NOAA (/home/andrey/Documents/ap/PX4-Autopilot/src/lib/world_magnetic_model/geo_magnetic_tables.hpp)"
	cd /home/andrey/Documents/ap/PX4-Autopilot/src/lib/world_magnetic_model && /usr/bin/python3 /home/andrey/Documents/ap/PX4-Autopilot/src/lib/world_magnetic_model/fetch_noaa_table.py > /home/andrey/Documents/ap/PX4-Autopilot/src/lib/world_magnetic_model/geo_magnetic_tables.hpp

world_magnetic_model_update: src/lib/world_magnetic_model/CMakeFiles/world_magnetic_model_update
world_magnetic_model_update: src/lib/world_magnetic_model/CMakeFiles/world_magnetic_model_update.dir/build.make

.PHONY : world_magnetic_model_update

# Rule to build all files generated by this target.
src/lib/world_magnetic_model/CMakeFiles/world_magnetic_model_update.dir/build: world_magnetic_model_update

.PHONY : src/lib/world_magnetic_model/CMakeFiles/world_magnetic_model_update.dir/build

src/lib/world_magnetic_model/CMakeFiles/world_magnetic_model_update.dir/clean:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/lib/world_magnetic_model && $(CMAKE_COMMAND) -P CMakeFiles/world_magnetic_model_update.dir/cmake_clean.cmake
.PHONY : src/lib/world_magnetic_model/CMakeFiles/world_magnetic_model_update.dir/clean

src/lib/world_magnetic_model/CMakeFiles/world_magnetic_model_update.dir/depend:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/andrey/Documents/ap/PX4-Autopilot /home/andrey/Documents/ap/PX4-Autopilot/src/lib/world_magnetic_model /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/lib/world_magnetic_model /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/lib/world_magnetic_model/CMakeFiles/world_magnetic_model_update.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/lib/world_magnetic_model/CMakeFiles/world_magnetic_model_update.dir/depend

