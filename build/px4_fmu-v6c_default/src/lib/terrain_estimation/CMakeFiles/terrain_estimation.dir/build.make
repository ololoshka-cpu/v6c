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

# Include any dependencies generated for this target.
include src/lib/terrain_estimation/CMakeFiles/terrain_estimation.dir/depend.make

# Include the progress variables for this target.
include src/lib/terrain_estimation/CMakeFiles/terrain_estimation.dir/progress.make

# Include the compile flags for this target's objects.
include src/lib/terrain_estimation/CMakeFiles/terrain_estimation.dir/flags.make

src/lib/terrain_estimation/CMakeFiles/terrain_estimation.dir/terrain_estimator.cpp.obj: src/lib/terrain_estimation/CMakeFiles/terrain_estimation.dir/flags.make
src/lib/terrain_estimation/CMakeFiles/terrain_estimation.dir/terrain_estimator.cpp.obj: ../../src/lib/terrain_estimation/terrain_estimator.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/lib/terrain_estimation/CMakeFiles/terrain_estimation.dir/terrain_estimator.cpp.obj"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/lib/terrain_estimation && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/terrain_estimation.dir/terrain_estimator.cpp.obj -c /home/andrey/Documents/ap/PX4-Autopilot/src/lib/terrain_estimation/terrain_estimator.cpp

src/lib/terrain_estimation/CMakeFiles/terrain_estimation.dir/terrain_estimator.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/terrain_estimation.dir/terrain_estimator.cpp.i"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/lib/terrain_estimation && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/andrey/Documents/ap/PX4-Autopilot/src/lib/terrain_estimation/terrain_estimator.cpp > CMakeFiles/terrain_estimation.dir/terrain_estimator.cpp.i

src/lib/terrain_estimation/CMakeFiles/terrain_estimation.dir/terrain_estimator.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/terrain_estimation.dir/terrain_estimator.cpp.s"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/lib/terrain_estimation && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/andrey/Documents/ap/PX4-Autopilot/src/lib/terrain_estimation/terrain_estimator.cpp -o CMakeFiles/terrain_estimation.dir/terrain_estimator.cpp.s

# Object files for target terrain_estimation
terrain_estimation_OBJECTS = \
"CMakeFiles/terrain_estimation.dir/terrain_estimator.cpp.obj"

# External object files for target terrain_estimation
terrain_estimation_EXTERNAL_OBJECTS =

src/lib/terrain_estimation/libterrain_estimation.a: src/lib/terrain_estimation/CMakeFiles/terrain_estimation.dir/terrain_estimator.cpp.obj
src/lib/terrain_estimation/libterrain_estimation.a: src/lib/terrain_estimation/CMakeFiles/terrain_estimation.dir/build.make
src/lib/terrain_estimation/libterrain_estimation.a: src/lib/terrain_estimation/CMakeFiles/terrain_estimation.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libterrain_estimation.a"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/lib/terrain_estimation && $(CMAKE_COMMAND) -P CMakeFiles/terrain_estimation.dir/cmake_clean_target.cmake
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/lib/terrain_estimation && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/terrain_estimation.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/lib/terrain_estimation/CMakeFiles/terrain_estimation.dir/build: src/lib/terrain_estimation/libterrain_estimation.a

.PHONY : src/lib/terrain_estimation/CMakeFiles/terrain_estimation.dir/build

src/lib/terrain_estimation/CMakeFiles/terrain_estimation.dir/clean:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/lib/terrain_estimation && $(CMAKE_COMMAND) -P CMakeFiles/terrain_estimation.dir/cmake_clean.cmake
.PHONY : src/lib/terrain_estimation/CMakeFiles/terrain_estimation.dir/clean

src/lib/terrain_estimation/CMakeFiles/terrain_estimation.dir/depend:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/andrey/Documents/ap/PX4-Autopilot /home/andrey/Documents/ap/PX4-Autopilot/src/lib/terrain_estimation /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/lib/terrain_estimation /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/lib/terrain_estimation/CMakeFiles/terrain_estimation.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/lib/terrain_estimation/CMakeFiles/terrain_estimation.dir/depend

