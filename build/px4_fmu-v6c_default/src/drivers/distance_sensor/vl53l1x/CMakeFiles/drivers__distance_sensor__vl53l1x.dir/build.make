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
include src/drivers/distance_sensor/vl53l1x/CMakeFiles/drivers__distance_sensor__vl53l1x.dir/depend.make

# Include the progress variables for this target.
include src/drivers/distance_sensor/vl53l1x/CMakeFiles/drivers__distance_sensor__vl53l1x.dir/progress.make

# Include the compile flags for this target's objects.
include src/drivers/distance_sensor/vl53l1x/CMakeFiles/drivers__distance_sensor__vl53l1x.dir/flags.make

src/drivers/distance_sensor/vl53l1x/CMakeFiles/drivers__distance_sensor__vl53l1x.dir/vl53l1x.cpp.obj: src/drivers/distance_sensor/vl53l1x/CMakeFiles/drivers__distance_sensor__vl53l1x.dir/flags.make
src/drivers/distance_sensor/vl53l1x/CMakeFiles/drivers__distance_sensor__vl53l1x.dir/vl53l1x.cpp.obj: ../../src/drivers/distance_sensor/vl53l1x/vl53l1x.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/drivers/distance_sensor/vl53l1x/CMakeFiles/drivers__distance_sensor__vl53l1x.dir/vl53l1x.cpp.obj"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/distance_sensor/vl53l1x && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/drivers__distance_sensor__vl53l1x.dir/vl53l1x.cpp.obj -c /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/distance_sensor/vl53l1x/vl53l1x.cpp

src/drivers/distance_sensor/vl53l1x/CMakeFiles/drivers__distance_sensor__vl53l1x.dir/vl53l1x.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drivers__distance_sensor__vl53l1x.dir/vl53l1x.cpp.i"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/distance_sensor/vl53l1x && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/distance_sensor/vl53l1x/vl53l1x.cpp > CMakeFiles/drivers__distance_sensor__vl53l1x.dir/vl53l1x.cpp.i

src/drivers/distance_sensor/vl53l1x/CMakeFiles/drivers__distance_sensor__vl53l1x.dir/vl53l1x.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drivers__distance_sensor__vl53l1x.dir/vl53l1x.cpp.s"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/distance_sensor/vl53l1x && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/distance_sensor/vl53l1x/vl53l1x.cpp -o CMakeFiles/drivers__distance_sensor__vl53l1x.dir/vl53l1x.cpp.s

# Object files for target drivers__distance_sensor__vl53l1x
drivers__distance_sensor__vl53l1x_OBJECTS = \
"CMakeFiles/drivers__distance_sensor__vl53l1x.dir/vl53l1x.cpp.obj"

# External object files for target drivers__distance_sensor__vl53l1x
drivers__distance_sensor__vl53l1x_EXTERNAL_OBJECTS =

src/drivers/distance_sensor/vl53l1x/libdrivers__distance_sensor__vl53l1x.a: src/drivers/distance_sensor/vl53l1x/CMakeFiles/drivers__distance_sensor__vl53l1x.dir/vl53l1x.cpp.obj
src/drivers/distance_sensor/vl53l1x/libdrivers__distance_sensor__vl53l1x.a: src/drivers/distance_sensor/vl53l1x/CMakeFiles/drivers__distance_sensor__vl53l1x.dir/build.make
src/drivers/distance_sensor/vl53l1x/libdrivers__distance_sensor__vl53l1x.a: src/drivers/distance_sensor/vl53l1x/CMakeFiles/drivers__distance_sensor__vl53l1x.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libdrivers__distance_sensor__vl53l1x.a"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/distance_sensor/vl53l1x && $(CMAKE_COMMAND) -P CMakeFiles/drivers__distance_sensor__vl53l1x.dir/cmake_clean_target.cmake
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/distance_sensor/vl53l1x && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/drivers__distance_sensor__vl53l1x.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/drivers/distance_sensor/vl53l1x/CMakeFiles/drivers__distance_sensor__vl53l1x.dir/build: src/drivers/distance_sensor/vl53l1x/libdrivers__distance_sensor__vl53l1x.a

.PHONY : src/drivers/distance_sensor/vl53l1x/CMakeFiles/drivers__distance_sensor__vl53l1x.dir/build

src/drivers/distance_sensor/vl53l1x/CMakeFiles/drivers__distance_sensor__vl53l1x.dir/clean:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/distance_sensor/vl53l1x && $(CMAKE_COMMAND) -P CMakeFiles/drivers__distance_sensor__vl53l1x.dir/cmake_clean.cmake
.PHONY : src/drivers/distance_sensor/vl53l1x/CMakeFiles/drivers__distance_sensor__vl53l1x.dir/clean

src/drivers/distance_sensor/vl53l1x/CMakeFiles/drivers__distance_sensor__vl53l1x.dir/depend:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/andrey/Documents/ap/PX4-Autopilot /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/distance_sensor/vl53l1x /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/distance_sensor/vl53l1x /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/distance_sensor/vl53l1x/CMakeFiles/drivers__distance_sensor__vl53l1x.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/drivers/distance_sensor/vl53l1x/CMakeFiles/drivers__distance_sensor__vl53l1x.dir/depend

