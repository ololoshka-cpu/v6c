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
include src/drivers/ins/vectornav/CMakeFiles/drivers__ins__vectornav.dir/depend.make

# Include the progress variables for this target.
include src/drivers/ins/vectornav/CMakeFiles/drivers__ins__vectornav.dir/progress.make

# Include the compile flags for this target's objects.
include src/drivers/ins/vectornav/CMakeFiles/drivers__ins__vectornav.dir/flags.make

src/drivers/ins/vectornav/CMakeFiles/drivers__ins__vectornav.dir/VectorNav.cpp.obj: src/drivers/ins/vectornav/CMakeFiles/drivers__ins__vectornav.dir/flags.make
src/drivers/ins/vectornav/CMakeFiles/drivers__ins__vectornav.dir/VectorNav.cpp.obj: ../../src/drivers/ins/vectornav/VectorNav.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/drivers/ins/vectornav/CMakeFiles/drivers__ins__vectornav.dir/VectorNav.cpp.obj"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/ins/vectornav && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/drivers__ins__vectornav.dir/VectorNav.cpp.obj -c /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/ins/vectornav/VectorNav.cpp

src/drivers/ins/vectornav/CMakeFiles/drivers__ins__vectornav.dir/VectorNav.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drivers__ins__vectornav.dir/VectorNav.cpp.i"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/ins/vectornav && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/ins/vectornav/VectorNav.cpp > CMakeFiles/drivers__ins__vectornav.dir/VectorNav.cpp.i

src/drivers/ins/vectornav/CMakeFiles/drivers__ins__vectornav.dir/VectorNav.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drivers__ins__vectornav.dir/VectorNav.cpp.s"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/ins/vectornav && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/ins/vectornav/VectorNav.cpp -o CMakeFiles/drivers__ins__vectornav.dir/VectorNav.cpp.s

# Object files for target drivers__ins__vectornav
drivers__ins__vectornav_OBJECTS = \
"CMakeFiles/drivers__ins__vectornav.dir/VectorNav.cpp.obj"

# External object files for target drivers__ins__vectornav
drivers__ins__vectornav_EXTERNAL_OBJECTS =

src/drivers/ins/vectornav/libdrivers__ins__vectornav.a: src/drivers/ins/vectornav/CMakeFiles/drivers__ins__vectornav.dir/VectorNav.cpp.obj
src/drivers/ins/vectornav/libdrivers__ins__vectornav.a: src/drivers/ins/vectornav/CMakeFiles/drivers__ins__vectornav.dir/build.make
src/drivers/ins/vectornav/libdrivers__ins__vectornav.a: src/drivers/ins/vectornav/CMakeFiles/drivers__ins__vectornav.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libdrivers__ins__vectornav.a"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/ins/vectornav && $(CMAKE_COMMAND) -P CMakeFiles/drivers__ins__vectornav.dir/cmake_clean_target.cmake
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/ins/vectornav && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/drivers__ins__vectornav.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/drivers/ins/vectornav/CMakeFiles/drivers__ins__vectornav.dir/build: src/drivers/ins/vectornav/libdrivers__ins__vectornav.a

.PHONY : src/drivers/ins/vectornav/CMakeFiles/drivers__ins__vectornav.dir/build

src/drivers/ins/vectornav/CMakeFiles/drivers__ins__vectornav.dir/clean:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/ins/vectornav && $(CMAKE_COMMAND) -P CMakeFiles/drivers__ins__vectornav.dir/cmake_clean.cmake
.PHONY : src/drivers/ins/vectornav/CMakeFiles/drivers__ins__vectornav.dir/clean

src/drivers/ins/vectornav/CMakeFiles/drivers__ins__vectornav.dir/depend:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/andrey/Documents/ap/PX4-Autopilot /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/ins/vectornav /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/ins/vectornav /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/ins/vectornav/CMakeFiles/drivers__ins__vectornav.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/drivers/ins/vectornav/CMakeFiles/drivers__ins__vectornav.dir/depend

