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
include src/modules/commander/failure_detector/CMakeFiles/failure_detector.dir/depend.make

# Include the progress variables for this target.
include src/modules/commander/failure_detector/CMakeFiles/failure_detector.dir/progress.make

# Include the compile flags for this target's objects.
include src/modules/commander/failure_detector/CMakeFiles/failure_detector.dir/flags.make

src/modules/commander/failure_detector/CMakeFiles/failure_detector.dir/FailureDetector.cpp.obj: src/modules/commander/failure_detector/CMakeFiles/failure_detector.dir/flags.make
src/modules/commander/failure_detector/CMakeFiles/failure_detector.dir/FailureDetector.cpp.obj: ../../src/modules/commander/failure_detector/FailureDetector.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/modules/commander/failure_detector/CMakeFiles/failure_detector.dir/FailureDetector.cpp.obj"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/commander/failure_detector && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/failure_detector.dir/FailureDetector.cpp.obj -c /home/andrey/Documents/ap/PX4-Autopilot/src/modules/commander/failure_detector/FailureDetector.cpp

src/modules/commander/failure_detector/CMakeFiles/failure_detector.dir/FailureDetector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/failure_detector.dir/FailureDetector.cpp.i"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/commander/failure_detector && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/andrey/Documents/ap/PX4-Autopilot/src/modules/commander/failure_detector/FailureDetector.cpp > CMakeFiles/failure_detector.dir/FailureDetector.cpp.i

src/modules/commander/failure_detector/CMakeFiles/failure_detector.dir/FailureDetector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/failure_detector.dir/FailureDetector.cpp.s"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/commander/failure_detector && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/andrey/Documents/ap/PX4-Autopilot/src/modules/commander/failure_detector/FailureDetector.cpp -o CMakeFiles/failure_detector.dir/FailureDetector.cpp.s

# Object files for target failure_detector
failure_detector_OBJECTS = \
"CMakeFiles/failure_detector.dir/FailureDetector.cpp.obj"

# External object files for target failure_detector
failure_detector_EXTERNAL_OBJECTS =

src/modules/commander/failure_detector/libfailure_detector.a: src/modules/commander/failure_detector/CMakeFiles/failure_detector.dir/FailureDetector.cpp.obj
src/modules/commander/failure_detector/libfailure_detector.a: src/modules/commander/failure_detector/CMakeFiles/failure_detector.dir/build.make
src/modules/commander/failure_detector/libfailure_detector.a: src/modules/commander/failure_detector/CMakeFiles/failure_detector.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libfailure_detector.a"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/commander/failure_detector && $(CMAKE_COMMAND) -P CMakeFiles/failure_detector.dir/cmake_clean_target.cmake
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/commander/failure_detector && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/failure_detector.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/modules/commander/failure_detector/CMakeFiles/failure_detector.dir/build: src/modules/commander/failure_detector/libfailure_detector.a

.PHONY : src/modules/commander/failure_detector/CMakeFiles/failure_detector.dir/build

src/modules/commander/failure_detector/CMakeFiles/failure_detector.dir/clean:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/commander/failure_detector && $(CMAKE_COMMAND) -P CMakeFiles/failure_detector.dir/cmake_clean.cmake
.PHONY : src/modules/commander/failure_detector/CMakeFiles/failure_detector.dir/clean

src/modules/commander/failure_detector/CMakeFiles/failure_detector.dir/depend:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/andrey/Documents/ap/PX4-Autopilot /home/andrey/Documents/ap/PX4-Autopilot/src/modules/commander/failure_detector /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/commander/failure_detector /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/commander/failure_detector/CMakeFiles/failure_detector.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/modules/commander/failure_detector/CMakeFiles/failure_detector.dir/depend

