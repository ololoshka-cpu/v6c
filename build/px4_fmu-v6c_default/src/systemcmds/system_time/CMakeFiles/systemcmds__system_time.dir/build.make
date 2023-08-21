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
include src/systemcmds/system_time/CMakeFiles/systemcmds__system_time.dir/depend.make

# Include the progress variables for this target.
include src/systemcmds/system_time/CMakeFiles/systemcmds__system_time.dir/progress.make

# Include the compile flags for this target's objects.
include src/systemcmds/system_time/CMakeFiles/systemcmds__system_time.dir/flags.make

src/systemcmds/system_time/CMakeFiles/systemcmds__system_time.dir/system_time.cpp.obj: src/systemcmds/system_time/CMakeFiles/systemcmds__system_time.dir/flags.make
src/systemcmds/system_time/CMakeFiles/systemcmds__system_time.dir/system_time.cpp.obj: ../../src/systemcmds/system_time/system_time.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/systemcmds/system_time/CMakeFiles/systemcmds__system_time.dir/system_time.cpp.obj"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/systemcmds/system_time && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/systemcmds__system_time.dir/system_time.cpp.obj -c /home/andrey/Documents/ap/PX4-Autopilot/src/systemcmds/system_time/system_time.cpp

src/systemcmds/system_time/CMakeFiles/systemcmds__system_time.dir/system_time.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/systemcmds__system_time.dir/system_time.cpp.i"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/systemcmds/system_time && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/andrey/Documents/ap/PX4-Autopilot/src/systemcmds/system_time/system_time.cpp > CMakeFiles/systemcmds__system_time.dir/system_time.cpp.i

src/systemcmds/system_time/CMakeFiles/systemcmds__system_time.dir/system_time.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/systemcmds__system_time.dir/system_time.cpp.s"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/systemcmds/system_time && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/andrey/Documents/ap/PX4-Autopilot/src/systemcmds/system_time/system_time.cpp -o CMakeFiles/systemcmds__system_time.dir/system_time.cpp.s

# Object files for target systemcmds__system_time
systemcmds__system_time_OBJECTS = \
"CMakeFiles/systemcmds__system_time.dir/system_time.cpp.obj"

# External object files for target systemcmds__system_time
systemcmds__system_time_EXTERNAL_OBJECTS =

src/systemcmds/system_time/libsystemcmds__system_time.a: src/systemcmds/system_time/CMakeFiles/systemcmds__system_time.dir/system_time.cpp.obj
src/systemcmds/system_time/libsystemcmds__system_time.a: src/systemcmds/system_time/CMakeFiles/systemcmds__system_time.dir/build.make
src/systemcmds/system_time/libsystemcmds__system_time.a: src/systemcmds/system_time/CMakeFiles/systemcmds__system_time.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libsystemcmds__system_time.a"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/systemcmds/system_time && $(CMAKE_COMMAND) -P CMakeFiles/systemcmds__system_time.dir/cmake_clean_target.cmake
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/systemcmds/system_time && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/systemcmds__system_time.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/systemcmds/system_time/CMakeFiles/systemcmds__system_time.dir/build: src/systemcmds/system_time/libsystemcmds__system_time.a

.PHONY : src/systemcmds/system_time/CMakeFiles/systemcmds__system_time.dir/build

src/systemcmds/system_time/CMakeFiles/systemcmds__system_time.dir/clean:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/systemcmds/system_time && $(CMAKE_COMMAND) -P CMakeFiles/systemcmds__system_time.dir/cmake_clean.cmake
.PHONY : src/systemcmds/system_time/CMakeFiles/systemcmds__system_time.dir/clean

src/systemcmds/system_time/CMakeFiles/systemcmds__system_time.dir/depend:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/andrey/Documents/ap/PX4-Autopilot /home/andrey/Documents/ap/PX4-Autopilot/src/systemcmds/system_time /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/systemcmds/system_time /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/systemcmds/system_time/CMakeFiles/systemcmds__system_time.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/systemcmds/system_time/CMakeFiles/systemcmds__system_time.dir/depend

