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
include src/modules/manual_control/CMakeFiles/modules__manual_control.dir/depend.make

# Include the progress variables for this target.
include src/modules/manual_control/CMakeFiles/modules__manual_control.dir/progress.make

# Include the compile flags for this target's objects.
include src/modules/manual_control/CMakeFiles/modules__manual_control.dir/flags.make

src/modules/manual_control/CMakeFiles/modules__manual_control.dir/ManualControl.cpp.obj: src/modules/manual_control/CMakeFiles/modules__manual_control.dir/flags.make
src/modules/manual_control/CMakeFiles/modules__manual_control.dir/ManualControl.cpp.obj: ../../src/modules/manual_control/ManualControl.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/modules/manual_control/CMakeFiles/modules__manual_control.dir/ManualControl.cpp.obj"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/manual_control && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/modules__manual_control.dir/ManualControl.cpp.obj -c /home/andrey/Documents/ap/PX4-Autopilot/src/modules/manual_control/ManualControl.cpp

src/modules/manual_control/CMakeFiles/modules__manual_control.dir/ManualControl.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/modules__manual_control.dir/ManualControl.cpp.i"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/manual_control && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/andrey/Documents/ap/PX4-Autopilot/src/modules/manual_control/ManualControl.cpp > CMakeFiles/modules__manual_control.dir/ManualControl.cpp.i

src/modules/manual_control/CMakeFiles/modules__manual_control.dir/ManualControl.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/modules__manual_control.dir/ManualControl.cpp.s"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/manual_control && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/andrey/Documents/ap/PX4-Autopilot/src/modules/manual_control/ManualControl.cpp -o CMakeFiles/modules__manual_control.dir/ManualControl.cpp.s

src/modules/manual_control/CMakeFiles/modules__manual_control.dir/ManualControlSelector.cpp.obj: src/modules/manual_control/CMakeFiles/modules__manual_control.dir/flags.make
src/modules/manual_control/CMakeFiles/modules__manual_control.dir/ManualControlSelector.cpp.obj: ../../src/modules/manual_control/ManualControlSelector.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/modules/manual_control/CMakeFiles/modules__manual_control.dir/ManualControlSelector.cpp.obj"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/manual_control && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/modules__manual_control.dir/ManualControlSelector.cpp.obj -c /home/andrey/Documents/ap/PX4-Autopilot/src/modules/manual_control/ManualControlSelector.cpp

src/modules/manual_control/CMakeFiles/modules__manual_control.dir/ManualControlSelector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/modules__manual_control.dir/ManualControlSelector.cpp.i"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/manual_control && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/andrey/Documents/ap/PX4-Autopilot/src/modules/manual_control/ManualControlSelector.cpp > CMakeFiles/modules__manual_control.dir/ManualControlSelector.cpp.i

src/modules/manual_control/CMakeFiles/modules__manual_control.dir/ManualControlSelector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/modules__manual_control.dir/ManualControlSelector.cpp.s"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/manual_control && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/andrey/Documents/ap/PX4-Autopilot/src/modules/manual_control/ManualControlSelector.cpp -o CMakeFiles/modules__manual_control.dir/ManualControlSelector.cpp.s

# Object files for target modules__manual_control
modules__manual_control_OBJECTS = \
"CMakeFiles/modules__manual_control.dir/ManualControl.cpp.obj" \
"CMakeFiles/modules__manual_control.dir/ManualControlSelector.cpp.obj"

# External object files for target modules__manual_control
modules__manual_control_EXTERNAL_OBJECTS =

src/modules/manual_control/libmodules__manual_control.a: src/modules/manual_control/CMakeFiles/modules__manual_control.dir/ManualControl.cpp.obj
src/modules/manual_control/libmodules__manual_control.a: src/modules/manual_control/CMakeFiles/modules__manual_control.dir/ManualControlSelector.cpp.obj
src/modules/manual_control/libmodules__manual_control.a: src/modules/manual_control/CMakeFiles/modules__manual_control.dir/build.make
src/modules/manual_control/libmodules__manual_control.a: src/modules/manual_control/CMakeFiles/modules__manual_control.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX static library libmodules__manual_control.a"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/manual_control && $(CMAKE_COMMAND) -P CMakeFiles/modules__manual_control.dir/cmake_clean_target.cmake
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/manual_control && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/modules__manual_control.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/modules/manual_control/CMakeFiles/modules__manual_control.dir/build: src/modules/manual_control/libmodules__manual_control.a

.PHONY : src/modules/manual_control/CMakeFiles/modules__manual_control.dir/build

src/modules/manual_control/CMakeFiles/modules__manual_control.dir/clean:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/manual_control && $(CMAKE_COMMAND) -P CMakeFiles/modules__manual_control.dir/cmake_clean.cmake
.PHONY : src/modules/manual_control/CMakeFiles/modules__manual_control.dir/clean

src/modules/manual_control/CMakeFiles/modules__manual_control.dir/depend:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/andrey/Documents/ap/PX4-Autopilot /home/andrey/Documents/ap/PX4-Autopilot/src/modules/manual_control /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/manual_control /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/manual_control/CMakeFiles/modules__manual_control.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/modules/manual_control/CMakeFiles/modules__manual_control.dir/depend
