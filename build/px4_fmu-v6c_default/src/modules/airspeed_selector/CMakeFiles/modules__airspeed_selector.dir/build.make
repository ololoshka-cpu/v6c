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
include src/modules/airspeed_selector/CMakeFiles/modules__airspeed_selector.dir/depend.make

# Include the progress variables for this target.
include src/modules/airspeed_selector/CMakeFiles/modules__airspeed_selector.dir/progress.make

# Include the compile flags for this target's objects.
include src/modules/airspeed_selector/CMakeFiles/modules__airspeed_selector.dir/flags.make

src/modules/airspeed_selector/CMakeFiles/modules__airspeed_selector.dir/airspeed_selector_main.cpp.obj: src/modules/airspeed_selector/CMakeFiles/modules__airspeed_selector.dir/flags.make
src/modules/airspeed_selector/CMakeFiles/modules__airspeed_selector.dir/airspeed_selector_main.cpp.obj: ../../src/modules/airspeed_selector/airspeed_selector_main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/modules/airspeed_selector/CMakeFiles/modules__airspeed_selector.dir/airspeed_selector_main.cpp.obj"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/airspeed_selector && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/modules__airspeed_selector.dir/airspeed_selector_main.cpp.obj -c /home/andrey/Documents/ap/PX4-Autopilot/src/modules/airspeed_selector/airspeed_selector_main.cpp

src/modules/airspeed_selector/CMakeFiles/modules__airspeed_selector.dir/airspeed_selector_main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/modules__airspeed_selector.dir/airspeed_selector_main.cpp.i"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/airspeed_selector && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/andrey/Documents/ap/PX4-Autopilot/src/modules/airspeed_selector/airspeed_selector_main.cpp > CMakeFiles/modules__airspeed_selector.dir/airspeed_selector_main.cpp.i

src/modules/airspeed_selector/CMakeFiles/modules__airspeed_selector.dir/airspeed_selector_main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/modules__airspeed_selector.dir/airspeed_selector_main.cpp.s"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/airspeed_selector && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/andrey/Documents/ap/PX4-Autopilot/src/modules/airspeed_selector/airspeed_selector_main.cpp -o CMakeFiles/modules__airspeed_selector.dir/airspeed_selector_main.cpp.s

src/modules/airspeed_selector/CMakeFiles/modules__airspeed_selector.dir/AirspeedValidator.cpp.obj: src/modules/airspeed_selector/CMakeFiles/modules__airspeed_selector.dir/flags.make
src/modules/airspeed_selector/CMakeFiles/modules__airspeed_selector.dir/AirspeedValidator.cpp.obj: ../../src/modules/airspeed_selector/AirspeedValidator.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/modules/airspeed_selector/CMakeFiles/modules__airspeed_selector.dir/AirspeedValidator.cpp.obj"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/airspeed_selector && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/modules__airspeed_selector.dir/AirspeedValidator.cpp.obj -c /home/andrey/Documents/ap/PX4-Autopilot/src/modules/airspeed_selector/AirspeedValidator.cpp

src/modules/airspeed_selector/CMakeFiles/modules__airspeed_selector.dir/AirspeedValidator.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/modules__airspeed_selector.dir/AirspeedValidator.cpp.i"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/airspeed_selector && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/andrey/Documents/ap/PX4-Autopilot/src/modules/airspeed_selector/AirspeedValidator.cpp > CMakeFiles/modules__airspeed_selector.dir/AirspeedValidator.cpp.i

src/modules/airspeed_selector/CMakeFiles/modules__airspeed_selector.dir/AirspeedValidator.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/modules__airspeed_selector.dir/AirspeedValidator.cpp.s"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/airspeed_selector && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/andrey/Documents/ap/PX4-Autopilot/src/modules/airspeed_selector/AirspeedValidator.cpp -o CMakeFiles/modules__airspeed_selector.dir/AirspeedValidator.cpp.s

# Object files for target modules__airspeed_selector
modules__airspeed_selector_OBJECTS = \
"CMakeFiles/modules__airspeed_selector.dir/airspeed_selector_main.cpp.obj" \
"CMakeFiles/modules__airspeed_selector.dir/AirspeedValidator.cpp.obj"

# External object files for target modules__airspeed_selector
modules__airspeed_selector_EXTERNAL_OBJECTS =

src/modules/airspeed_selector/libmodules__airspeed_selector.a: src/modules/airspeed_selector/CMakeFiles/modules__airspeed_selector.dir/airspeed_selector_main.cpp.obj
src/modules/airspeed_selector/libmodules__airspeed_selector.a: src/modules/airspeed_selector/CMakeFiles/modules__airspeed_selector.dir/AirspeedValidator.cpp.obj
src/modules/airspeed_selector/libmodules__airspeed_selector.a: src/modules/airspeed_selector/CMakeFiles/modules__airspeed_selector.dir/build.make
src/modules/airspeed_selector/libmodules__airspeed_selector.a: src/modules/airspeed_selector/CMakeFiles/modules__airspeed_selector.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX static library libmodules__airspeed_selector.a"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/airspeed_selector && $(CMAKE_COMMAND) -P CMakeFiles/modules__airspeed_selector.dir/cmake_clean_target.cmake
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/airspeed_selector && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/modules__airspeed_selector.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/modules/airspeed_selector/CMakeFiles/modules__airspeed_selector.dir/build: src/modules/airspeed_selector/libmodules__airspeed_selector.a

.PHONY : src/modules/airspeed_selector/CMakeFiles/modules__airspeed_selector.dir/build

src/modules/airspeed_selector/CMakeFiles/modules__airspeed_selector.dir/clean:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/airspeed_selector && $(CMAKE_COMMAND) -P CMakeFiles/modules__airspeed_selector.dir/cmake_clean.cmake
.PHONY : src/modules/airspeed_selector/CMakeFiles/modules__airspeed_selector.dir/clean

src/modules/airspeed_selector/CMakeFiles/modules__airspeed_selector.dir/depend:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/andrey/Documents/ap/PX4-Autopilot /home/andrey/Documents/ap/PX4-Autopilot/src/modules/airspeed_selector /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/airspeed_selector /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/airspeed_selector/CMakeFiles/modules__airspeed_selector.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/modules/airspeed_selector/CMakeFiles/modules__airspeed_selector.dir/depend

