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
include src/drivers/telemetry/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/depend.make

# Include the progress variables for this target.
include src/drivers/telemetry/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/progress.make

# Include the compile flags for this target's objects.
include src/drivers/telemetry/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/flags.make

src/drivers/telemetry/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/frsky_data.cpp.obj: src/drivers/telemetry/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/flags.make
src/drivers/telemetry/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/frsky_data.cpp.obj: ../../src/drivers/telemetry/frsky_telemetry/frsky_data.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/drivers/telemetry/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/frsky_data.cpp.obj"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/telemetry/frsky_telemetry && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/drivers__frsky_telemetry.dir/frsky_data.cpp.obj -c /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/telemetry/frsky_telemetry/frsky_data.cpp

src/drivers/telemetry/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/frsky_data.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drivers__frsky_telemetry.dir/frsky_data.cpp.i"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/telemetry/frsky_telemetry && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/telemetry/frsky_telemetry/frsky_data.cpp > CMakeFiles/drivers__frsky_telemetry.dir/frsky_data.cpp.i

src/drivers/telemetry/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/frsky_data.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drivers__frsky_telemetry.dir/frsky_data.cpp.s"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/telemetry/frsky_telemetry && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/telemetry/frsky_telemetry/frsky_data.cpp -o CMakeFiles/drivers__frsky_telemetry.dir/frsky_data.cpp.s

src/drivers/telemetry/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/sPort_data.cpp.obj: src/drivers/telemetry/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/flags.make
src/drivers/telemetry/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/sPort_data.cpp.obj: ../../src/drivers/telemetry/frsky_telemetry/sPort_data.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/drivers/telemetry/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/sPort_data.cpp.obj"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/telemetry/frsky_telemetry && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/drivers__frsky_telemetry.dir/sPort_data.cpp.obj -c /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/telemetry/frsky_telemetry/sPort_data.cpp

src/drivers/telemetry/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/sPort_data.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drivers__frsky_telemetry.dir/sPort_data.cpp.i"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/telemetry/frsky_telemetry && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/telemetry/frsky_telemetry/sPort_data.cpp > CMakeFiles/drivers__frsky_telemetry.dir/sPort_data.cpp.i

src/drivers/telemetry/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/sPort_data.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drivers__frsky_telemetry.dir/sPort_data.cpp.s"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/telemetry/frsky_telemetry && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/telemetry/frsky_telemetry/sPort_data.cpp -o CMakeFiles/drivers__frsky_telemetry.dir/sPort_data.cpp.s

src/drivers/telemetry/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/frsky_telemetry.cpp.obj: src/drivers/telemetry/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/flags.make
src/drivers/telemetry/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/frsky_telemetry.cpp.obj: ../../src/drivers/telemetry/frsky_telemetry/frsky_telemetry.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object src/drivers/telemetry/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/frsky_telemetry.cpp.obj"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/telemetry/frsky_telemetry && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/drivers__frsky_telemetry.dir/frsky_telemetry.cpp.obj -c /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/telemetry/frsky_telemetry/frsky_telemetry.cpp

src/drivers/telemetry/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/frsky_telemetry.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drivers__frsky_telemetry.dir/frsky_telemetry.cpp.i"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/telemetry/frsky_telemetry && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/telemetry/frsky_telemetry/frsky_telemetry.cpp > CMakeFiles/drivers__frsky_telemetry.dir/frsky_telemetry.cpp.i

src/drivers/telemetry/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/frsky_telemetry.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drivers__frsky_telemetry.dir/frsky_telemetry.cpp.s"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/telemetry/frsky_telemetry && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/telemetry/frsky_telemetry/frsky_telemetry.cpp -o CMakeFiles/drivers__frsky_telemetry.dir/frsky_telemetry.cpp.s

# Object files for target drivers__frsky_telemetry
drivers__frsky_telemetry_OBJECTS = \
"CMakeFiles/drivers__frsky_telemetry.dir/frsky_data.cpp.obj" \
"CMakeFiles/drivers__frsky_telemetry.dir/sPort_data.cpp.obj" \
"CMakeFiles/drivers__frsky_telemetry.dir/frsky_telemetry.cpp.obj"

# External object files for target drivers__frsky_telemetry
drivers__frsky_telemetry_EXTERNAL_OBJECTS =

src/drivers/telemetry/frsky_telemetry/libdrivers__frsky_telemetry.a: src/drivers/telemetry/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/frsky_data.cpp.obj
src/drivers/telemetry/frsky_telemetry/libdrivers__frsky_telemetry.a: src/drivers/telemetry/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/sPort_data.cpp.obj
src/drivers/telemetry/frsky_telemetry/libdrivers__frsky_telemetry.a: src/drivers/telemetry/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/frsky_telemetry.cpp.obj
src/drivers/telemetry/frsky_telemetry/libdrivers__frsky_telemetry.a: src/drivers/telemetry/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/build.make
src/drivers/telemetry/frsky_telemetry/libdrivers__frsky_telemetry.a: src/drivers/telemetry/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX static library libdrivers__frsky_telemetry.a"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/telemetry/frsky_telemetry && $(CMAKE_COMMAND) -P CMakeFiles/drivers__frsky_telemetry.dir/cmake_clean_target.cmake
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/telemetry/frsky_telemetry && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/drivers__frsky_telemetry.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/drivers/telemetry/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/build: src/drivers/telemetry/frsky_telemetry/libdrivers__frsky_telemetry.a

.PHONY : src/drivers/telemetry/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/build

src/drivers/telemetry/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/clean:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/telemetry/frsky_telemetry && $(CMAKE_COMMAND) -P CMakeFiles/drivers__frsky_telemetry.dir/cmake_clean.cmake
.PHONY : src/drivers/telemetry/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/clean

src/drivers/telemetry/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/depend:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/andrey/Documents/ap/PX4-Autopilot /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/telemetry/frsky_telemetry /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/telemetry/frsky_telemetry /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/telemetry/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/drivers/telemetry/frsky_telemetry/CMakeFiles/drivers__frsky_telemetry.dir/depend

