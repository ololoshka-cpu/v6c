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
include src/drivers/telemetry/hott/CMakeFiles/drivers__hott.dir/depend.make

# Include the progress variables for this target.
include src/drivers/telemetry/hott/CMakeFiles/drivers__hott.dir/progress.make

# Include the compile flags for this target's objects.
include src/drivers/telemetry/hott/CMakeFiles/drivers__hott.dir/flags.make

src/drivers/telemetry/hott/CMakeFiles/drivers__hott.dir/messages.cpp.obj: src/drivers/telemetry/hott/CMakeFiles/drivers__hott.dir/flags.make
src/drivers/telemetry/hott/CMakeFiles/drivers__hott.dir/messages.cpp.obj: ../../src/drivers/telemetry/hott/messages.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/drivers/telemetry/hott/CMakeFiles/drivers__hott.dir/messages.cpp.obj"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/telemetry/hott && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/drivers__hott.dir/messages.cpp.obj -c /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/telemetry/hott/messages.cpp

src/drivers/telemetry/hott/CMakeFiles/drivers__hott.dir/messages.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drivers__hott.dir/messages.cpp.i"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/telemetry/hott && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/telemetry/hott/messages.cpp > CMakeFiles/drivers__hott.dir/messages.cpp.i

src/drivers/telemetry/hott/CMakeFiles/drivers__hott.dir/messages.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drivers__hott.dir/messages.cpp.s"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/telemetry/hott && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/telemetry/hott/messages.cpp -o CMakeFiles/drivers__hott.dir/messages.cpp.s

src/drivers/telemetry/hott/CMakeFiles/drivers__hott.dir/comms.cpp.obj: src/drivers/telemetry/hott/CMakeFiles/drivers__hott.dir/flags.make
src/drivers/telemetry/hott/CMakeFiles/drivers__hott.dir/comms.cpp.obj: ../../src/drivers/telemetry/hott/comms.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/drivers/telemetry/hott/CMakeFiles/drivers__hott.dir/comms.cpp.obj"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/telemetry/hott && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/drivers__hott.dir/comms.cpp.obj -c /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/telemetry/hott/comms.cpp

src/drivers/telemetry/hott/CMakeFiles/drivers__hott.dir/comms.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drivers__hott.dir/comms.cpp.i"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/telemetry/hott && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/telemetry/hott/comms.cpp > CMakeFiles/drivers__hott.dir/comms.cpp.i

src/drivers/telemetry/hott/CMakeFiles/drivers__hott.dir/comms.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drivers__hott.dir/comms.cpp.s"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/telemetry/hott && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/telemetry/hott/comms.cpp -o CMakeFiles/drivers__hott.dir/comms.cpp.s

# Object files for target drivers__hott
drivers__hott_OBJECTS = \
"CMakeFiles/drivers__hott.dir/messages.cpp.obj" \
"CMakeFiles/drivers__hott.dir/comms.cpp.obj"

# External object files for target drivers__hott
drivers__hott_EXTERNAL_OBJECTS =

src/drivers/telemetry/hott/libdrivers__hott.a: src/drivers/telemetry/hott/CMakeFiles/drivers__hott.dir/messages.cpp.obj
src/drivers/telemetry/hott/libdrivers__hott.a: src/drivers/telemetry/hott/CMakeFiles/drivers__hott.dir/comms.cpp.obj
src/drivers/telemetry/hott/libdrivers__hott.a: src/drivers/telemetry/hott/CMakeFiles/drivers__hott.dir/build.make
src/drivers/telemetry/hott/libdrivers__hott.a: src/drivers/telemetry/hott/CMakeFiles/drivers__hott.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX static library libdrivers__hott.a"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/telemetry/hott && $(CMAKE_COMMAND) -P CMakeFiles/drivers__hott.dir/cmake_clean_target.cmake
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/telemetry/hott && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/drivers__hott.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/drivers/telemetry/hott/CMakeFiles/drivers__hott.dir/build: src/drivers/telemetry/hott/libdrivers__hott.a

.PHONY : src/drivers/telemetry/hott/CMakeFiles/drivers__hott.dir/build

src/drivers/telemetry/hott/CMakeFiles/drivers__hott.dir/clean:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/telemetry/hott && $(CMAKE_COMMAND) -P CMakeFiles/drivers__hott.dir/cmake_clean.cmake
.PHONY : src/drivers/telemetry/hott/CMakeFiles/drivers__hott.dir/clean

src/drivers/telemetry/hott/CMakeFiles/drivers__hott.dir/depend:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/andrey/Documents/ap/PX4-Autopilot /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/telemetry/hott /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/telemetry/hott /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/telemetry/hott/CMakeFiles/drivers__hott.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/drivers/telemetry/hott/CMakeFiles/drivers__hott.dir/depend

