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
include src/drivers/magnetometer/vtrantech/vcm1193l/CMakeFiles/drivers__magnetometer__vcm1193l.dir/depend.make

# Include the progress variables for this target.
include src/drivers/magnetometer/vtrantech/vcm1193l/CMakeFiles/drivers__magnetometer__vcm1193l.dir/progress.make

# Include the compile flags for this target's objects.
include src/drivers/magnetometer/vtrantech/vcm1193l/CMakeFiles/drivers__magnetometer__vcm1193l.dir/flags.make

src/drivers/magnetometer/vtrantech/vcm1193l/CMakeFiles/drivers__magnetometer__vcm1193l.dir/VCM1193L.cpp.obj: src/drivers/magnetometer/vtrantech/vcm1193l/CMakeFiles/drivers__magnetometer__vcm1193l.dir/flags.make
src/drivers/magnetometer/vtrantech/vcm1193l/CMakeFiles/drivers__magnetometer__vcm1193l.dir/VCM1193L.cpp.obj: ../../src/drivers/magnetometer/vtrantech/vcm1193l/VCM1193L.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/drivers/magnetometer/vtrantech/vcm1193l/CMakeFiles/drivers__magnetometer__vcm1193l.dir/VCM1193L.cpp.obj"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/magnetometer/vtrantech/vcm1193l && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/drivers__magnetometer__vcm1193l.dir/VCM1193L.cpp.obj -c /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/magnetometer/vtrantech/vcm1193l/VCM1193L.cpp

src/drivers/magnetometer/vtrantech/vcm1193l/CMakeFiles/drivers__magnetometer__vcm1193l.dir/VCM1193L.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drivers__magnetometer__vcm1193l.dir/VCM1193L.cpp.i"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/magnetometer/vtrantech/vcm1193l && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/magnetometer/vtrantech/vcm1193l/VCM1193L.cpp > CMakeFiles/drivers__magnetometer__vcm1193l.dir/VCM1193L.cpp.i

src/drivers/magnetometer/vtrantech/vcm1193l/CMakeFiles/drivers__magnetometer__vcm1193l.dir/VCM1193L.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drivers__magnetometer__vcm1193l.dir/VCM1193L.cpp.s"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/magnetometer/vtrantech/vcm1193l && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/magnetometer/vtrantech/vcm1193l/VCM1193L.cpp -o CMakeFiles/drivers__magnetometer__vcm1193l.dir/VCM1193L.cpp.s

src/drivers/magnetometer/vtrantech/vcm1193l/CMakeFiles/drivers__magnetometer__vcm1193l.dir/vcm1193l_main.cpp.obj: src/drivers/magnetometer/vtrantech/vcm1193l/CMakeFiles/drivers__magnetometer__vcm1193l.dir/flags.make
src/drivers/magnetometer/vtrantech/vcm1193l/CMakeFiles/drivers__magnetometer__vcm1193l.dir/vcm1193l_main.cpp.obj: ../../src/drivers/magnetometer/vtrantech/vcm1193l/vcm1193l_main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/drivers/magnetometer/vtrantech/vcm1193l/CMakeFiles/drivers__magnetometer__vcm1193l.dir/vcm1193l_main.cpp.obj"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/magnetometer/vtrantech/vcm1193l && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/drivers__magnetometer__vcm1193l.dir/vcm1193l_main.cpp.obj -c /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/magnetometer/vtrantech/vcm1193l/vcm1193l_main.cpp

src/drivers/magnetometer/vtrantech/vcm1193l/CMakeFiles/drivers__magnetometer__vcm1193l.dir/vcm1193l_main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drivers__magnetometer__vcm1193l.dir/vcm1193l_main.cpp.i"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/magnetometer/vtrantech/vcm1193l && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/magnetometer/vtrantech/vcm1193l/vcm1193l_main.cpp > CMakeFiles/drivers__magnetometer__vcm1193l.dir/vcm1193l_main.cpp.i

src/drivers/magnetometer/vtrantech/vcm1193l/CMakeFiles/drivers__magnetometer__vcm1193l.dir/vcm1193l_main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drivers__magnetometer__vcm1193l.dir/vcm1193l_main.cpp.s"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/magnetometer/vtrantech/vcm1193l && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/magnetometer/vtrantech/vcm1193l/vcm1193l_main.cpp -o CMakeFiles/drivers__magnetometer__vcm1193l.dir/vcm1193l_main.cpp.s

# Object files for target drivers__magnetometer__vcm1193l
drivers__magnetometer__vcm1193l_OBJECTS = \
"CMakeFiles/drivers__magnetometer__vcm1193l.dir/VCM1193L.cpp.obj" \
"CMakeFiles/drivers__magnetometer__vcm1193l.dir/vcm1193l_main.cpp.obj"

# External object files for target drivers__magnetometer__vcm1193l
drivers__magnetometer__vcm1193l_EXTERNAL_OBJECTS =

src/drivers/magnetometer/vtrantech/vcm1193l/libdrivers__magnetometer__vcm1193l.a: src/drivers/magnetometer/vtrantech/vcm1193l/CMakeFiles/drivers__magnetometer__vcm1193l.dir/VCM1193L.cpp.obj
src/drivers/magnetometer/vtrantech/vcm1193l/libdrivers__magnetometer__vcm1193l.a: src/drivers/magnetometer/vtrantech/vcm1193l/CMakeFiles/drivers__magnetometer__vcm1193l.dir/vcm1193l_main.cpp.obj
src/drivers/magnetometer/vtrantech/vcm1193l/libdrivers__magnetometer__vcm1193l.a: src/drivers/magnetometer/vtrantech/vcm1193l/CMakeFiles/drivers__magnetometer__vcm1193l.dir/build.make
src/drivers/magnetometer/vtrantech/vcm1193l/libdrivers__magnetometer__vcm1193l.a: src/drivers/magnetometer/vtrantech/vcm1193l/CMakeFiles/drivers__magnetometer__vcm1193l.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX static library libdrivers__magnetometer__vcm1193l.a"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/magnetometer/vtrantech/vcm1193l && $(CMAKE_COMMAND) -P CMakeFiles/drivers__magnetometer__vcm1193l.dir/cmake_clean_target.cmake
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/magnetometer/vtrantech/vcm1193l && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/drivers__magnetometer__vcm1193l.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/drivers/magnetometer/vtrantech/vcm1193l/CMakeFiles/drivers__magnetometer__vcm1193l.dir/build: src/drivers/magnetometer/vtrantech/vcm1193l/libdrivers__magnetometer__vcm1193l.a

.PHONY : src/drivers/magnetometer/vtrantech/vcm1193l/CMakeFiles/drivers__magnetometer__vcm1193l.dir/build

src/drivers/magnetometer/vtrantech/vcm1193l/CMakeFiles/drivers__magnetometer__vcm1193l.dir/clean:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/magnetometer/vtrantech/vcm1193l && $(CMAKE_COMMAND) -P CMakeFiles/drivers__magnetometer__vcm1193l.dir/cmake_clean.cmake
.PHONY : src/drivers/magnetometer/vtrantech/vcm1193l/CMakeFiles/drivers__magnetometer__vcm1193l.dir/clean

src/drivers/magnetometer/vtrantech/vcm1193l/CMakeFiles/drivers__magnetometer__vcm1193l.dir/depend:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/andrey/Documents/ap/PX4-Autopilot /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/magnetometer/vtrantech/vcm1193l /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/magnetometer/vtrantech/vcm1193l /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/magnetometer/vtrantech/vcm1193l/CMakeFiles/drivers__magnetometer__vcm1193l.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/drivers/magnetometer/vtrantech/vcm1193l/CMakeFiles/drivers__magnetometer__vcm1193l.dir/depend
