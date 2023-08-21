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
include src/systemcmds/i2cdetect/CMakeFiles/systemcmds__i2cdetect.dir/depend.make

# Include the progress variables for this target.
include src/systemcmds/i2cdetect/CMakeFiles/systemcmds__i2cdetect.dir/progress.make

# Include the compile flags for this target's objects.
include src/systemcmds/i2cdetect/CMakeFiles/systemcmds__i2cdetect.dir/flags.make

src/systemcmds/i2cdetect/CMakeFiles/systemcmds__i2cdetect.dir/i2cdetect.cpp.obj: src/systemcmds/i2cdetect/CMakeFiles/systemcmds__i2cdetect.dir/flags.make
src/systemcmds/i2cdetect/CMakeFiles/systemcmds__i2cdetect.dir/i2cdetect.cpp.obj: ../../src/systemcmds/i2cdetect/i2cdetect.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/systemcmds/i2cdetect/CMakeFiles/systemcmds__i2cdetect.dir/i2cdetect.cpp.obj"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/systemcmds/i2cdetect && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/systemcmds__i2cdetect.dir/i2cdetect.cpp.obj -c /home/andrey/Documents/ap/PX4-Autopilot/src/systemcmds/i2cdetect/i2cdetect.cpp

src/systemcmds/i2cdetect/CMakeFiles/systemcmds__i2cdetect.dir/i2cdetect.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/systemcmds__i2cdetect.dir/i2cdetect.cpp.i"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/systemcmds/i2cdetect && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/andrey/Documents/ap/PX4-Autopilot/src/systemcmds/i2cdetect/i2cdetect.cpp > CMakeFiles/systemcmds__i2cdetect.dir/i2cdetect.cpp.i

src/systemcmds/i2cdetect/CMakeFiles/systemcmds__i2cdetect.dir/i2cdetect.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/systemcmds__i2cdetect.dir/i2cdetect.cpp.s"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/systemcmds/i2cdetect && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/andrey/Documents/ap/PX4-Autopilot/src/systemcmds/i2cdetect/i2cdetect.cpp -o CMakeFiles/systemcmds__i2cdetect.dir/i2cdetect.cpp.s

# Object files for target systemcmds__i2cdetect
systemcmds__i2cdetect_OBJECTS = \
"CMakeFiles/systemcmds__i2cdetect.dir/i2cdetect.cpp.obj"

# External object files for target systemcmds__i2cdetect
systemcmds__i2cdetect_EXTERNAL_OBJECTS =

src/systemcmds/i2cdetect/libsystemcmds__i2cdetect.a: src/systemcmds/i2cdetect/CMakeFiles/systemcmds__i2cdetect.dir/i2cdetect.cpp.obj
src/systemcmds/i2cdetect/libsystemcmds__i2cdetect.a: src/systemcmds/i2cdetect/CMakeFiles/systemcmds__i2cdetect.dir/build.make
src/systemcmds/i2cdetect/libsystemcmds__i2cdetect.a: src/systemcmds/i2cdetect/CMakeFiles/systemcmds__i2cdetect.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libsystemcmds__i2cdetect.a"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/systemcmds/i2cdetect && $(CMAKE_COMMAND) -P CMakeFiles/systemcmds__i2cdetect.dir/cmake_clean_target.cmake
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/systemcmds/i2cdetect && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/systemcmds__i2cdetect.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/systemcmds/i2cdetect/CMakeFiles/systemcmds__i2cdetect.dir/build: src/systemcmds/i2cdetect/libsystemcmds__i2cdetect.a

.PHONY : src/systemcmds/i2cdetect/CMakeFiles/systemcmds__i2cdetect.dir/build

src/systemcmds/i2cdetect/CMakeFiles/systemcmds__i2cdetect.dir/clean:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/systemcmds/i2cdetect && $(CMAKE_COMMAND) -P CMakeFiles/systemcmds__i2cdetect.dir/cmake_clean.cmake
.PHONY : src/systemcmds/i2cdetect/CMakeFiles/systemcmds__i2cdetect.dir/clean

src/systemcmds/i2cdetect/CMakeFiles/systemcmds__i2cdetect.dir/depend:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/andrey/Documents/ap/PX4-Autopilot /home/andrey/Documents/ap/PX4-Autopilot/src/systemcmds/i2cdetect /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/systemcmds/i2cdetect /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/systemcmds/i2cdetect/CMakeFiles/systemcmds__i2cdetect.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/systemcmds/i2cdetect/CMakeFiles/systemcmds__i2cdetect.dir/depend

