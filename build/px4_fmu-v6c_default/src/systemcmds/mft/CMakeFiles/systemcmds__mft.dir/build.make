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
include src/systemcmds/mft/CMakeFiles/systemcmds__mft.dir/depend.make

# Include the progress variables for this target.
include src/systemcmds/mft/CMakeFiles/systemcmds__mft.dir/progress.make

# Include the compile flags for this target's objects.
include src/systemcmds/mft/CMakeFiles/systemcmds__mft.dir/flags.make

src/systemcmds/mft/CMakeFiles/systemcmds__mft.dir/mft.cpp.obj: src/systemcmds/mft/CMakeFiles/systemcmds__mft.dir/flags.make
src/systemcmds/mft/CMakeFiles/systemcmds__mft.dir/mft.cpp.obj: ../../src/systemcmds/mft/mft.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/systemcmds/mft/CMakeFiles/systemcmds__mft.dir/mft.cpp.obj"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/systemcmds/mft && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/systemcmds__mft.dir/mft.cpp.obj -c /home/andrey/Documents/ap/PX4-Autopilot/src/systemcmds/mft/mft.cpp

src/systemcmds/mft/CMakeFiles/systemcmds__mft.dir/mft.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/systemcmds__mft.dir/mft.cpp.i"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/systemcmds/mft && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/andrey/Documents/ap/PX4-Autopilot/src/systemcmds/mft/mft.cpp > CMakeFiles/systemcmds__mft.dir/mft.cpp.i

src/systemcmds/mft/CMakeFiles/systemcmds__mft.dir/mft.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/systemcmds__mft.dir/mft.cpp.s"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/systemcmds/mft && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/andrey/Documents/ap/PX4-Autopilot/src/systemcmds/mft/mft.cpp -o CMakeFiles/systemcmds__mft.dir/mft.cpp.s

# Object files for target systemcmds__mft
systemcmds__mft_OBJECTS = \
"CMakeFiles/systemcmds__mft.dir/mft.cpp.obj"

# External object files for target systemcmds__mft
systemcmds__mft_EXTERNAL_OBJECTS =

src/systemcmds/mft/libsystemcmds__mft.a: src/systemcmds/mft/CMakeFiles/systemcmds__mft.dir/mft.cpp.obj
src/systemcmds/mft/libsystemcmds__mft.a: src/systemcmds/mft/CMakeFiles/systemcmds__mft.dir/build.make
src/systemcmds/mft/libsystemcmds__mft.a: src/systemcmds/mft/CMakeFiles/systemcmds__mft.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libsystemcmds__mft.a"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/systemcmds/mft && $(CMAKE_COMMAND) -P CMakeFiles/systemcmds__mft.dir/cmake_clean_target.cmake
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/systemcmds/mft && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/systemcmds__mft.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/systemcmds/mft/CMakeFiles/systemcmds__mft.dir/build: src/systemcmds/mft/libsystemcmds__mft.a

.PHONY : src/systemcmds/mft/CMakeFiles/systemcmds__mft.dir/build

src/systemcmds/mft/CMakeFiles/systemcmds__mft.dir/clean:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/systemcmds/mft && $(CMAKE_COMMAND) -P CMakeFiles/systemcmds__mft.dir/cmake_clean.cmake
.PHONY : src/systemcmds/mft/CMakeFiles/systemcmds__mft.dir/clean

src/systemcmds/mft/CMakeFiles/systemcmds__mft.dir/depend:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/andrey/Documents/ap/PX4-Autopilot /home/andrey/Documents/ap/PX4-Autopilot/src/systemcmds/mft /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/systemcmds/mft /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/systemcmds/mft/CMakeFiles/systemcmds__mft.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/systemcmds/mft/CMakeFiles/systemcmds__mft.dir/depend

