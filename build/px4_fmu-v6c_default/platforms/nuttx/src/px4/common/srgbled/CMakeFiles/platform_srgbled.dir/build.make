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
include platforms/nuttx/src/px4/common/srgbled/CMakeFiles/platform_srgbled.dir/depend.make

# Include the progress variables for this target.
include platforms/nuttx/src/px4/common/srgbled/CMakeFiles/platform_srgbled.dir/progress.make

# Include the compile flags for this target's objects.
include platforms/nuttx/src/px4/common/srgbled/CMakeFiles/platform_srgbled.dir/flags.make

platforms/nuttx/src/px4/common/srgbled/CMakeFiles/platform_srgbled.dir/srgbled.cpp.obj: platforms/nuttx/src/px4/common/srgbled/CMakeFiles/platform_srgbled.dir/flags.make
platforms/nuttx/src/px4/common/srgbled/CMakeFiles/platform_srgbled.dir/srgbled.cpp.obj: ../../platforms/nuttx/src/px4/common/srgbled/srgbled.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object platforms/nuttx/src/px4/common/srgbled/CMakeFiles/platform_srgbled.dir/srgbled.cpp.obj"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/platforms/nuttx/src/px4/common/srgbled && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/platform_srgbled.dir/srgbled.cpp.obj -c /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/src/px4/common/srgbled/srgbled.cpp

platforms/nuttx/src/px4/common/srgbled/CMakeFiles/platform_srgbled.dir/srgbled.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/platform_srgbled.dir/srgbled.cpp.i"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/platforms/nuttx/src/px4/common/srgbled && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/src/px4/common/srgbled/srgbled.cpp > CMakeFiles/platform_srgbled.dir/srgbled.cpp.i

platforms/nuttx/src/px4/common/srgbled/CMakeFiles/platform_srgbled.dir/srgbled.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/platform_srgbled.dir/srgbled.cpp.s"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/platforms/nuttx/src/px4/common/srgbled && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/src/px4/common/srgbled/srgbled.cpp -o CMakeFiles/platform_srgbled.dir/srgbled.cpp.s

# Object files for target platform_srgbled
platform_srgbled_OBJECTS = \
"CMakeFiles/platform_srgbled.dir/srgbled.cpp.obj"

# External object files for target platform_srgbled
platform_srgbled_EXTERNAL_OBJECTS =

platforms/nuttx/src/px4/common/srgbled/libplatform_srgbled.a: platforms/nuttx/src/px4/common/srgbled/CMakeFiles/platform_srgbled.dir/srgbled.cpp.obj
platforms/nuttx/src/px4/common/srgbled/libplatform_srgbled.a: platforms/nuttx/src/px4/common/srgbled/CMakeFiles/platform_srgbled.dir/build.make
platforms/nuttx/src/px4/common/srgbled/libplatform_srgbled.a: platforms/nuttx/src/px4/common/srgbled/CMakeFiles/platform_srgbled.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libplatform_srgbled.a"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/platforms/nuttx/src/px4/common/srgbled && $(CMAKE_COMMAND) -P CMakeFiles/platform_srgbled.dir/cmake_clean_target.cmake
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/platforms/nuttx/src/px4/common/srgbled && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/platform_srgbled.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
platforms/nuttx/src/px4/common/srgbled/CMakeFiles/platform_srgbled.dir/build: platforms/nuttx/src/px4/common/srgbled/libplatform_srgbled.a

.PHONY : platforms/nuttx/src/px4/common/srgbled/CMakeFiles/platform_srgbled.dir/build

platforms/nuttx/src/px4/common/srgbled/CMakeFiles/platform_srgbled.dir/clean:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/platforms/nuttx/src/px4/common/srgbled && $(CMAKE_COMMAND) -P CMakeFiles/platform_srgbled.dir/cmake_clean.cmake
.PHONY : platforms/nuttx/src/px4/common/srgbled/CMakeFiles/platform_srgbled.dir/clean

platforms/nuttx/src/px4/common/srgbled/CMakeFiles/platform_srgbled.dir/depend:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/andrey/Documents/ap/PX4-Autopilot /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/src/px4/common/srgbled /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/platforms/nuttx/src/px4/common/srgbled /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/platforms/nuttx/src/px4/common/srgbled/CMakeFiles/platform_srgbled.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : platforms/nuttx/src/px4/common/srgbled/CMakeFiles/platform_srgbled.dir/depend

