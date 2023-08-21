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
include src/lib/bezier/CMakeFiles/bezier.dir/depend.make

# Include the progress variables for this target.
include src/lib/bezier/CMakeFiles/bezier.dir/progress.make

# Include the compile flags for this target's objects.
include src/lib/bezier/CMakeFiles/bezier.dir/flags.make

src/lib/bezier/CMakeFiles/bezier.dir/BezierQuad.cpp.obj: src/lib/bezier/CMakeFiles/bezier.dir/flags.make
src/lib/bezier/CMakeFiles/bezier.dir/BezierQuad.cpp.obj: ../../src/lib/bezier/BezierQuad.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/lib/bezier/CMakeFiles/bezier.dir/BezierQuad.cpp.obj"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/lib/bezier && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/bezier.dir/BezierQuad.cpp.obj -c /home/andrey/Documents/ap/PX4-Autopilot/src/lib/bezier/BezierQuad.cpp

src/lib/bezier/CMakeFiles/bezier.dir/BezierQuad.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/bezier.dir/BezierQuad.cpp.i"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/lib/bezier && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/andrey/Documents/ap/PX4-Autopilot/src/lib/bezier/BezierQuad.cpp > CMakeFiles/bezier.dir/BezierQuad.cpp.i

src/lib/bezier/CMakeFiles/bezier.dir/BezierQuad.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/bezier.dir/BezierQuad.cpp.s"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/lib/bezier && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/andrey/Documents/ap/PX4-Autopilot/src/lib/bezier/BezierQuad.cpp -o CMakeFiles/bezier.dir/BezierQuad.cpp.s

src/lib/bezier/CMakeFiles/bezier.dir/BezierN.cpp.obj: src/lib/bezier/CMakeFiles/bezier.dir/flags.make
src/lib/bezier/CMakeFiles/bezier.dir/BezierN.cpp.obj: ../../src/lib/bezier/BezierN.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/lib/bezier/CMakeFiles/bezier.dir/BezierN.cpp.obj"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/lib/bezier && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/bezier.dir/BezierN.cpp.obj -c /home/andrey/Documents/ap/PX4-Autopilot/src/lib/bezier/BezierN.cpp

src/lib/bezier/CMakeFiles/bezier.dir/BezierN.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/bezier.dir/BezierN.cpp.i"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/lib/bezier && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/andrey/Documents/ap/PX4-Autopilot/src/lib/bezier/BezierN.cpp > CMakeFiles/bezier.dir/BezierN.cpp.i

src/lib/bezier/CMakeFiles/bezier.dir/BezierN.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/bezier.dir/BezierN.cpp.s"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/lib/bezier && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/andrey/Documents/ap/PX4-Autopilot/src/lib/bezier/BezierN.cpp -o CMakeFiles/bezier.dir/BezierN.cpp.s

# Object files for target bezier
bezier_OBJECTS = \
"CMakeFiles/bezier.dir/BezierQuad.cpp.obj" \
"CMakeFiles/bezier.dir/BezierN.cpp.obj"

# External object files for target bezier
bezier_EXTERNAL_OBJECTS =

src/lib/bezier/libbezier.a: src/lib/bezier/CMakeFiles/bezier.dir/BezierQuad.cpp.obj
src/lib/bezier/libbezier.a: src/lib/bezier/CMakeFiles/bezier.dir/BezierN.cpp.obj
src/lib/bezier/libbezier.a: src/lib/bezier/CMakeFiles/bezier.dir/build.make
src/lib/bezier/libbezier.a: src/lib/bezier/CMakeFiles/bezier.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX static library libbezier.a"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/lib/bezier && $(CMAKE_COMMAND) -P CMakeFiles/bezier.dir/cmake_clean_target.cmake
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/lib/bezier && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/bezier.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/lib/bezier/CMakeFiles/bezier.dir/build: src/lib/bezier/libbezier.a

.PHONY : src/lib/bezier/CMakeFiles/bezier.dir/build

src/lib/bezier/CMakeFiles/bezier.dir/clean:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/lib/bezier && $(CMAKE_COMMAND) -P CMakeFiles/bezier.dir/cmake_clean.cmake
.PHONY : src/lib/bezier/CMakeFiles/bezier.dir/clean

src/lib/bezier/CMakeFiles/bezier.dir/depend:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/andrey/Documents/ap/PX4-Autopilot /home/andrey/Documents/ap/PX4-Autopilot/src/lib/bezier /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/lib/bezier /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/lib/bezier/CMakeFiles/bezier.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/lib/bezier/CMakeFiles/bezier.dir/depend

