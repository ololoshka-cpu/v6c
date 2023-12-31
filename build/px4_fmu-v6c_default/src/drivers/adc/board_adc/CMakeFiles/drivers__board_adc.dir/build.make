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
include src/drivers/adc/board_adc/CMakeFiles/drivers__board_adc.dir/depend.make

# Include the progress variables for this target.
include src/drivers/adc/board_adc/CMakeFiles/drivers__board_adc.dir/progress.make

# Include the compile flags for this target's objects.
include src/drivers/adc/board_adc/CMakeFiles/drivers__board_adc.dir/flags.make

src/drivers/adc/board_adc/CMakeFiles/drivers__board_adc.dir/ADC.cpp.obj: src/drivers/adc/board_adc/CMakeFiles/drivers__board_adc.dir/flags.make
src/drivers/adc/board_adc/CMakeFiles/drivers__board_adc.dir/ADC.cpp.obj: ../../src/drivers/adc/board_adc/ADC.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/drivers/adc/board_adc/CMakeFiles/drivers__board_adc.dir/ADC.cpp.obj"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/adc/board_adc && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/drivers__board_adc.dir/ADC.cpp.obj -c /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/adc/board_adc/ADC.cpp

src/drivers/adc/board_adc/CMakeFiles/drivers__board_adc.dir/ADC.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drivers__board_adc.dir/ADC.cpp.i"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/adc/board_adc && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/adc/board_adc/ADC.cpp > CMakeFiles/drivers__board_adc.dir/ADC.cpp.i

src/drivers/adc/board_adc/CMakeFiles/drivers__board_adc.dir/ADC.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drivers__board_adc.dir/ADC.cpp.s"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/adc/board_adc && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/adc/board_adc/ADC.cpp -o CMakeFiles/drivers__board_adc.dir/ADC.cpp.s

# Object files for target drivers__board_adc
drivers__board_adc_OBJECTS = \
"CMakeFiles/drivers__board_adc.dir/ADC.cpp.obj"

# External object files for target drivers__board_adc
drivers__board_adc_EXTERNAL_OBJECTS =

src/drivers/adc/board_adc/libdrivers__board_adc.a: src/drivers/adc/board_adc/CMakeFiles/drivers__board_adc.dir/ADC.cpp.obj
src/drivers/adc/board_adc/libdrivers__board_adc.a: src/drivers/adc/board_adc/CMakeFiles/drivers__board_adc.dir/build.make
src/drivers/adc/board_adc/libdrivers__board_adc.a: src/drivers/adc/board_adc/CMakeFiles/drivers__board_adc.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libdrivers__board_adc.a"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/adc/board_adc && $(CMAKE_COMMAND) -P CMakeFiles/drivers__board_adc.dir/cmake_clean_target.cmake
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/adc/board_adc && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/drivers__board_adc.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/drivers/adc/board_adc/CMakeFiles/drivers__board_adc.dir/build: src/drivers/adc/board_adc/libdrivers__board_adc.a

.PHONY : src/drivers/adc/board_adc/CMakeFiles/drivers__board_adc.dir/build

src/drivers/adc/board_adc/CMakeFiles/drivers__board_adc.dir/clean:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/adc/board_adc && $(CMAKE_COMMAND) -P CMakeFiles/drivers__board_adc.dir/cmake_clean.cmake
.PHONY : src/drivers/adc/board_adc/CMakeFiles/drivers__board_adc.dir/clean

src/drivers/adc/board_adc/CMakeFiles/drivers__board_adc.dir/depend:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/andrey/Documents/ap/PX4-Autopilot /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/adc/board_adc /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/adc/board_adc /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/adc/board_adc/CMakeFiles/drivers__board_adc.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/drivers/adc/board_adc/CMakeFiles/drivers__board_adc.dir/depend

