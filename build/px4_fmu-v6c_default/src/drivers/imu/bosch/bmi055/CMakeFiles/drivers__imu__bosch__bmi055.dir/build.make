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
include src/drivers/imu/bosch/bmi055/CMakeFiles/drivers__imu__bosch__bmi055.dir/depend.make

# Include the progress variables for this target.
include src/drivers/imu/bosch/bmi055/CMakeFiles/drivers__imu__bosch__bmi055.dir/progress.make

# Include the compile flags for this target's objects.
include src/drivers/imu/bosch/bmi055/CMakeFiles/drivers__imu__bosch__bmi055.dir/flags.make

src/drivers/imu/bosch/bmi055/CMakeFiles/drivers__imu__bosch__bmi055.dir/BMI055.cpp.obj: src/drivers/imu/bosch/bmi055/CMakeFiles/drivers__imu__bosch__bmi055.dir/flags.make
src/drivers/imu/bosch/bmi055/CMakeFiles/drivers__imu__bosch__bmi055.dir/BMI055.cpp.obj: ../../src/drivers/imu/bosch/bmi055/BMI055.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/drivers/imu/bosch/bmi055/CMakeFiles/drivers__imu__bosch__bmi055.dir/BMI055.cpp.obj"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/imu/bosch/bmi055 && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/drivers__imu__bosch__bmi055.dir/BMI055.cpp.obj -c /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/imu/bosch/bmi055/BMI055.cpp

src/drivers/imu/bosch/bmi055/CMakeFiles/drivers__imu__bosch__bmi055.dir/BMI055.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drivers__imu__bosch__bmi055.dir/BMI055.cpp.i"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/imu/bosch/bmi055 && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/imu/bosch/bmi055/BMI055.cpp > CMakeFiles/drivers__imu__bosch__bmi055.dir/BMI055.cpp.i

src/drivers/imu/bosch/bmi055/CMakeFiles/drivers__imu__bosch__bmi055.dir/BMI055.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drivers__imu__bosch__bmi055.dir/BMI055.cpp.s"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/imu/bosch/bmi055 && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/imu/bosch/bmi055/BMI055.cpp -o CMakeFiles/drivers__imu__bosch__bmi055.dir/BMI055.cpp.s

src/drivers/imu/bosch/bmi055/CMakeFiles/drivers__imu__bosch__bmi055.dir/BMI055_Accelerometer.cpp.obj: src/drivers/imu/bosch/bmi055/CMakeFiles/drivers__imu__bosch__bmi055.dir/flags.make
src/drivers/imu/bosch/bmi055/CMakeFiles/drivers__imu__bosch__bmi055.dir/BMI055_Accelerometer.cpp.obj: ../../src/drivers/imu/bosch/bmi055/BMI055_Accelerometer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/drivers/imu/bosch/bmi055/CMakeFiles/drivers__imu__bosch__bmi055.dir/BMI055_Accelerometer.cpp.obj"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/imu/bosch/bmi055 && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/drivers__imu__bosch__bmi055.dir/BMI055_Accelerometer.cpp.obj -c /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/imu/bosch/bmi055/BMI055_Accelerometer.cpp

src/drivers/imu/bosch/bmi055/CMakeFiles/drivers__imu__bosch__bmi055.dir/BMI055_Accelerometer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drivers__imu__bosch__bmi055.dir/BMI055_Accelerometer.cpp.i"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/imu/bosch/bmi055 && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/imu/bosch/bmi055/BMI055_Accelerometer.cpp > CMakeFiles/drivers__imu__bosch__bmi055.dir/BMI055_Accelerometer.cpp.i

src/drivers/imu/bosch/bmi055/CMakeFiles/drivers__imu__bosch__bmi055.dir/BMI055_Accelerometer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drivers__imu__bosch__bmi055.dir/BMI055_Accelerometer.cpp.s"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/imu/bosch/bmi055 && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/imu/bosch/bmi055/BMI055_Accelerometer.cpp -o CMakeFiles/drivers__imu__bosch__bmi055.dir/BMI055_Accelerometer.cpp.s

src/drivers/imu/bosch/bmi055/CMakeFiles/drivers__imu__bosch__bmi055.dir/BMI055_Gyroscope.cpp.obj: src/drivers/imu/bosch/bmi055/CMakeFiles/drivers__imu__bosch__bmi055.dir/flags.make
src/drivers/imu/bosch/bmi055/CMakeFiles/drivers__imu__bosch__bmi055.dir/BMI055_Gyroscope.cpp.obj: ../../src/drivers/imu/bosch/bmi055/BMI055_Gyroscope.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object src/drivers/imu/bosch/bmi055/CMakeFiles/drivers__imu__bosch__bmi055.dir/BMI055_Gyroscope.cpp.obj"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/imu/bosch/bmi055 && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/drivers__imu__bosch__bmi055.dir/BMI055_Gyroscope.cpp.obj -c /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/imu/bosch/bmi055/BMI055_Gyroscope.cpp

src/drivers/imu/bosch/bmi055/CMakeFiles/drivers__imu__bosch__bmi055.dir/BMI055_Gyroscope.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drivers__imu__bosch__bmi055.dir/BMI055_Gyroscope.cpp.i"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/imu/bosch/bmi055 && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/imu/bosch/bmi055/BMI055_Gyroscope.cpp > CMakeFiles/drivers__imu__bosch__bmi055.dir/BMI055_Gyroscope.cpp.i

src/drivers/imu/bosch/bmi055/CMakeFiles/drivers__imu__bosch__bmi055.dir/BMI055_Gyroscope.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drivers__imu__bosch__bmi055.dir/BMI055_Gyroscope.cpp.s"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/imu/bosch/bmi055 && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/imu/bosch/bmi055/BMI055_Gyroscope.cpp -o CMakeFiles/drivers__imu__bosch__bmi055.dir/BMI055_Gyroscope.cpp.s

src/drivers/imu/bosch/bmi055/CMakeFiles/drivers__imu__bosch__bmi055.dir/bmi055_main.cpp.obj: src/drivers/imu/bosch/bmi055/CMakeFiles/drivers__imu__bosch__bmi055.dir/flags.make
src/drivers/imu/bosch/bmi055/CMakeFiles/drivers__imu__bosch__bmi055.dir/bmi055_main.cpp.obj: ../../src/drivers/imu/bosch/bmi055/bmi055_main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object src/drivers/imu/bosch/bmi055/CMakeFiles/drivers__imu__bosch__bmi055.dir/bmi055_main.cpp.obj"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/imu/bosch/bmi055 && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/drivers__imu__bosch__bmi055.dir/bmi055_main.cpp.obj -c /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/imu/bosch/bmi055/bmi055_main.cpp

src/drivers/imu/bosch/bmi055/CMakeFiles/drivers__imu__bosch__bmi055.dir/bmi055_main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drivers__imu__bosch__bmi055.dir/bmi055_main.cpp.i"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/imu/bosch/bmi055 && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/imu/bosch/bmi055/bmi055_main.cpp > CMakeFiles/drivers__imu__bosch__bmi055.dir/bmi055_main.cpp.i

src/drivers/imu/bosch/bmi055/CMakeFiles/drivers__imu__bosch__bmi055.dir/bmi055_main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drivers__imu__bosch__bmi055.dir/bmi055_main.cpp.s"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/imu/bosch/bmi055 && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/imu/bosch/bmi055/bmi055_main.cpp -o CMakeFiles/drivers__imu__bosch__bmi055.dir/bmi055_main.cpp.s

# Object files for target drivers__imu__bosch__bmi055
drivers__imu__bosch__bmi055_OBJECTS = \
"CMakeFiles/drivers__imu__bosch__bmi055.dir/BMI055.cpp.obj" \
"CMakeFiles/drivers__imu__bosch__bmi055.dir/BMI055_Accelerometer.cpp.obj" \
"CMakeFiles/drivers__imu__bosch__bmi055.dir/BMI055_Gyroscope.cpp.obj" \
"CMakeFiles/drivers__imu__bosch__bmi055.dir/bmi055_main.cpp.obj"

# External object files for target drivers__imu__bosch__bmi055
drivers__imu__bosch__bmi055_EXTERNAL_OBJECTS =

src/drivers/imu/bosch/bmi055/libdrivers__imu__bosch__bmi055.a: src/drivers/imu/bosch/bmi055/CMakeFiles/drivers__imu__bosch__bmi055.dir/BMI055.cpp.obj
src/drivers/imu/bosch/bmi055/libdrivers__imu__bosch__bmi055.a: src/drivers/imu/bosch/bmi055/CMakeFiles/drivers__imu__bosch__bmi055.dir/BMI055_Accelerometer.cpp.obj
src/drivers/imu/bosch/bmi055/libdrivers__imu__bosch__bmi055.a: src/drivers/imu/bosch/bmi055/CMakeFiles/drivers__imu__bosch__bmi055.dir/BMI055_Gyroscope.cpp.obj
src/drivers/imu/bosch/bmi055/libdrivers__imu__bosch__bmi055.a: src/drivers/imu/bosch/bmi055/CMakeFiles/drivers__imu__bosch__bmi055.dir/bmi055_main.cpp.obj
src/drivers/imu/bosch/bmi055/libdrivers__imu__bosch__bmi055.a: src/drivers/imu/bosch/bmi055/CMakeFiles/drivers__imu__bosch__bmi055.dir/build.make
src/drivers/imu/bosch/bmi055/libdrivers__imu__bosch__bmi055.a: src/drivers/imu/bosch/bmi055/CMakeFiles/drivers__imu__bosch__bmi055.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX static library libdrivers__imu__bosch__bmi055.a"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/imu/bosch/bmi055 && $(CMAKE_COMMAND) -P CMakeFiles/drivers__imu__bosch__bmi055.dir/cmake_clean_target.cmake
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/imu/bosch/bmi055 && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/drivers__imu__bosch__bmi055.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/drivers/imu/bosch/bmi055/CMakeFiles/drivers__imu__bosch__bmi055.dir/build: src/drivers/imu/bosch/bmi055/libdrivers__imu__bosch__bmi055.a

.PHONY : src/drivers/imu/bosch/bmi055/CMakeFiles/drivers__imu__bosch__bmi055.dir/build

src/drivers/imu/bosch/bmi055/CMakeFiles/drivers__imu__bosch__bmi055.dir/clean:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/imu/bosch/bmi055 && $(CMAKE_COMMAND) -P CMakeFiles/drivers__imu__bosch__bmi055.dir/cmake_clean.cmake
.PHONY : src/drivers/imu/bosch/bmi055/CMakeFiles/drivers__imu__bosch__bmi055.dir/clean

src/drivers/imu/bosch/bmi055/CMakeFiles/drivers__imu__bosch__bmi055.dir/depend:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/andrey/Documents/ap/PX4-Autopilot /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/imu/bosch/bmi055 /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/imu/bosch/bmi055 /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/drivers/imu/bosch/bmi055/CMakeFiles/drivers__imu__bosch__bmi055.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/drivers/imu/bosch/bmi055/CMakeFiles/drivers__imu__bosch__bmi055.dir/depend

