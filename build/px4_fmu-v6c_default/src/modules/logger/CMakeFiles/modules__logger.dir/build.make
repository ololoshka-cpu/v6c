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
include src/modules/logger/CMakeFiles/modules__logger.dir/depend.make

# Include the progress variables for this target.
include src/modules/logger/CMakeFiles/modules__logger.dir/progress.make

# Include the compile flags for this target's objects.
include src/modules/logger/CMakeFiles/modules__logger.dir/flags.make

src/modules/logger/CMakeFiles/modules__logger.dir/logged_topics.cpp.obj: src/modules/logger/CMakeFiles/modules__logger.dir/flags.make
src/modules/logger/CMakeFiles/modules__logger.dir/logged_topics.cpp.obj: ../../src/modules/logger/logged_topics.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/modules/logger/CMakeFiles/modules__logger.dir/logged_topics.cpp.obj"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/logger && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/modules__logger.dir/logged_topics.cpp.obj -c /home/andrey/Documents/ap/PX4-Autopilot/src/modules/logger/logged_topics.cpp

src/modules/logger/CMakeFiles/modules__logger.dir/logged_topics.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/modules__logger.dir/logged_topics.cpp.i"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/logger && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/andrey/Documents/ap/PX4-Autopilot/src/modules/logger/logged_topics.cpp > CMakeFiles/modules__logger.dir/logged_topics.cpp.i

src/modules/logger/CMakeFiles/modules__logger.dir/logged_topics.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/modules__logger.dir/logged_topics.cpp.s"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/logger && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/andrey/Documents/ap/PX4-Autopilot/src/modules/logger/logged_topics.cpp -o CMakeFiles/modules__logger.dir/logged_topics.cpp.s

src/modules/logger/CMakeFiles/modules__logger.dir/logger.cpp.obj: src/modules/logger/CMakeFiles/modules__logger.dir/flags.make
src/modules/logger/CMakeFiles/modules__logger.dir/logger.cpp.obj: ../../src/modules/logger/logger.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/modules/logger/CMakeFiles/modules__logger.dir/logger.cpp.obj"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/logger && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/modules__logger.dir/logger.cpp.obj -c /home/andrey/Documents/ap/PX4-Autopilot/src/modules/logger/logger.cpp

src/modules/logger/CMakeFiles/modules__logger.dir/logger.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/modules__logger.dir/logger.cpp.i"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/logger && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/andrey/Documents/ap/PX4-Autopilot/src/modules/logger/logger.cpp > CMakeFiles/modules__logger.dir/logger.cpp.i

src/modules/logger/CMakeFiles/modules__logger.dir/logger.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/modules__logger.dir/logger.cpp.s"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/logger && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/andrey/Documents/ap/PX4-Autopilot/src/modules/logger/logger.cpp -o CMakeFiles/modules__logger.dir/logger.cpp.s

src/modules/logger/CMakeFiles/modules__logger.dir/log_writer.cpp.obj: src/modules/logger/CMakeFiles/modules__logger.dir/flags.make
src/modules/logger/CMakeFiles/modules__logger.dir/log_writer.cpp.obj: ../../src/modules/logger/log_writer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object src/modules/logger/CMakeFiles/modules__logger.dir/log_writer.cpp.obj"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/logger && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/modules__logger.dir/log_writer.cpp.obj -c /home/andrey/Documents/ap/PX4-Autopilot/src/modules/logger/log_writer.cpp

src/modules/logger/CMakeFiles/modules__logger.dir/log_writer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/modules__logger.dir/log_writer.cpp.i"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/logger && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/andrey/Documents/ap/PX4-Autopilot/src/modules/logger/log_writer.cpp > CMakeFiles/modules__logger.dir/log_writer.cpp.i

src/modules/logger/CMakeFiles/modules__logger.dir/log_writer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/modules__logger.dir/log_writer.cpp.s"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/logger && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/andrey/Documents/ap/PX4-Autopilot/src/modules/logger/log_writer.cpp -o CMakeFiles/modules__logger.dir/log_writer.cpp.s

src/modules/logger/CMakeFiles/modules__logger.dir/log_writer_file.cpp.obj: src/modules/logger/CMakeFiles/modules__logger.dir/flags.make
src/modules/logger/CMakeFiles/modules__logger.dir/log_writer_file.cpp.obj: ../../src/modules/logger/log_writer_file.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object src/modules/logger/CMakeFiles/modules__logger.dir/log_writer_file.cpp.obj"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/logger && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/modules__logger.dir/log_writer_file.cpp.obj -c /home/andrey/Documents/ap/PX4-Autopilot/src/modules/logger/log_writer_file.cpp

src/modules/logger/CMakeFiles/modules__logger.dir/log_writer_file.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/modules__logger.dir/log_writer_file.cpp.i"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/logger && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/andrey/Documents/ap/PX4-Autopilot/src/modules/logger/log_writer_file.cpp > CMakeFiles/modules__logger.dir/log_writer_file.cpp.i

src/modules/logger/CMakeFiles/modules__logger.dir/log_writer_file.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/modules__logger.dir/log_writer_file.cpp.s"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/logger && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/andrey/Documents/ap/PX4-Autopilot/src/modules/logger/log_writer_file.cpp -o CMakeFiles/modules__logger.dir/log_writer_file.cpp.s

src/modules/logger/CMakeFiles/modules__logger.dir/log_writer_mavlink.cpp.obj: src/modules/logger/CMakeFiles/modules__logger.dir/flags.make
src/modules/logger/CMakeFiles/modules__logger.dir/log_writer_mavlink.cpp.obj: ../../src/modules/logger/log_writer_mavlink.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object src/modules/logger/CMakeFiles/modules__logger.dir/log_writer_mavlink.cpp.obj"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/logger && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/modules__logger.dir/log_writer_mavlink.cpp.obj -c /home/andrey/Documents/ap/PX4-Autopilot/src/modules/logger/log_writer_mavlink.cpp

src/modules/logger/CMakeFiles/modules__logger.dir/log_writer_mavlink.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/modules__logger.dir/log_writer_mavlink.cpp.i"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/logger && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/andrey/Documents/ap/PX4-Autopilot/src/modules/logger/log_writer_mavlink.cpp > CMakeFiles/modules__logger.dir/log_writer_mavlink.cpp.i

src/modules/logger/CMakeFiles/modules__logger.dir/log_writer_mavlink.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/modules__logger.dir/log_writer_mavlink.cpp.s"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/logger && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/andrey/Documents/ap/PX4-Autopilot/src/modules/logger/log_writer_mavlink.cpp -o CMakeFiles/modules__logger.dir/log_writer_mavlink.cpp.s

src/modules/logger/CMakeFiles/modules__logger.dir/util.cpp.obj: src/modules/logger/CMakeFiles/modules__logger.dir/flags.make
src/modules/logger/CMakeFiles/modules__logger.dir/util.cpp.obj: ../../src/modules/logger/util.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object src/modules/logger/CMakeFiles/modules__logger.dir/util.cpp.obj"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/logger && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/modules__logger.dir/util.cpp.obj -c /home/andrey/Documents/ap/PX4-Autopilot/src/modules/logger/util.cpp

src/modules/logger/CMakeFiles/modules__logger.dir/util.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/modules__logger.dir/util.cpp.i"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/logger && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/andrey/Documents/ap/PX4-Autopilot/src/modules/logger/util.cpp > CMakeFiles/modules__logger.dir/util.cpp.i

src/modules/logger/CMakeFiles/modules__logger.dir/util.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/modules__logger.dir/util.cpp.s"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/logger && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/andrey/Documents/ap/PX4-Autopilot/src/modules/logger/util.cpp -o CMakeFiles/modules__logger.dir/util.cpp.s

src/modules/logger/CMakeFiles/modules__logger.dir/watchdog.cpp.obj: src/modules/logger/CMakeFiles/modules__logger.dir/flags.make
src/modules/logger/CMakeFiles/modules__logger.dir/watchdog.cpp.obj: ../../src/modules/logger/watchdog.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object src/modules/logger/CMakeFiles/modules__logger.dir/watchdog.cpp.obj"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/logger && /usr/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/modules__logger.dir/watchdog.cpp.obj -c /home/andrey/Documents/ap/PX4-Autopilot/src/modules/logger/watchdog.cpp

src/modules/logger/CMakeFiles/modules__logger.dir/watchdog.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/modules__logger.dir/watchdog.cpp.i"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/logger && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/andrey/Documents/ap/PX4-Autopilot/src/modules/logger/watchdog.cpp > CMakeFiles/modules__logger.dir/watchdog.cpp.i

src/modules/logger/CMakeFiles/modules__logger.dir/watchdog.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/modules__logger.dir/watchdog.cpp.s"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/logger && /usr/bin/arm-none-eabi-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/andrey/Documents/ap/PX4-Autopilot/src/modules/logger/watchdog.cpp -o CMakeFiles/modules__logger.dir/watchdog.cpp.s

# Object files for target modules__logger
modules__logger_OBJECTS = \
"CMakeFiles/modules__logger.dir/logged_topics.cpp.obj" \
"CMakeFiles/modules__logger.dir/logger.cpp.obj" \
"CMakeFiles/modules__logger.dir/log_writer.cpp.obj" \
"CMakeFiles/modules__logger.dir/log_writer_file.cpp.obj" \
"CMakeFiles/modules__logger.dir/log_writer_mavlink.cpp.obj" \
"CMakeFiles/modules__logger.dir/util.cpp.obj" \
"CMakeFiles/modules__logger.dir/watchdog.cpp.obj"

# External object files for target modules__logger
modules__logger_EXTERNAL_OBJECTS =

src/modules/logger/libmodules__logger.a: src/modules/logger/CMakeFiles/modules__logger.dir/logged_topics.cpp.obj
src/modules/logger/libmodules__logger.a: src/modules/logger/CMakeFiles/modules__logger.dir/logger.cpp.obj
src/modules/logger/libmodules__logger.a: src/modules/logger/CMakeFiles/modules__logger.dir/log_writer.cpp.obj
src/modules/logger/libmodules__logger.a: src/modules/logger/CMakeFiles/modules__logger.dir/log_writer_file.cpp.obj
src/modules/logger/libmodules__logger.a: src/modules/logger/CMakeFiles/modules__logger.dir/log_writer_mavlink.cpp.obj
src/modules/logger/libmodules__logger.a: src/modules/logger/CMakeFiles/modules__logger.dir/util.cpp.obj
src/modules/logger/libmodules__logger.a: src/modules/logger/CMakeFiles/modules__logger.dir/watchdog.cpp.obj
src/modules/logger/libmodules__logger.a: src/modules/logger/CMakeFiles/modules__logger.dir/build.make
src/modules/logger/libmodules__logger.a: src/modules/logger/CMakeFiles/modules__logger.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Linking CXX static library libmodules__logger.a"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/logger && $(CMAKE_COMMAND) -P CMakeFiles/modules__logger.dir/cmake_clean_target.cmake
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/logger && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/modules__logger.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/modules/logger/CMakeFiles/modules__logger.dir/build: src/modules/logger/libmodules__logger.a

.PHONY : src/modules/logger/CMakeFiles/modules__logger.dir/build

src/modules/logger/CMakeFiles/modules__logger.dir/clean:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/logger && $(CMAKE_COMMAND) -P CMakeFiles/modules__logger.dir/cmake_clean.cmake
.PHONY : src/modules/logger/CMakeFiles/modules__logger.dir/clean

src/modules/logger/CMakeFiles/modules__logger.dir/depend:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/andrey/Documents/ap/PX4-Autopilot /home/andrey/Documents/ap/PX4-Autopilot/src/modules/logger /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/logger /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/logger/CMakeFiles/modules__logger.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/modules/logger/CMakeFiles/modules__logger.dir/depend

