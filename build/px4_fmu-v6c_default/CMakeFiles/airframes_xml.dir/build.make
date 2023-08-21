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

# Utility rule file for airframes_xml.

# Include the progress variables for this target.
include CMakeFiles/airframes_xml.dir/progress.make

CMakeFiles/airframes_xml: airframes.xml


airframes.xml: ../../Tools/px_process_airframes.py
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Creating airframes.xml"
	/usr/bin/python3 /home/andrey/Documents/ap/PX4-Autopilot/Tools/px_process_airframes.py --airframes-path /home/andrey/Documents/ap/PX4-Autopilot/ROMFS/px4fmu_common/init.d --board CONFIG_ARCH_BOARD_px4_fmu-v6c --xml /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/airframes.xml

airframes_xml: CMakeFiles/airframes_xml
airframes_xml: airframes.xml
airframes_xml: CMakeFiles/airframes_xml.dir/build.make

.PHONY : airframes_xml

# Rule to build all files generated by this target.
CMakeFiles/airframes_xml.dir/build: airframes_xml

.PHONY : CMakeFiles/airframes_xml.dir/build

CMakeFiles/airframes_xml.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/airframes_xml.dir/cmake_clean.cmake
.PHONY : CMakeFiles/airframes_xml.dir/clean

CMakeFiles/airframes_xml.dir/depend:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/andrey/Documents/ap/PX4-Autopilot /home/andrey/Documents/ap/PX4-Autopilot /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles/airframes_xml.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/airframes_xml.dir/depend

