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

# Utility rule file for component_general_json.

# Include the progress variables for this target.
include src/lib/component_information/CMakeFiles/component_general_json.dir/progress.make

src/lib/component_information/CMakeFiles/component_general_json: component_general.json


component_general.json: ../../src/lib/component_information/generate_component_general.py
component_general.json: parameters.json.xz
component_general.json: events/all_events.json.xz
component_general.json: actuators.json.xz
component_general.json: ../../src/lib/component_information/generate_crc.py
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating component_general.json and checksums.h"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/lib/component_information && /usr/bin/python3 /home/andrey/Documents/ap/PX4-Autopilot/src/lib/component_information/generate_component_general.py /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/component_general.json --compress --type 1,/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/parameters.json.xz,mftp://etc/extras/parameters.json.xz,https://px4-travis.s3.amazonaws.com/Firmware/{version}/px4_fmu-v6c_default/parameters.json.xz,https://raw.githubusercontent.com/PX4/PX4-Metadata-Translations/main/translated/parameters_summary.json --type 4,/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/events/all_events.json.xz,mftp://etc/extras/all_events.json.xz,https://px4-travis.s3.amazonaws.com/Firmware/{version}/px4_fmu-v6c_default/all_events.json.xz,https://raw.githubusercontent.com/PX4/PX4-Metadata-Translations/main/translated/events_summary.json --type 5,/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/actuators.json.xz,mftp://etc/extras/actuators.json.xz,https://px4-travis.s3.amazonaws.com/Firmware/{version}/px4_fmu-v6c_default/actuators.json.xz, --version-file /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/lib/version/build_git_version.h
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/lib/component_information && /usr/bin/python3 /home/andrey/Documents/ap/PX4-Autopilot/src/lib/component_information/generate_crc.py /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/component_general.json /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/events/all_events.json.xz --output /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/lib/component_information/checksums.h

component_general.json.xz: component_general.json
	@$(CMAKE_COMMAND) -E touch_nocreate component_general.json.xz

src/lib/component_information/checksums.h: component_general.json
	@$(CMAKE_COMMAND) -E touch_nocreate src/lib/component_information/checksums.h

component_general_json: src/lib/component_information/CMakeFiles/component_general_json
component_general_json: component_general.json
component_general_json: component_general.json.xz
component_general_json: src/lib/component_information/checksums.h
component_general_json: src/lib/component_information/CMakeFiles/component_general_json.dir/build.make

.PHONY : component_general_json

# Rule to build all files generated by this target.
src/lib/component_information/CMakeFiles/component_general_json.dir/build: component_general_json

.PHONY : src/lib/component_information/CMakeFiles/component_general_json.dir/build

src/lib/component_information/CMakeFiles/component_general_json.dir/clean:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/lib/component_information && $(CMAKE_COMMAND) -P CMakeFiles/component_general_json.dir/cmake_clean.cmake
.PHONY : src/lib/component_information/CMakeFiles/component_general_json.dir/clean

src/lib/component_information/CMakeFiles/component_general_json.dir/depend:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/andrey/Documents/ap/PX4-Autopilot /home/andrey/Documents/ap/PX4-Autopilot/src/lib/component_information /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/lib/component_information /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/lib/component_information/CMakeFiles/component_general_json.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/lib/component_information/CMakeFiles/component_general_json.dir/depend
