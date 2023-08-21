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

# Utility rule file for metadata_extract_events.

# Include the progress variables for this target.
include CMakeFiles/metadata_extract_events.dir/progress.make

CMakeFiles/metadata_extract_events:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Extracting events from full source"
	/usr/bin/cmake -E make_directory /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/events
	/usr/bin/python3 /home/andrey/Documents/ap/PX4-Autopilot/Tools/px_process_events.py --src-path /home/andrey/Documents/ap/PX4-Autopilot/src --json /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/events/px4_full.json
	/usr/bin/python3 /home/andrey/Documents/ap/PX4-Autopilot/src/lib/events/libevents/scripts/combine.py /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/events/px4_full.json /home/andrey/Documents/ap/PX4-Autopilot/src/lib/events/libevents/events/common.json /home/andrey/Documents/ap/PX4-Autopilot/src/lib/events/enums.json --output /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/events/all_events_full.json
	/usr/bin/python3 /home/andrey/Documents/ap/PX4-Autopilot/src/lib/events/libevents/scripts/validate.py /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/events/all_events_full.json
	/usr/bin/python3 /home/andrey/Documents/ap/PX4-Autopilot/Tools/compress.py /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/events/all_events_full.json

metadata_extract_events: CMakeFiles/metadata_extract_events
metadata_extract_events: CMakeFiles/metadata_extract_events.dir/build.make

.PHONY : metadata_extract_events

# Rule to build all files generated by this target.
CMakeFiles/metadata_extract_events.dir/build: metadata_extract_events

.PHONY : CMakeFiles/metadata_extract_events.dir/build

CMakeFiles/metadata_extract_events.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/metadata_extract_events.dir/cmake_clean.cmake
.PHONY : CMakeFiles/metadata_extract_events.dir/clean

CMakeFiles/metadata_extract_events.dir/depend:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/andrey/Documents/ap/PX4-Autopilot /home/andrey/Documents/ap/PX4-Autopilot /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles/metadata_extract_events.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/metadata_extract_events.dir/depend

