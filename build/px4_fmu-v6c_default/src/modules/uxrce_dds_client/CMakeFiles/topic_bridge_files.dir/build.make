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

# Utility rule file for topic_bridge_files.

# Include the progress variables for this target.
include src/modules/uxrce_dds_client/CMakeFiles/topic_bridge_files.dir/progress.make

src/modules/uxrce_dds_client/CMakeFiles/topic_bridge_files: src/modules/uxrce_dds_client/dds_topics.h


src/modules/uxrce_dds_client/dds_topics.h: ../../src/modules/uxrce_dds_client/generate_dds_topics.py
src/modules/uxrce_dds_client/dds_topics.h: ../../src/modules/uxrce_dds_client/dds_topics.yaml
src/modules/uxrce_dds_client/dds_topics.h: ../../src/modules/uxrce_dds_client/dds_topics.h.em
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating XRCE-DDS topic bridge"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client && /usr/bin/python3 /home/andrey/Documents/ap/PX4-Autopilot/src/modules/uxrce_dds_client/generate_dds_topics.py --topic-msg-dir /home/andrey/Documents/ap/PX4-Autopilot/msg --client-outdir /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client --dds-topics-file /home/andrey/Documents/ap/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml --template_file /home/andrey/Documents/ap/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.h.em

topic_bridge_files: src/modules/uxrce_dds_client/CMakeFiles/topic_bridge_files
topic_bridge_files: src/modules/uxrce_dds_client/dds_topics.h
topic_bridge_files: src/modules/uxrce_dds_client/CMakeFiles/topic_bridge_files.dir/build.make

.PHONY : topic_bridge_files

# Rule to build all files generated by this target.
src/modules/uxrce_dds_client/CMakeFiles/topic_bridge_files.dir/build: topic_bridge_files

.PHONY : src/modules/uxrce_dds_client/CMakeFiles/topic_bridge_files.dir/build

src/modules/uxrce_dds_client/CMakeFiles/topic_bridge_files.dir/clean:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client && $(CMAKE_COMMAND) -P CMakeFiles/topic_bridge_files.dir/cmake_clean.cmake
.PHONY : src/modules/uxrce_dds_client/CMakeFiles/topic_bridge_files.dir/clean

src/modules/uxrce_dds_client/CMakeFiles/topic_bridge_files.dir/depend:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/andrey/Documents/ap/PX4-Autopilot /home/andrey/Documents/ap/PX4-Autopilot/src/modules/uxrce_dds_client /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client/CMakeFiles/topic_bridge_files.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/modules/uxrce_dds_client/CMakeFiles/topic_bridge_files.dir/depend

