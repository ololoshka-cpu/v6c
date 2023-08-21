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

# Utility rule file for failsafe_uorb_struct_header.

# Include the progress variables for this target.
include src/modules/commander/failsafe/CMakeFiles/failsafe_uorb_struct_header.dir/progress.make

src/modules/commander/failsafe/CMakeFiles/failsafe_uorb_struct_header: generated_uorb_struct_field_mapping.h


generated_uorb_struct_field_mapping.h: ../../msg/FailsafeFlags.msg
generated_uorb_struct_field_mapping.h: ../../src/modules/commander/failsafe/emscripten_template.html
generated_uorb_struct_field_mapping.h: ../../src/modules/commander/failsafe/parse_flags_from_msg.py
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Extracting info from failsafe flags msg file"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/commander/failsafe && /usr/bin/python3 /home/andrey/Documents/ap/PX4-Autopilot/src/modules/commander/failsafe/parse_flags_from_msg.py /home/andrey/Documents/ap/PX4-Autopilot/msg/FailsafeFlags.msg /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/generated_uorb_struct_field_mapping.h /home/andrey/Documents/ap/PX4-Autopilot/src/modules/commander/failsafe/emscripten_template.html /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/failsafe_html_template.html

failsafe_html_template.html: generated_uorb_struct_field_mapping.h
	@$(CMAKE_COMMAND) -E touch_nocreate failsafe_html_template.html

failsafe_uorb_struct_header: src/modules/commander/failsafe/CMakeFiles/failsafe_uorb_struct_header
failsafe_uorb_struct_header: generated_uorb_struct_field_mapping.h
failsafe_uorb_struct_header: failsafe_html_template.html
failsafe_uorb_struct_header: src/modules/commander/failsafe/CMakeFiles/failsafe_uorb_struct_header.dir/build.make

.PHONY : failsafe_uorb_struct_header

# Rule to build all files generated by this target.
src/modules/commander/failsafe/CMakeFiles/failsafe_uorb_struct_header.dir/build: failsafe_uorb_struct_header

.PHONY : src/modules/commander/failsafe/CMakeFiles/failsafe_uorb_struct_header.dir/build

src/modules/commander/failsafe/CMakeFiles/failsafe_uorb_struct_header.dir/clean:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/commander/failsafe && $(CMAKE_COMMAND) -P CMakeFiles/failsafe_uorb_struct_header.dir/cmake_clean.cmake
.PHONY : src/modules/commander/failsafe/CMakeFiles/failsafe_uorb_struct_header.dir/clean

src/modules/commander/failsafe/CMakeFiles/failsafe_uorb_struct_header.dir/depend:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/andrey/Documents/ap/PX4-Autopilot /home/andrey/Documents/ap/PX4-Autopilot/src/modules/commander/failsafe /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/commander/failsafe /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/commander/failsafe/CMakeFiles/failsafe_uorb_struct_header.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/modules/commander/failsafe/CMakeFiles/failsafe_uorb_struct_header.dir/depend

