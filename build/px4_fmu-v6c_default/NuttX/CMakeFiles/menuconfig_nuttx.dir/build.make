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

# Utility rule file for menuconfig_nuttx.

# Include the progress variables for this target.
include NuttX/CMakeFiles/menuconfig_nuttx.dir/progress.make

NuttX/CMakeFiles/menuconfig_nuttx: ../../platforms/nuttx/NuttX/nuttx/.config
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Running NuttX make menuconfig for nsh"
	cd /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/nuttx && make --no-print-directory --silent menuconfig

../../platforms/nuttx/NuttX/nuttx/.config: ../../boards/px4/fmu-v6c/nuttx-config/nsh/defconfig
../../platforms/nuttx/NuttX/nuttx/.config: ../../platforms/nuttx/NuttX/nuttx/defconfig
../../platforms/nuttx/NuttX/nuttx/.config: NuttX/nuttx/Make.defs
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating ../../../platforms/nuttx/NuttX/nuttx/.config"
	cd /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/nuttx && /usr/bin/cmake -E copy_if_different /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/NuttX/nuttx/Make.defs /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/nuttx/Make.defs
	cd /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/nuttx && /usr/bin/cmake -E copy_if_different /home/andrey/Documents/ap/PX4-Autopilot/boards/px4/fmu-v6c/nuttx-config/nsh/defconfig /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/nuttx/.config
	cd /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/nuttx && /usr/bin/cmake -E copy_if_different /home/andrey/Documents/ap/PX4-Autopilot/boards/px4/fmu-v6c/nuttx-config/nsh/defconfig /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/nuttx/defconfig
	cd /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/nuttx && /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/tools/px4_nuttx_make_olddefconfig.sh > /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/NuttX/nuttx_olddefconfig.log
	cd /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/nuttx && /usr/bin/cmake -E copy_if_different /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/nuttx/.config /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/NuttX/nuttx/.config

menuconfig_nuttx: NuttX/CMakeFiles/menuconfig_nuttx
menuconfig_nuttx: ../../platforms/nuttx/NuttX/nuttx/.config
menuconfig_nuttx: NuttX/CMakeFiles/menuconfig_nuttx.dir/build.make

.PHONY : menuconfig_nuttx

# Rule to build all files generated by this target.
NuttX/CMakeFiles/menuconfig_nuttx.dir/build: menuconfig_nuttx

.PHONY : NuttX/CMakeFiles/menuconfig_nuttx.dir/build

NuttX/CMakeFiles/menuconfig_nuttx.dir/clean:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/NuttX && $(CMAKE_COMMAND) -P CMakeFiles/menuconfig_nuttx.dir/cmake_clean.cmake
.PHONY : NuttX/CMakeFiles/menuconfig_nuttx.dir/clean

NuttX/CMakeFiles/menuconfig_nuttx.dir/depend:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/andrey/Documents/ap/PX4-Autopilot /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/NuttX /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/NuttX/CMakeFiles/menuconfig_nuttx.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : NuttX/CMakeFiles/menuconfig_nuttx.dir/depend

