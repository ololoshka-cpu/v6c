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
include platforms/nuttx/src/px4/stm/stm32h7/hrt/CMakeFiles/arch_hrt.dir/depend.make

# Include the progress variables for this target.
include platforms/nuttx/src/px4/stm/stm32h7/hrt/CMakeFiles/arch_hrt.dir/progress.make

# Include the compile flags for this target's objects.
include platforms/nuttx/src/px4/stm/stm32h7/hrt/CMakeFiles/arch_hrt.dir/flags.make

platforms/nuttx/src/px4/stm/stm32h7/hrt/CMakeFiles/arch_hrt.dir/hrt.c.obj: platforms/nuttx/src/px4/stm/stm32h7/hrt/CMakeFiles/arch_hrt.dir/flags.make
platforms/nuttx/src/px4/stm/stm32h7/hrt/CMakeFiles/arch_hrt.dir/hrt.c.obj: ../../platforms/nuttx/src/px4/stm/stm32_common/hrt/hrt.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object platforms/nuttx/src/px4/stm/stm32h7/hrt/CMakeFiles/arch_hrt.dir/hrt.c.obj"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/platforms/nuttx/src/px4/stm/stm32h7/hrt && /usr/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/arch_hrt.dir/hrt.c.obj   -c /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/src/px4/stm/stm32_common/hrt/hrt.c

platforms/nuttx/src/px4/stm/stm32h7/hrt/CMakeFiles/arch_hrt.dir/hrt.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/arch_hrt.dir/hrt.c.i"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/platforms/nuttx/src/px4/stm/stm32h7/hrt && /usr/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/src/px4/stm/stm32_common/hrt/hrt.c > CMakeFiles/arch_hrt.dir/hrt.c.i

platforms/nuttx/src/px4/stm/stm32h7/hrt/CMakeFiles/arch_hrt.dir/hrt.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/arch_hrt.dir/hrt.c.s"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/platforms/nuttx/src/px4/stm/stm32h7/hrt && /usr/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/src/px4/stm/stm32_common/hrt/hrt.c -o CMakeFiles/arch_hrt.dir/hrt.c.s

# Object files for target arch_hrt
arch_hrt_OBJECTS = \
"CMakeFiles/arch_hrt.dir/hrt.c.obj"

# External object files for target arch_hrt
arch_hrt_EXTERNAL_OBJECTS =

platforms/nuttx/src/px4/stm/stm32h7/hrt/libarch_hrt.a: platforms/nuttx/src/px4/stm/stm32h7/hrt/CMakeFiles/arch_hrt.dir/hrt.c.obj
platforms/nuttx/src/px4/stm/stm32h7/hrt/libarch_hrt.a: platforms/nuttx/src/px4/stm/stm32h7/hrt/CMakeFiles/arch_hrt.dir/build.make
platforms/nuttx/src/px4/stm/stm32h7/hrt/libarch_hrt.a: platforms/nuttx/src/px4/stm/stm32h7/hrt/CMakeFiles/arch_hrt.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C static library libarch_hrt.a"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/platforms/nuttx/src/px4/stm/stm32h7/hrt && $(CMAKE_COMMAND) -P CMakeFiles/arch_hrt.dir/cmake_clean_target.cmake
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/platforms/nuttx/src/px4/stm/stm32h7/hrt && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/arch_hrt.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
platforms/nuttx/src/px4/stm/stm32h7/hrt/CMakeFiles/arch_hrt.dir/build: platforms/nuttx/src/px4/stm/stm32h7/hrt/libarch_hrt.a

.PHONY : platforms/nuttx/src/px4/stm/stm32h7/hrt/CMakeFiles/arch_hrt.dir/build

platforms/nuttx/src/px4/stm/stm32h7/hrt/CMakeFiles/arch_hrt.dir/clean:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/platforms/nuttx/src/px4/stm/stm32h7/hrt && $(CMAKE_COMMAND) -P CMakeFiles/arch_hrt.dir/cmake_clean.cmake
.PHONY : platforms/nuttx/src/px4/stm/stm32h7/hrt/CMakeFiles/arch_hrt.dir/clean

platforms/nuttx/src/px4/stm/stm32h7/hrt/CMakeFiles/arch_hrt.dir/depend:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/andrey/Documents/ap/PX4-Autopilot /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/src/px4/stm/stm32_common/hrt /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/platforms/nuttx/src/px4/stm/stm32h7/hrt /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/platforms/nuttx/src/px4/stm/stm32h7/hrt/CMakeFiles/arch_hrt.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : platforms/nuttx/src/px4/stm/stm32h7/hrt/CMakeFiles/arch_hrt.dir/depend
