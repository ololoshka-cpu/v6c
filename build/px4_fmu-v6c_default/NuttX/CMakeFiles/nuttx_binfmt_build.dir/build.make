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

# Utility rule file for nuttx_binfmt_build.

# Include the progress variables for this target.
include NuttX/CMakeFiles/nuttx_binfmt_build.dir/progress.make

NuttX/CMakeFiles/nuttx_binfmt_build: NuttX/nuttx/binfmt/libbinfmt.a


NuttX/nuttx/binfmt/libbinfmt.a: ../../platforms/nuttx/NuttX/nuttx/binfmt/binfmt.h
NuttX/nuttx/binfmt/libbinfmt.a: ../../platforms/nuttx/NuttX/nuttx/binfmt/binfmt_copyargv.c
NuttX/nuttx/binfmt/libbinfmt.a: ../../platforms/nuttx/NuttX/nuttx/binfmt/binfmt_coredump.c
NuttX/nuttx/binfmt/libbinfmt.a: ../../platforms/nuttx/NuttX/nuttx/binfmt/binfmt_dumpmodule.c
NuttX/nuttx/binfmt/libbinfmt.a: ../../platforms/nuttx/NuttX/nuttx/binfmt/binfmt_exec.c
NuttX/nuttx/binfmt/libbinfmt.a: ../../platforms/nuttx/NuttX/nuttx/binfmt/binfmt_execmodule.c
NuttX/nuttx/binfmt/libbinfmt.a: ../../platforms/nuttx/NuttX/nuttx/binfmt/binfmt_execsymtab.c
NuttX/nuttx/binfmt/libbinfmt.a: ../../platforms/nuttx/NuttX/nuttx/binfmt/binfmt_exit.c
NuttX/nuttx/binfmt/libbinfmt.a: ../../platforms/nuttx/NuttX/nuttx/binfmt/binfmt_globals.c
NuttX/nuttx/binfmt/libbinfmt.a: ../../platforms/nuttx/NuttX/nuttx/binfmt/binfmt_initialize.c
NuttX/nuttx/binfmt/libbinfmt.a: ../../platforms/nuttx/NuttX/nuttx/binfmt/binfmt_loadmodule.c
NuttX/nuttx/binfmt/libbinfmt.a: ../../platforms/nuttx/NuttX/nuttx/binfmt/binfmt_register.c
NuttX/nuttx/binfmt/libbinfmt.a: ../../platforms/nuttx/NuttX/nuttx/binfmt/binfmt_unloadmodule.c
NuttX/nuttx/binfmt/libbinfmt.a: ../../platforms/nuttx/NuttX/nuttx/binfmt/binfmt_unregister.c
NuttX/nuttx/binfmt/libbinfmt.a: ../../platforms/nuttx/NuttX/nuttx/binfmt/builtin.c
NuttX/nuttx/binfmt/libbinfmt.a: ../../platforms/nuttx/NuttX/nuttx/binfmt/elf.c
NuttX/nuttx/binfmt/libbinfmt.a: ../../platforms/nuttx/NuttX/nuttx/binfmt/libelf/libelf.h
NuttX/nuttx/binfmt/libbinfmt.a: ../../platforms/nuttx/NuttX/nuttx/binfmt/libelf/libelf_addrenv.c
NuttX/nuttx/binfmt/libbinfmt.a: ../../platforms/nuttx/NuttX/nuttx/binfmt/libelf/libelf_bind.c
NuttX/nuttx/binfmt/libbinfmt.a: ../../platforms/nuttx/NuttX/nuttx/binfmt/libelf/libelf_coredump.c
NuttX/nuttx/binfmt/libbinfmt.a: ../../platforms/nuttx/NuttX/nuttx/binfmt/libelf/libelf_ctors.c
NuttX/nuttx/binfmt/libbinfmt.a: ../../platforms/nuttx/NuttX/nuttx/binfmt/libelf/libelf_dtors.c
NuttX/nuttx/binfmt/libbinfmt.a: ../../platforms/nuttx/NuttX/nuttx/binfmt/libelf/libelf_init.c
NuttX/nuttx/binfmt/libbinfmt.a: ../../platforms/nuttx/NuttX/nuttx/binfmt/libelf/libelf_iobuffer.c
NuttX/nuttx/binfmt/libbinfmt.a: ../../platforms/nuttx/NuttX/nuttx/binfmt/libelf/libelf_load.c
NuttX/nuttx/binfmt/libbinfmt.a: ../../platforms/nuttx/NuttX/nuttx/binfmt/libelf/libelf_read.c
NuttX/nuttx/binfmt/libbinfmt.a: ../../platforms/nuttx/NuttX/nuttx/binfmt/libelf/libelf_sections.c
NuttX/nuttx/binfmt/libbinfmt.a: ../../platforms/nuttx/NuttX/nuttx/binfmt/libelf/libelf_symbols.c
NuttX/nuttx/binfmt/libbinfmt.a: ../../platforms/nuttx/NuttX/nuttx/binfmt/libelf/libelf_uninit.c
NuttX/nuttx/binfmt/libbinfmt.a: ../../platforms/nuttx/NuttX/nuttx/binfmt/libelf/libelf_unload.c
NuttX/nuttx/binfmt/libbinfmt.a: ../../platforms/nuttx/NuttX/nuttx/binfmt/libelf/libelf_verify.c
NuttX/nuttx/binfmt/libbinfmt.a: ../../platforms/nuttx/NuttX/nuttx/binfmt/libnxflat/libnxflat.h
NuttX/nuttx/binfmt/libbinfmt.a: ../../platforms/nuttx/NuttX/nuttx/binfmt/libnxflat/libnxflat_addrenv.c
NuttX/nuttx/binfmt/libbinfmt.a: ../../platforms/nuttx/NuttX/nuttx/binfmt/libnxflat/libnxflat_bind.c
NuttX/nuttx/binfmt/libbinfmt.a: ../../platforms/nuttx/NuttX/nuttx/binfmt/libnxflat/libnxflat_init.c
NuttX/nuttx/binfmt/libbinfmt.a: ../../platforms/nuttx/NuttX/nuttx/binfmt/libnxflat/libnxflat_load.c
NuttX/nuttx/binfmt/libbinfmt.a: ../../platforms/nuttx/NuttX/nuttx/binfmt/libnxflat/libnxflat_read.c
NuttX/nuttx/binfmt/libbinfmt.a: ../../platforms/nuttx/NuttX/nuttx/binfmt/libnxflat/libnxflat_uninit.c
NuttX/nuttx/binfmt/libbinfmt.a: ../../platforms/nuttx/NuttX/nuttx/binfmt/libnxflat/libnxflat_unload.c
NuttX/nuttx/binfmt/libbinfmt.a: ../../platforms/nuttx/NuttX/nuttx/binfmt/libnxflat/libnxflat_verify.c
NuttX/nuttx/binfmt/libbinfmt.a: ../../platforms/nuttx/NuttX/nuttx/binfmt/nxflat.c
NuttX/nuttx/binfmt/libbinfmt.a: ../../platforms/nuttx/NuttX/nuttx/include/nuttx/config.h
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating nuttx/binfmt/libbinfmt.a"
	cd /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/nuttx && /usr/bin/cmake -E remove -f /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/nuttx/binfmt/libbinfmt.a
	cd /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/nuttx && find binfmt -type f -name *.o -delete
	cd /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/nuttx && make -C binfmt --no-print-directory --silent all TOPDIR="/home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/nuttx" KERNEL=y EXTRAFLAGS=-D__KERNEL__
	cd /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/nuttx && /usr/bin/cmake -E copy_if_different /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/nuttx/binfmt/libbinfmt.a /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/NuttX/nuttx/binfmt/libbinfmt.a

../../platforms/nuttx/NuttX/nuttx/include/nuttx/config.h: NuttX/nuttx/Make.defs
../../platforms/nuttx/NuttX/nuttx/include/nuttx/config.h: ../../platforms/nuttx/NuttX/nuttx/.config
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating ../../../platforms/nuttx/NuttX/nuttx/include/nuttx/config.h, ../../../platforms/nuttx/NuttX/nuttx/include/arch/chip"
	cd /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/nuttx && /usr/bin/cmake -E copy_if_different /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/NuttX/nuttx/.config /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/nuttx/.config
	cd /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/nuttx && /usr/bin/cmake -E copy_if_different /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/NuttX/nuttx/Make.defs /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/nuttx/Make.defs
	cd /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/nuttx && make --no-print-directory --silent clean_context
	cd /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/nuttx && make --no-print-directory --silent pass1dep > /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/NuttX/nuttx_context.log

../../platforms/nuttx/NuttX/nuttx/include/arch/chip: ../../platforms/nuttx/NuttX/nuttx/include/nuttx/config.h
	@$(CMAKE_COMMAND) -E touch_nocreate ../../platforms/nuttx/NuttX/nuttx/include/arch/chip

../../platforms/nuttx/NuttX/nuttx/.config: ../../boards/px4/fmu-v6c/nuttx-config/nsh/defconfig
../../platforms/nuttx/NuttX/nuttx/.config: ../../platforms/nuttx/NuttX/nuttx/defconfig
../../platforms/nuttx/NuttX/nuttx/.config: NuttX/nuttx/Make.defs
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Generating ../../../platforms/nuttx/NuttX/nuttx/.config"
	cd /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/nuttx && /usr/bin/cmake -E copy_if_different /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/NuttX/nuttx/Make.defs /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/nuttx/Make.defs
	cd /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/nuttx && /usr/bin/cmake -E copy_if_different /home/andrey/Documents/ap/PX4-Autopilot/boards/px4/fmu-v6c/nuttx-config/nsh/defconfig /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/nuttx/.config
	cd /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/nuttx && /usr/bin/cmake -E copy_if_different /home/andrey/Documents/ap/PX4-Autopilot/boards/px4/fmu-v6c/nuttx-config/nsh/defconfig /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/nuttx/defconfig
	cd /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/nuttx && /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/tools/px4_nuttx_make_olddefconfig.sh > /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/NuttX/nuttx_olddefconfig.log
	cd /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/nuttx && /usr/bin/cmake -E copy_if_different /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/nuttx/.config /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/NuttX/nuttx/.config

nuttx_binfmt_build: NuttX/CMakeFiles/nuttx_binfmt_build
nuttx_binfmt_build: NuttX/nuttx/binfmt/libbinfmt.a
nuttx_binfmt_build: ../../platforms/nuttx/NuttX/nuttx/include/nuttx/config.h
nuttx_binfmt_build: ../../platforms/nuttx/NuttX/nuttx/include/arch/chip
nuttx_binfmt_build: ../../platforms/nuttx/NuttX/nuttx/.config
nuttx_binfmt_build: NuttX/CMakeFiles/nuttx_binfmt_build.dir/build.make

.PHONY : nuttx_binfmt_build

# Rule to build all files generated by this target.
NuttX/CMakeFiles/nuttx_binfmt_build.dir/build: nuttx_binfmt_build

.PHONY : NuttX/CMakeFiles/nuttx_binfmt_build.dir/build

NuttX/CMakeFiles/nuttx_binfmt_build.dir/clean:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/NuttX && $(CMAKE_COMMAND) -P CMakeFiles/nuttx_binfmt_build.dir/cmake_clean.cmake
.PHONY : NuttX/CMakeFiles/nuttx_binfmt_build.dir/clean

NuttX/CMakeFiles/nuttx_binfmt_build.dir/depend:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/andrey/Documents/ap/PX4-Autopilot /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/NuttX /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/NuttX/CMakeFiles/nuttx_binfmt_build.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : NuttX/CMakeFiles/nuttx_binfmt_build.dir/depend

