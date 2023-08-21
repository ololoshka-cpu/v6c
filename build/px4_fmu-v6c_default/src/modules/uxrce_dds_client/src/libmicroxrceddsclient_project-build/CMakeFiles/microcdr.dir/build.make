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
CMAKE_SOURCE_DIR = /home/andrey/Documents/ap/PX4-Autopilot/src/modules/uxrce_dds_client/Micro-XRCE-DDS-Client

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build

# Utility rule file for microcdr.

# Include the progress variables for this target.
include CMakeFiles/microcdr.dir/progress.make

CMakeFiles/microcdr: CMakeFiles/microcdr-complete


CMakeFiles/microcdr-complete: microcdr/src/microcdr-stamp/microcdr-install
CMakeFiles/microcdr-complete: microcdr/src/microcdr-stamp/microcdr-mkdir
CMakeFiles/microcdr-complete: microcdr/src/microcdr-stamp/microcdr-download
CMakeFiles/microcdr-complete: microcdr/src/microcdr-stamp/microcdr-update
CMakeFiles/microcdr-complete: microcdr/src/microcdr-stamp/microcdr-patch
CMakeFiles/microcdr-complete: microcdr/src/microcdr-stamp/microcdr-configure
CMakeFiles/microcdr-complete: microcdr/src/microcdr-stamp/microcdr-build
CMakeFiles/microcdr-complete: microcdr/src/microcdr-stamp/microcdr-install
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Completed 'microcdr'"
	/usr/bin/cmake -E make_directory /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/CMakeFiles
	/usr/bin/cmake -E touch /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/CMakeFiles/microcdr-complete
	/usr/bin/cmake -E touch /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr-stamp/microcdr-done

microcdr/src/microcdr-stamp/microcdr-install: microcdr/src/microcdr-stamp/microcdr-build
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Performing install step for 'microcdr'"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr-build && $(MAKE) install
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr-build && /usr/bin/cmake -E touch /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr-stamp/microcdr-install

microcdr/src/microcdr-stamp/microcdr-mkdir:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Creating directories for 'microcdr'"
	/usr/bin/cmake -E make_directory /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr
	/usr/bin/cmake -E make_directory /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr-build
	/usr/bin/cmake -E make_directory /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/temp_install
	/usr/bin/cmake -E make_directory /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/tmp
	/usr/bin/cmake -E make_directory /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr-stamp
	/usr/bin/cmake -E make_directory /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src
	/usr/bin/cmake -E make_directory /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr-stamp
	/usr/bin/cmake -E touch /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr-stamp/microcdr-mkdir

microcdr/src/microcdr-stamp/microcdr-download: microcdr/src/microcdr-stamp/microcdr-gitinfo.txt
microcdr/src/microcdr-stamp/microcdr-download: microcdr/src/microcdr-stamp/microcdr-mkdir
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Performing download step (git clone) for 'microcdr'"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src && /usr/bin/cmake -P /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/tmp/microcdr-gitclone.cmake
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src && /usr/bin/cmake -E touch /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr-stamp/microcdr-download

microcdr/src/microcdr-stamp/microcdr-update: microcdr/src/microcdr-stamp/microcdr-download
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Performing update step for 'microcdr'"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr && /usr/bin/cmake -P /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/tmp/microcdr-gitupdate.cmake

microcdr/src/microcdr-stamp/microcdr-patch: microcdr/src/microcdr-stamp/microcdr-download
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "No patch step for 'microcdr'"
	/usr/bin/cmake -E echo_append
	/usr/bin/cmake -E touch /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr-stamp/microcdr-patch

microcdr/src/microcdr-stamp/microcdr-configure: microcdr/tmp/microcdr-cfgcmd.txt
microcdr/src/microcdr-stamp/microcdr-configure: microcdr/src/microcdr-stamp/microcdr-update
microcdr/src/microcdr-stamp/microcdr-configure: microcdr/src/microcdr-stamp/microcdr-patch
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Performing configure step for 'microcdr'"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr-build && /usr/bin/cmake -DBUILD_SHARED_LIBS=OFF -DCMAKE_INSTALL_PREFIX=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/temp_install -DCMAKE_TOOLCHAIN_FILE=/home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/cmake/Toolchain-arm-none-eabi.cmake -DCMAKE_SYSROOT:PATH= "-DCMAKE_C_FLAGS:STRING=-mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard -I/home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/include/cxx -I/home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/nuttx/include/cxx -I/home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/nuttx/include -I/home/andrey/Documents/ap/PX4-Autopilot/boards/px4/fmu-v6c/src -I/home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/src/px4/common/include -I/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default -I/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/lib -I/home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/src/px4/stm/stm32h7/include -I/home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/src/px4/common/include -I/home/andrey/Documents/ap/PX4-Autopilot/platforms/common -I/home/andrey/Documents/ap/PX4-Autopilot/platforms/common/include -I/home/andrey/Documents/ap/PX4-Autopilot/src -I/home/andrey/Documents/ap/PX4-Autopilot/src/include -I/home/andrey/Documents/ap/PX4-Autopilot/src/lib -I/home/andrey/Documents/ap/PX4-Autopilot/src/lib/matrix -I/home/andrey/Documents/ap/PX4-Autopilot/src/modules -I/home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/nuttx/arch/arm/src/armv7-m -I/home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/nuttx/arch/arm/src/chip -I/home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/nuttx/arch/arm/src/common -I/home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/apps/include -I/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/external/Install/include" "-DCMAKE_CXX_FLAGS:STRING=-mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard -I/home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/include/cxx -I/home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/nuttx/include/cxx -I/home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/nuttx/include -I/home/andrey/Documents/ap/PX4-Autopilot/boards/px4/fmu-v6c/src -I/home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/src/px4/common/include -I/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default -I/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/lib -I/home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/src/px4/stm/stm32h7/include -I/home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/src/px4/common/include -I/home/andrey/Documents/ap/PX4-Autopilot/platforms/common -I/home/andrey/Documents/ap/PX4-Autopilot/platforms/common/include -I/home/andrey/Documents/ap/PX4-Autopilot/src -I/home/andrey/Documents/ap/PX4-Autopilot/src/include -I/home/andrey/Documents/ap/PX4-Autopilot/src/lib -I/home/andrey/Documents/ap/PX4-Autopilot/src/lib/matrix -I/home/andrey/Documents/ap/PX4-Autopilot/src/modules -I/home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/nuttx/arch/arm/src/armv7-m -I/home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/nuttx/arch/arm/src/chip -I/home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/nuttx/arch/arm/src/common -I/home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/apps/include -I/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/external/Install/include" -DCMAKE_EXE_LINKER_FLAGS:STRING=--specs=nosys.specs -DCMAKE_BUILD_TYPE:STRING=MinSizeRel -DCONFIG_BIG_ENDIANNESS=OFF -DUCDR_PIC=OFF "-GUnix Makefiles" -C/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/tmp/microcdr-cache-MinSizeRel.cmake /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr-build && /usr/bin/cmake -E touch /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr-stamp/microcdr-configure

microcdr/src/microcdr-stamp/microcdr-build: microcdr/src/microcdr-stamp/microcdr-configure
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Performing build step for 'microcdr'"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr-build && $(MAKE)
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr-build && /usr/bin/cmake -E touch /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr-stamp/microcdr-build

microcdr: CMakeFiles/microcdr
microcdr: CMakeFiles/microcdr-complete
microcdr: microcdr/src/microcdr-stamp/microcdr-install
microcdr: microcdr/src/microcdr-stamp/microcdr-mkdir
microcdr: microcdr/src/microcdr-stamp/microcdr-download
microcdr: microcdr/src/microcdr-stamp/microcdr-update
microcdr: microcdr/src/microcdr-stamp/microcdr-patch
microcdr: microcdr/src/microcdr-stamp/microcdr-configure
microcdr: microcdr/src/microcdr-stamp/microcdr-build
microcdr: CMakeFiles/microcdr.dir/build.make

.PHONY : microcdr

# Rule to build all files generated by this target.
CMakeFiles/microcdr.dir/build: microcdr

.PHONY : CMakeFiles/microcdr.dir/build

CMakeFiles/microcdr.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/microcdr.dir/cmake_clean.cmake
.PHONY : CMakeFiles/microcdr.dir/clean

CMakeFiles/microcdr.dir/depend:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/andrey/Documents/ap/PX4-Autopilot/src/modules/uxrce_dds_client/Micro-XRCE-DDS-Client /home/andrey/Documents/ap/PX4-Autopilot/src/modules/uxrce_dds_client/Micro-XRCE-DDS-Client /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/CMakeFiles/microcdr.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/microcdr.dir/depend
