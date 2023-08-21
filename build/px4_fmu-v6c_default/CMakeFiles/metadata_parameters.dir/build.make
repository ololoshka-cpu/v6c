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

# Utility rule file for metadata_parameters.

# Include the progress variables for this target.
include CMakeFiles/metadata_parameters.dir/progress.make

CMakeFiles/metadata_parameters:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating full parameter metadata (markdown, xml, and json)"
	/usr/bin/cmake -E make_directory /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/docs
	/usr/bin/cmake -E make_directory /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/generated_params_metadata
	/usr/bin/python3 /home/andrey/Documents/ap/PX4-Autopilot/Tools/serial/generate_config.py --all-ports --ethernet --params-file /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/generated_params_metadata/serial_params.c --config-files /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/actuators/modal_io/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/cyphal/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/distance_sensor/cm8jl65/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/distance_sensor/leddar_one/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/distance_sensor/lightware_laser_serial/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/distance_sensor/lightware_sf45_serial/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/distance_sensor/tfmini/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/distance_sensor/ulanding_radar/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/dshot/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/gps/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/ins/vectornav/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/optical_flow/thoneflow/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/osd/msp_osd/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/pca9685_pwm_out/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/pwm_out/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/px4io/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/rc/crsf_rc/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/rc_input/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/tap_esc/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/telemetry/frsky_telemetry/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/telemetry/hott/hott_telemetry/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/telemetry/iridiumsbd/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/transponder/sagetech_mxs/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/uavcan/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/uwb/uwb_sr150/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/lib/battery/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/lib/crypto/libtommath/doc/.latexindent.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/lib/events/libevents/libs/cpp/parse/nlohmann_json/.github/ISSUE_TEMPLATE/bug.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/lib/mixer_module/output_functions.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/modules/battery_status/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/modules/control_allocator/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/modules/mavlink/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/modules/payload_deliverer/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/modules/sensors/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/modules/simulation/gz_bridge/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/modules/uxrce_dds_client/module.yaml
	/usr/bin/python3 /home/andrey/Documents/ap/PX4-Autopilot/Tools/module_config/generate_params.py --params-file /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/generated_params_metadata/module_params.c --timer-config /home/andrey/Documents/ap/PX4-Autopilot/boards/px4/fmu-v5/src/timer_config.cpp --board-with-io --ethernet --config-files /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/actuators/modal_io/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/cyphal/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/distance_sensor/cm8jl65/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/distance_sensor/leddar_one/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/distance_sensor/lightware_laser_serial/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/distance_sensor/lightware_sf45_serial/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/distance_sensor/tfmini/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/distance_sensor/ulanding_radar/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/dshot/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/gps/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/ins/vectornav/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/optical_flow/thoneflow/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/osd/msp_osd/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/pca9685_pwm_out/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/pwm_out/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/px4io/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/rc/crsf_rc/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/rc_input/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/tap_esc/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/telemetry/frsky_telemetry/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/telemetry/hott/hott_telemetry/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/telemetry/iridiumsbd/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/transponder/sagetech_mxs/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/uavcan/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/uwb/uwb_sr150/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/lib/battery/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/lib/crypto/libtommath/doc/.latexindent.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/lib/events/libevents/libs/cpp/parse/nlohmann_json/.github/ISSUE_TEMPLATE/bug.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/lib/mixer_module/output_functions.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/modules/battery_status/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/modules/control_allocator/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/modules/mavlink/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/modules/payload_deliverer/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/modules/sensors/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/modules/simulation/gz_bridge/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/modules/uxrce_dds_client/module.yaml
	/usr/bin/python3 /home/andrey/Documents/ap/PX4-Autopilot/src/lib/parameters/px_process_params.py --src-path `find /home/andrey/Documents/ap/PX4-Autopilot/src -maxdepth 4 -type d` /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/generated_params_metadata --inject-xml /home/andrey/Documents/ap/PX4-Autopilot/src/lib/parameters/parameters_injected.xml --markdown /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/docs/parameters.md
	/usr/bin/python3 /home/andrey/Documents/ap/PX4-Autopilot/src/lib/parameters/px_process_params.py --src-path `find /home/andrey/Documents/ap/PX4-Autopilot/src -maxdepth 4 -type d` --inject-xml /home/andrey/Documents/ap/PX4-Autopilot/src/lib/parameters/parameters_injected.xml --json /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/docs/parameters.json --compress
	/usr/bin/python3 /home/andrey/Documents/ap/PX4-Autopilot/src/lib/parameters/px_process_params.py --src-path `find /home/andrey/Documents/ap/PX4-Autopilot/src -maxdepth 4 -type d` --inject-xml /home/andrey/Documents/ap/PX4-Autopilot/src/lib/parameters/parameters_injected.xml --xml /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/docs/parameters.xml

metadata_parameters: CMakeFiles/metadata_parameters
metadata_parameters: CMakeFiles/metadata_parameters.dir/build.make

.PHONY : metadata_parameters

# Rule to build all files generated by this target.
CMakeFiles/metadata_parameters.dir/build: metadata_parameters

.PHONY : CMakeFiles/metadata_parameters.dir/build

CMakeFiles/metadata_parameters.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/metadata_parameters.dir/cmake_clean.cmake
.PHONY : CMakeFiles/metadata_parameters.dir/clean

CMakeFiles/metadata_parameters.dir/depend:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/andrey/Documents/ap/PX4-Autopilot /home/andrey/Documents/ap/PX4-Autopilot /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles/metadata_parameters.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/metadata_parameters.dir/depend
