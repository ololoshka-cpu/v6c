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

# Utility rule file for actuators_json.

# Include the progress variables for this target.
include src/lib/metadata/CMakeFiles/actuators_json.dir/progress.make

src/lib/metadata/CMakeFiles/actuators_json: actuators.json


actuators.json: ../../src/lib/battery/module.yaml
actuators.json: ../../src/drivers/distance_sensor/cm8jl65/module.yaml
actuators.json: ../../src/drivers/distance_sensor/leddar_one/module.yaml
actuators.json: ../../src/drivers/distance_sensor/lightware_laser_serial/module.yaml
actuators.json: ../../src/drivers/distance_sensor/tfmini/module.yaml
actuators.json: ../../src/drivers/distance_sensor/ulanding_radar/module.yaml
actuators.json: ../../src/drivers/dshot/module.yaml
actuators.json: ../../src/drivers/gps/module.yaml
actuators.json: ../../src/drivers/ins/vectornav/module.yaml
actuators.json: ../../src/drivers/optical_flow/thoneflow/module.yaml
actuators.json: ../../src/drivers/pwm_out/module.yaml
actuators.json: ../../src/drivers/px4io/module.yaml
actuators.json: ../../src/drivers/telemetry/frsky_telemetry/module.yaml
actuators.json: ../../src/drivers/telemetry/hott/hott_telemetry/module.yaml
actuators.json: ../../src/drivers/uavcan/module.yaml
actuators.json: ../../src/modules/battery_status/module.yaml
actuators.json: ../../src/modules/control_allocator/module.yaml
actuators.json: ../../src/modules/mavlink/module.yaml
actuators.json: ../../src/modules/sensors/module.yaml
actuators.json: ../../src/modules/simulation/pwm_out_sim/module_hil.yaml
actuators.json: ../../src/modules/uxrce_dds_client/module.yaml
actuators.json: ../../Tools/module_config/generate_actuators_metadata.py
actuators.json: ../../src/lib/mixer_module/output_functions.yaml
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating actuators.json"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/lib/metadata && /usr/bin/python3 /home/andrey/Documents/ap/PX4-Autopilot/Tools/module_config/generate_actuators_metadata.py --board-with-io --timer-config /home/andrey/Documents/ap/PX4-Autopilot/boards/px4/fmu-v6c/src/timer_config.cpp --config-files /home/andrey/Documents/ap/PX4-Autopilot/src/lib/battery/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/distance_sensor/cm8jl65/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/distance_sensor/leddar_one/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/distance_sensor/lightware_laser_serial/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/distance_sensor/tfmini/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/distance_sensor/ulanding_radar/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/dshot/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/gps/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/ins/vectornav/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/optical_flow/thoneflow/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/pwm_out/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/px4io/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/telemetry/frsky_telemetry/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/telemetry/hott/hott_telemetry/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/uavcan/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/modules/battery_status/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/modules/control_allocator/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/modules/mavlink/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/modules/sensors/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/modules/simulation/pwm_out_sim/module_hil.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/modules/uxrce_dds_client/module.yaml --compress --board px4_fmu-v6c --output-file /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/actuators.json
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/lib/metadata && /usr/bin/python3 /home/andrey/Documents/ap/PX4-Autopilot/Tools/validate_json.py --schema-file /home/andrey/Documents/ap/PX4-Autopilot/src/modules/mavlink/mavlink/component_information/actuators.schema.json /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/actuators.json --skip-if-no-schema

actuators.json.xz: actuators.json
	@$(CMAKE_COMMAND) -E touch_nocreate actuators.json.xz

actuators_json: src/lib/metadata/CMakeFiles/actuators_json
actuators_json: actuators.json
actuators_json: actuators.json.xz
actuators_json: src/lib/metadata/CMakeFiles/actuators_json.dir/build.make

.PHONY : actuators_json

# Rule to build all files generated by this target.
src/lib/metadata/CMakeFiles/actuators_json.dir/build: actuators_json

.PHONY : src/lib/metadata/CMakeFiles/actuators_json.dir/build

src/lib/metadata/CMakeFiles/actuators_json.dir/clean:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/lib/metadata && $(CMAKE_COMMAND) -P CMakeFiles/actuators_json.dir/cmake_clean.cmake
.PHONY : src/lib/metadata/CMakeFiles/actuators_json.dir/clean

src/lib/metadata/CMakeFiles/actuators_json.dir/depend:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/andrey/Documents/ap/PX4-Autopilot /home/andrey/Documents/ap/PX4-Autopilot/src/lib/metadata /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/lib/metadata /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/lib/metadata/CMakeFiles/actuators_json.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/lib/metadata/CMakeFiles/actuators_json.dir/depend

