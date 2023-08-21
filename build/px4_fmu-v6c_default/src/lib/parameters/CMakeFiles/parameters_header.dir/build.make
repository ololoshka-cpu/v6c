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

# Utility rule file for parameters_header.

# Include the progress variables for this target.
include src/lib/parameters/CMakeFiles/parameters_header.dir/progress.make

src/lib/parameters/CMakeFiles/parameters_header: src/lib/parameters/px4_parameters.hpp


src/lib/parameters/px4_parameters.hpp: parameters.xml
src/lib/parameters/px4_parameters.hpp: ../../src/lib/parameters/px_generate_params.py
src/lib/parameters/px4_parameters.hpp: ../../src/lib/parameters/templates/px4_parameters.hpp.jinja
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating px4_parameters.hpp"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/lib/parameters && /usr/bin/python3 /home/andrey/Documents/ap/PX4-Autopilot/src/lib/parameters/px_generate_params.py --xml /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/parameters.xml --dest /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/lib/parameters

parameters.xml: ../../src/drivers/actuators/modal_io/modal_io_params.c
parameters.xml: ../../src/drivers/adc/ads1115/ads1115_params.c
parameters.xml: ../../src/drivers/barometer/goertek/spl06/parameters.c
parameters.xml: ../../src/drivers/batt_smbus/parameters.c
parameters.xml: ../../src/drivers/camera_capture/camera_capture_params.c
parameters.xml: ../../src/drivers/camera_trigger/camera_trigger_params.c
parameters.xml: ../../src/drivers/cyphal/parameters.c
parameters.xml: ../../src/drivers/differential_pressure/ets/parameters.c
parameters.xml: ../../src/drivers/differential_pressure/ms4515/parameters.c
parameters.xml: ../../src/drivers/differential_pressure/ms4525do/parameters.c
parameters.xml: ../../src/drivers/differential_pressure/ms5525dso/parameters.c
parameters.xml: ../../src/drivers/differential_pressure/sdp3x/parameters.c
parameters.xml: ../../src/drivers/distance_sensor/lightware_laser_i2c/parameters.c
parameters.xml: ../../src/drivers/distance_sensor/lightware_laser_serial/parameters.c
parameters.xml: ../../src/drivers/distance_sensor/ll40ls/parameters.c
parameters.xml: ../../src/drivers/distance_sensor/mappydot/parameters.c
parameters.xml: ../../src/drivers/distance_sensor/mb12xx/parameters.c
parameters.xml: ../../src/drivers/distance_sensor/pga460/parameters.c
parameters.xml: ../../src/drivers/distance_sensor/srf05/parameters.c
parameters.xml: ../../src/drivers/distance_sensor/teraranger/parameters.c
parameters.xml: ../../src/drivers/distance_sensor/tf02pro/parameters.c
parameters.xml: ../../src/drivers/distance_sensor/vl53l0x/params.c
parameters.xml: ../../src/drivers/distance_sensor/vl53l1x/params.c
parameters.xml: ../../src/drivers/gps/params.c
parameters.xml: ../../src/drivers/heater/heater_params.c
parameters.xml: ../../src/drivers/hygrometer/sht3x/sht3x_params.c
parameters.xml: ../../src/drivers/imu/analog_devices/adis16448/parameters.c
parameters.xml: ../../src/drivers/imu/analog_devices/adis16507/parameters.c
parameters.xml: ../../src/drivers/irlock/parameters.c
parameters.xml: ../../src/drivers/optical_flow/paa3905/parameters.c
parameters.xml: ../../src/drivers/optical_flow/paw3902/parameters.c
parameters.xml: ../../src/drivers/optical_flow/pmw3901/parameters.c
parameters.xml: ../../src/drivers/optical_flow/px4flow/parameters.c
parameters.xml: ../../src/drivers/osd/atxxxx/params.c
parameters.xml: ../../src/drivers/power_monitor/ina220/ina220_params.c
parameters.xml: ../../src/drivers/power_monitor/ina226/ina226_params.c
parameters.xml: ../../src/drivers/power_monitor/ina228/ina228_params.c
parameters.xml: ../../src/drivers/power_monitor/ina238/ina238_params.c
parameters.xml: ../../src/drivers/power_monitor/voxlpm/voxlpm_params.c
parameters.xml: ../../src/drivers/pps_capture/pps_capture_params.c
parameters.xml: ../../src/drivers/px4io/px4io_params.c
parameters.xml: ../../src/drivers/rpm/pcf8583/parameters.c
parameters.xml: ../../src/drivers/smart_battery/batmon/batmon_params.c
parameters.xml: ../../src/drivers/tap_esc/tap_esc_params.c
parameters.xml: ../../src/drivers/telemetry/bst/bst_params.c
parameters.xml: ../../src/drivers/telemetry/iridiumsbd/iridiumsbd_params.c
parameters.xml: ../../src/drivers/transponder/sagetech_mxs/parameters.c
parameters.xml: ../../src/drivers/uavcan/uavcan_params.c
parameters.xml: ../../src/drivers/uavcannode/uavcannode_params.c
parameters.xml: ../../src/lib/adsb/parameters.c
parameters.xml: ../../src/lib/circuit_breaker/circuit_breaker_params.c
parameters.xml: ../../src/lib/collision_prevention/collisionprevention_params.c
parameters.xml: ../../src/lib/controllib/controllib_test/test_params.c
parameters.xml: ../../src/lib/led/led_params.c
parameters.xml: ../../src/lib/mixer_module/motor_params.c
parameters.xml: ../../src/lib/mixer_module/params.c
parameters.xml: ../../src/lib/systemlib/system_params.c
parameters.xml: ../../src/lib/weather_vane/weathervane_params.c
parameters.xml: ../../src/modules/airspeed_selector/airspeed_selector_params.c
parameters.xml: ../../src/modules/attitude_estimator_q/attitude_estimator_q_params.c
parameters.xml: ../../src/modules/commander/commander_params.c
parameters.xml: ../../src/modules/commander/failure_detector/failure_detector_params.c
parameters.xml: ../../src/modules/dataman/parameters.c
parameters.xml: ../../src/modules/ekf2/ekf2_params.c
parameters.xml: ../../src/modules/events/events_params.c
parameters.xml: ../../src/modules/flight_mode_manager/flight_mode_manager_params.c
parameters.xml: ../../src/modules/flight_mode_manager/tasks/AutoFollowTarget/follow_target_params.c
parameters.xml: ../../src/modules/flight_mode_manager/tasks/Orbit/flight_task_orbit_params.c
parameters.xml: ../../src/modules/fw_att_control/fw_att_control_params.c
parameters.xml: ../../src/modules/fw_autotune_attitude_control/fw_autotune_attitude_control_params.c
parameters.xml: ../../src/modules/fw_pos_control/fw_path_navigation_params.c
parameters.xml: ../../src/modules/fw_pos_control/launchdetection/launchdetection_params.c
parameters.xml: ../../src/modules/fw_pos_control/runway_takeoff/runway_takeoff_params.c
parameters.xml: ../../src/modules/fw_rate_control/fw_rate_control_params.c
parameters.xml: ../../src/modules/gimbal/gimbal_params.c
parameters.xml: ../../src/modules/gyro_calibration/parameters.c
parameters.xml: ../../src/modules/gyro_fft/parameters.c
parameters.xml: ../../src/modules/land_detector/land_detector_params.c
parameters.xml: ../../src/modules/landing_target_estimator/landing_target_estimator_params.c
parameters.xml: ../../src/modules/load_mon/params.c
parameters.xml: ../../src/modules/local_position_estimator/params.c
parameters.xml: ../../src/modules/logger/params.c
parameters.xml: ../../src/modules/mag_bias_estimator/params.c
parameters.xml: ../../src/modules/manual_control/manual_control_params.c
parameters.xml: ../../src/modules/mavlink/mavlink_params.c
parameters.xml: ../../src/modules/mc_att_control/mc_att_control_params.c
parameters.xml: ../../src/modules/mc_autotune_attitude_control/mc_autotune_attitude_control_params.c
parameters.xml: ../../src/modules/mc_hover_thrust_estimator/hover_thrust_estimator_params.c
parameters.xml: ../../src/modules/mc_pos_control/multicopter_altitude_mode_params.c
parameters.xml: ../../src/modules/mc_pos_control/multicopter_autonomous_params.c
parameters.xml: ../../src/modules/mc_pos_control/multicopter_nudging_params.c
parameters.xml: ../../src/modules/mc_pos_control/multicopter_position_control_gain_params.c
parameters.xml: ../../src/modules/mc_pos_control/multicopter_position_control_limits_params.c
parameters.xml: ../../src/modules/mc_pos_control/multicopter_position_control_params.c
parameters.xml: ../../src/modules/mc_pos_control/multicopter_position_mode_params.c
parameters.xml: ../../src/modules/mc_pos_control/multicopter_responsiveness_params.c
parameters.xml: ../../src/modules/mc_pos_control/multicopter_stabilized_mode_params.c
parameters.xml: ../../src/modules/mc_pos_control/multicopter_takeoff_land_params.c
parameters.xml: ../../src/modules/mc_rate_control/mc_rate_control_params.c
parameters.xml: ../../src/modules/navigator/geofence_params.c
parameters.xml: ../../src/modules/navigator/mission_params.c
parameters.xml: ../../src/modules/navigator/navigator_params.c
parameters.xml: ../../src/modules/navigator/precland_params.c
parameters.xml: ../../src/modules/navigator/rtl_params.c
parameters.xml: ../../src/modules/navigator/vtol_takeoff_params.c
parameters.xml: ../../src/modules/rc_update/params.c
parameters.xml: ../../src/modules/rover_pos_control/rover_pos_control_params.c
parameters.xml: ../../src/modules/sensors/sensor_params.c
parameters.xml: ../../src/modules/sensors/vehicle_acceleration/imu_accel_parameters.c
parameters.xml: ../../src/modules/sensors/vehicle_air_data/params.c
parameters.xml: ../../src/modules/sensors/vehicle_angular_velocity/imu_gyro_parameters.c
parameters.xml: ../../src/modules/sensors/vehicle_gps_position/params.c
parameters.xml: ../../src/modules/sensors/vehicle_imu/imu_parameters.c
parameters.xml: ../../src/modules/simulation/battery_simulator/battery_simulator_params.c
parameters.xml: ../../src/modules/simulation/gz_bridge/parameters.c
parameters.xml: ../../src/modules/simulation/sensor_airspeed_sim/parameters.c
parameters.xml: ../../src/modules/simulation/sensor_baro_sim/parameters.c
parameters.xml: ../../src/modules/simulation/sensor_gps_sim/parameters.c
parameters.xml: ../../src/modules/simulation/sensor_mag_sim/parameters.c
parameters.xml: ../../src/modules/simulation/simulator_sih/sih_params.c
parameters.xml: ../../src/modules/uuv_att_control/uuv_att_control_params.c
parameters.xml: ../../src/modules/uuv_pos_control/uuv_pos_control_params.c
parameters.xml: ../../src/modules/vtol_att_control/standard_params.c
parameters.xml: ../../src/modules/vtol_att_control/tiltrotor_params.c
parameters.xml: ../../src/modules/vtol_att_control/vtol_att_control_params.c
parameters.xml: ../../src/systemcmds/tests/params.c
parameters.xml: generated_params/serial_params.c
parameters.xml: generated_params/module_params.c
parameters.xml: ../../src/lib/parameters/parameters_injected.xml
parameters.xml: ../../src/lib/parameters/px4params/srcparser.py
parameters.xml: ../../src/lib/parameters/px4params/srcscanner.py
parameters.xml: ../../src/lib/parameters/px4params/jsonout.py
parameters.xml: ../../src/lib/parameters/px4params/xmlout.py
parameters.xml: ../../src/lib/parameters/px_process_params.py
parameters.xml: ../../src/lib/parameters/parameters_injected.xml
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating parameters.xml"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/lib/parameters && /usr/bin/python3 /home/andrey/Documents/ap/PX4-Autopilot/src/lib/parameters/px_process_params.py --src-path /home/andrey/Documents/ap/PX4-Autopilot/src/lib/adsb /home/andrey/Documents/ap/PX4-Autopilot/src/lib/airspeed /home/andrey/Documents/ap/PX4-Autopilot/src/lib/avoidance /home/andrey/Documents/ap/PX4-Autopilot/src/lib/battery /home/andrey/Documents/ap/PX4-Autopilot/src/lib/bezier /home/andrey/Documents/ap/PX4-Autopilot/src/lib/button /home/andrey/Documents/ap/PX4-Autopilot/src/lib/cdev /home/andrey/Documents/ap/PX4-Autopilot/src/lib/circuit_breaker /home/andrey/Documents/ap/PX4-Autopilot/src/lib/collision_prevention /home/andrey/Documents/ap/PX4-Autopilot/src/lib/controllib /home/andrey/Documents/ap/PX4-Autopilot/src/lib/conversion /home/andrey/Documents/ap/PX4-Autopilot/src/lib/dataman_client /home/andrey/Documents/ap/PX4-Autopilot/src/lib/drivers/accelerometer /home/andrey/Documents/ap/PX4-Autopilot/src/lib/drivers/device /home/andrey/Documents/ap/PX4-Autopilot/src/lib/drivers/gyroscope /home/andrey/Documents/ap/PX4-Autopilot/src/lib/drivers/led /home/andrey/Documents/ap/PX4-Autopilot/src/lib/drivers/magnetometer /home/andrey/Documents/ap/PX4-Autopilot/src/lib/drivers/rangefinder /home/andrey/Documents/ap/PX4-Autopilot/src/lib/drivers/smbus /home/andrey/Documents/ap/PX4-Autopilot/src/lib/drivers/smbus_sbs /home/andrey/Documents/ap/PX4-Autopilot/src/lib/hysteresis /home/andrey/Documents/ap/PX4-Autopilot/src/lib/l1 /home/andrey/Documents/ap/PX4-Autopilot/src/lib/led /home/andrey/Documents/ap/PX4-Autopilot/src/lib/mathlib /home/andrey/Documents/ap/PX4-Autopilot/src/lib/mixer_module /home/andrey/Documents/ap/PX4-Autopilot/src/lib/motion_planning /home/andrey/Documents/ap/PX4-Autopilot/src/lib/npfg /home/andrey/Documents/ap/PX4-Autopilot/src/lib/pid /home/andrey/Documents/ap/PX4-Autopilot/src/lib/rate_control /home/andrey/Documents/ap/PX4-Autopilot/src/lib/sensor_calibration /home/andrey/Documents/ap/PX4-Autopilot/src/lib/slew_rate /home/andrey/Documents/ap/PX4-Autopilot/src/lib/systemlib /home/andrey/Documents/ap/PX4-Autopilot/src/lib/system_identification /home/andrey/Documents/ap/PX4-Autopilot/src/lib/tecs /home/andrey/Documents/ap/PX4-Autopilot/src/lib/terrain_estimation /home/andrey/Documents/ap/PX4-Autopilot/src/lib/timesync /home/andrey/Documents/ap/PX4-Autopilot/src/lib/tunes /home/andrey/Documents/ap/PX4-Autopilot/src/lib/weather_vane /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/src/px4/common/gpio/mcp23009 /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/src/px4/common/srgbled /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/src/px4/stm/stm32h7/adc /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/src/px4/stm/stm32_common/board_critmon /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/src/px4/stm/stm32_common/board_hw_info /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/src/px4/stm/stm32_common/board_reset /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/src/px4/stm/stm32_common/dshot /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/src/px4/stm/stm32_common/hrt /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/src/px4/stm/stm32_common/led_pwm /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/src/px4/stm/stm32_common/io_pins /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/src/px4/stm/stm32_common/spi /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/src/px4/stm/stm32_common/tone_alarm /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/src/px4/stm/stm32_common/version /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/src/px4/stm/stm32h7/px4io_serial /home/andrey/Documents/ap/PX4-Autopilot/platforms/common/uORB /home/andrey/Documents/ap/PX4-Autopilot/platforms/common/px4_work_queue /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/adc/board_adc /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/barometer/ms5611 /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/batt_smbus /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/camera_capture /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/camera_trigger /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/differential_pressure/ms4525do /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/differential_pressure/ms5525dso /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/differential_pressure/sdp3x /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/distance_sensor/cm8jl65 /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/distance_sensor/gy_us42 /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/distance_sensor/leddar_one /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/distance_sensor/lightware_laser_i2c /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/distance_sensor/lightware_laser_serial /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/distance_sensor/ll40ls /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/distance_sensor/ll40ls_pwm /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/distance_sensor/mappydot /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/distance_sensor/mb12xx /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/distance_sensor/pga460 /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/distance_sensor/srf02 /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/distance_sensor/teraranger /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/distance_sensor/tf02pro /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/distance_sensor/tfmini /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/distance_sensor/ulanding_radar /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/distance_sensor/vl53l0x /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/distance_sensor/vl53l1x /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/dshot /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/gps /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/heater /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/imu/bosch/bmi055 /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/imu/invensense/icm42688p /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/ins/vectornav /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/lights/rgbled /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/lights/rgbled_is31fl3195 /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/lights/rgbled_lp5562 /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/lights/rgbled_ncp5623c /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/magnetometer/akm/ak09916 /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/magnetometer/akm/ak8963 /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/magnetometer/bosch/bmm150 /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/magnetometer/hmc5883 /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/magnetometer/isentek/ist8308 /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/magnetometer/isentek/ist8310 /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/magnetometer/lis2mdl /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/magnetometer/lis3mdl /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/magnetometer/lsm303agr /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/magnetometer/qmc5883l /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/magnetometer/rm3100 /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/magnetometer/vtrantech/vcm1193l /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/optical_flow/paa3905 /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/optical_flow/paw3902 /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/optical_flow/pmw3901 /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/optical_flow/px4flow /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/optical_flow/thoneflow /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/power_monitor/ina226 /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/power_monitor/ina228 /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/power_monitor/ina238 /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/pwm_out /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/px4io /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/telemetry/bst /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/telemetry/frsky_telemetry /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/telemetry/hott /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/telemetry/hott/hott_sensors /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/telemetry/hott/hott_telemetry /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/tone_alarm /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/uavcan /home/andrey/Documents/ap/PX4-Autopilot/src/modules/airspeed_selector /home/andrey/Documents/ap/PX4-Autopilot/src/modules/battery_status /home/andrey/Documents/ap/PX4-Autopilot/src/modules/camera_feedback /home/andrey/Documents/ap/PX4-Autopilot/src/modules/commander/failure_detector /home/andrey/Documents/ap/PX4-Autopilot/src/modules/commander/HealthAndArmingChecks /home/andrey/Documents/ap/PX4-Autopilot/src/modules/commander/failsafe /home/andrey/Documents/ap/PX4-Autopilot/src/modules/commander/Arming/ArmAuthorization /home/andrey/Documents/ap/PX4-Autopilot/src/modules/commander /home/andrey/Documents/ap/PX4-Autopilot/src/modules/control_allocator/ActuatorEffectiveness /home/andrey/Documents/ap/PX4-Autopilot/src/modules/control_allocator/ControlAllocation /home/andrey/Documents/ap/PX4-Autopilot/src/modules/control_allocator /home/andrey/Documents/ap/PX4-Autopilot/src/modules/dataman /home/andrey/Documents/ap/PX4-Autopilot/src/modules/ekf2/Utility /home/andrey/Documents/ap/PX4-Autopilot/src/modules/ekf2 /home/andrey/Documents/ap/PX4-Autopilot/src/modules/esc_battery /home/andrey/Documents/ap/PX4-Autopilot/src/modules/events /home/andrey/Documents/ap/PX4-Autopilot/src/modules/flight_mode_manager/tasks/FlightTask /home/andrey/Documents/ap/PX4-Autopilot/src/modules/flight_mode_manager/tasks/Utility /home/andrey/Documents/ap/PX4-Autopilot/src/modules/flight_mode_manager/tasks/Auto /home/andrey/Documents/ap/PX4-Autopilot/src/modules/flight_mode_manager/tasks/Descend /home/andrey/Documents/ap/PX4-Autopilot/src/modules/flight_mode_manager/tasks/Failsafe /home/andrey/Documents/ap/PX4-Autopilot/src/modules/flight_mode_manager/tasks/ManualAcceleration /home/andrey/Documents/ap/PX4-Autopilot/src/modules/flight_mode_manager/tasks/ManualAltitude /home/andrey/Documents/ap/PX4-Autopilot/src/modules/flight_mode_manager/tasks/ManualAltitudeSmoothVel /home/andrey/Documents/ap/PX4-Autopilot/src/modules/flight_mode_manager/tasks/ManualPosition /home/andrey/Documents/ap/PX4-Autopilot/src/modules/flight_mode_manager/tasks/ManualPositionSmoothVel /home/andrey/Documents/ap/PX4-Autopilot/src/modules/flight_mode_manager/tasks/Transition /home/andrey/Documents/ap/PX4-Autopilot/src/modules/flight_mode_manager/tasks/AutoFollowTarget/follow_target_estimator /home/andrey/Documents/ap/PX4-Autopilot/src/modules/flight_mode_manager/tasks/AutoFollowTarget /home/andrey/Documents/ap/PX4-Autopilot/src/modules/flight_mode_manager/tasks/Orbit /home/andrey/Documents/ap/PX4-Autopilot/src/modules/flight_mode_manager /home/andrey/Documents/ap/PX4-Autopilot/src/modules/fw_att_control /home/andrey/Documents/ap/PX4-Autopilot/src/modules/fw_autotune_attitude_control /home/andrey/Documents/ap/PX4-Autopilot/src/modules/fw_pos_control/launchdetection /home/andrey/Documents/ap/PX4-Autopilot/src/modules/fw_pos_control/runway_takeoff /home/andrey/Documents/ap/PX4-Autopilot/src/modules/fw_pos_control /home/andrey/Documents/ap/PX4-Autopilot/src/modules/fw_rate_control /home/andrey/Documents/ap/PX4-Autopilot/src/modules/gimbal /home/andrey/Documents/ap/PX4-Autopilot/src/modules/gyro_calibration /home/andrey/Documents/ap/PX4-Autopilot/src/modules/gyro_fft /home/andrey/Documents/ap/PX4-Autopilot/src/modules/land_detector /home/andrey/Documents/ap/PX4-Autopilot/src/modules/landing_target_estimator /home/andrey/Documents/ap/PX4-Autopilot/src/modules/load_mon /home/andrey/Documents/ap/PX4-Autopilot/src/modules/logger /home/andrey/Documents/ap/PX4-Autopilot/src/modules/mag_bias_estimator /home/andrey/Documents/ap/PX4-Autopilot/src/modules/manual_control /home/andrey/Documents/ap/PX4-Autopilot/src/modules/mavlink /home/andrey/Documents/ap/PX4-Autopilot/src/modules/mc_att_control/AttitudeControl /home/andrey/Documents/ap/PX4-Autopilot/src/modules/mc_att_control /home/andrey/Documents/ap/PX4-Autopilot/src/modules/mc_autotune_attitude_control /home/andrey/Documents/ap/PX4-Autopilot/src/modules/mc_hover_thrust_estimator /home/andrey/Documents/ap/PX4-Autopilot/src/modules/mc_pos_control/PositionControl /home/andrey/Documents/ap/PX4-Autopilot/src/modules/mc_pos_control/Takeoff /home/andrey/Documents/ap/PX4-Autopilot/src/modules/mc_pos_control /home/andrey/Documents/ap/PX4-Autopilot/src/modules/mc_rate_control /home/andrey/Documents/ap/PX4-Autopilot/src/modules/navigator/GeofenceBreachAvoidance /home/andrey/Documents/ap/PX4-Autopilot/src/modules/navigator/MissionFeasibility /home/andrey/Documents/ap/PX4-Autopilot/src/modules/navigator /home/andrey/Documents/ap/PX4-Autopilot/src/modules/rc_update /home/andrey/Documents/ap/PX4-Autopilot/src/modules/rover_pos_control /home/andrey/Documents/ap/PX4-Autopilot/src/modules/sensors/data_validator /home/andrey/Documents/ap/PX4-Autopilot/src/modules/sensors/vehicle_acceleration /home/andrey/Documents/ap/PX4-Autopilot/src/modules/sensors/vehicle_imu /home/andrey/Documents/ap/PX4-Autopilot/src/modules/sensors/vehicle_air_data /home/andrey/Documents/ap/PX4-Autopilot/src/modules/sensors/vehicle_angular_velocity /home/andrey/Documents/ap/PX4-Autopilot/src/modules/sensors/vehicle_gps_position /home/andrey/Documents/ap/PX4-Autopilot/src/modules/sensors/vehicle_magnetometer /home/andrey/Documents/ap/PX4-Autopilot/src/modules/sensors/vehicle_optical_flow /home/andrey/Documents/ap/PX4-Autopilot/src/modules/sensors /home/andrey/Documents/ap/PX4-Autopilot/src/modules/simulation/pwm_out_sim /home/andrey/Documents/ap/PX4-Autopilot/src/modules/simulation/sensor_baro_sim /home/andrey/Documents/ap/PX4-Autopilot/src/modules/simulation/sensor_gps_sim /home/andrey/Documents/ap/PX4-Autopilot/src/modules/simulation/sensor_mag_sim /home/andrey/Documents/ap/PX4-Autopilot/src/modules/simulation/simulator_sih /home/andrey/Documents/ap/PX4-Autopilot/src/modules/temperature_compensation /home/andrey/Documents/ap/PX4-Autopilot/src/modules/uxrce_dds_client /home/andrey/Documents/ap/PX4-Autopilot/src/modules/vtol_att_control /home/andrey/Documents/ap/PX4-Autopilot/src/systemcmds/actuator_test /home/andrey/Documents/ap/PX4-Autopilot/src/systemcmds/bsondump /home/andrey/Documents/ap/PX4-Autopilot/src/systemcmds/dmesg /home/andrey/Documents/ap/PX4-Autopilot/src/systemcmds/hardfault_log /home/andrey/Documents/ap/PX4-Autopilot/src/systemcmds/i2cdetect /home/andrey/Documents/ap/PX4-Autopilot/src/systemcmds/led_control /home/andrey/Documents/ap/PX4-Autopilot/src/systemcmds/mft /home/andrey/Documents/ap/PX4-Autopilot/src/systemcmds/mtd /home/andrey/Documents/ap/PX4-Autopilot/src/systemcmds/nshterm /home/andrey/Documents/ap/PX4-Autopilot/src/systemcmds/param /home/andrey/Documents/ap/PX4-Autopilot/src/systemcmds/perf /home/andrey/Documents/ap/PX4-Autopilot/src/systemcmds/reboot /home/andrey/Documents/ap/PX4-Autopilot/src/systemcmds/sd_bench /home/andrey/Documents/ap/PX4-Autopilot/src/systemcmds/system_time /home/andrey/Documents/ap/PX4-Autopilot/src/systemcmds/top /home/andrey/Documents/ap/PX4-Autopilot/src/systemcmds/topic_listener /home/andrey/Documents/ap/PX4-Autopilot/src/systemcmds/tune_control /home/andrey/Documents/ap/PX4-Autopilot/src/systemcmds/uorb /home/andrey/Documents/ap/PX4-Autopilot/src/systemcmds/ver /home/andrey/Documents/ap/PX4-Autopilot/src/systemcmds/work_queue /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/generated_params --xml /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/parameters.xml --json /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/parameters.json --compress --inject-xml /home/andrey/Documents/ap/PX4-Autopilot/src/lib/parameters/parameters_injected.xml --overrides {} --board px4_fmu-v6c
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/lib/parameters && /usr/bin/python3 /home/andrey/Documents/ap/PX4-Autopilot/Tools/validate_json.py --schema-file /home/andrey/Documents/ap/PX4-Autopilot/src/modules/mavlink/mavlink/component_information/parameter.schema.json /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/parameters.json --skip-if-no-schema

parameters.json: parameters.xml
	@$(CMAKE_COMMAND) -E touch_nocreate parameters.json

parameters.json.xz: parameters.xml
	@$(CMAKE_COMMAND) -E touch_nocreate parameters.json.xz

generated_params/serial_params.c: ../../src/lib/battery/module.yaml
generated_params/serial_params.c: ../../src/drivers/distance_sensor/cm8jl65/module.yaml
generated_params/serial_params.c: ../../src/drivers/distance_sensor/leddar_one/module.yaml
generated_params/serial_params.c: ../../src/drivers/distance_sensor/lightware_laser_serial/module.yaml
generated_params/serial_params.c: ../../src/drivers/distance_sensor/tfmini/module.yaml
generated_params/serial_params.c: ../../src/drivers/distance_sensor/ulanding_radar/module.yaml
generated_params/serial_params.c: ../../src/drivers/dshot/module.yaml
generated_params/serial_params.c: ../../src/drivers/gps/module.yaml
generated_params/serial_params.c: ../../src/drivers/ins/vectornav/module.yaml
generated_params/serial_params.c: ../../src/drivers/optical_flow/thoneflow/module.yaml
generated_params/serial_params.c: ../../src/drivers/pwm_out/module.yaml
generated_params/serial_params.c: ../../src/drivers/px4io/module.yaml
generated_params/serial_params.c: ../../src/drivers/telemetry/frsky_telemetry/module.yaml
generated_params/serial_params.c: ../../src/drivers/telemetry/hott/hott_telemetry/module.yaml
generated_params/serial_params.c: ../../src/drivers/uavcan/module.yaml
generated_params/serial_params.c: ../../src/modules/battery_status/module.yaml
generated_params/serial_params.c: ../../src/modules/control_allocator/module.yaml
generated_params/serial_params.c: ../../src/modules/mavlink/module.yaml
generated_params/serial_params.c: ../../src/modules/sensors/module.yaml
generated_params/serial_params.c: ../../src/modules/simulation/pwm_out_sim/module_hil.yaml
generated_params/serial_params.c: ../../src/modules/uxrce_dds_client/module.yaml
generated_params/serial_params.c: ../../Tools/serial/rc.serial.jinja
generated_params/serial_params.c: ../../Tools/serial/rc.serial_port.jinja
generated_params/serial_params.c: ../../Tools/serial/serial_params.c.jinja
generated_params/serial_params.c: ../../Tools/serial/generate_config.py
generated_params/serial_params.c: ../../Tools/module_config/generate_params.py
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Generating serial_params.c"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/lib/parameters && /usr/bin/cmake -E make_directory /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/generated_params
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/lib/parameters && /usr/bin/python3 /home/andrey/Documents/ap/PX4-Autopilot/Tools/serial/generate_config.py --params-file /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/generated_params/serial_params.c --serial-ports GPS1:/dev/ttyS0 GPS2:/dev/ttyS6 TEL1:/dev/ttyS5 TEL2:/dev/ttyS3 TEL3:/dev/ttyS1 --config-files /home/andrey/Documents/ap/PX4-Autopilot/src/lib/battery/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/distance_sensor/cm8jl65/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/distance_sensor/leddar_one/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/distance_sensor/lightware_laser_serial/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/distance_sensor/tfmini/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/distance_sensor/ulanding_radar/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/dshot/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/gps/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/ins/vectornav/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/optical_flow/thoneflow/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/pwm_out/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/px4io/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/telemetry/frsky_telemetry/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/telemetry/hott/hott_telemetry/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/uavcan/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/modules/battery_status/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/modules/control_allocator/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/modules/mavlink/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/modules/sensors/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/modules/simulation/pwm_out_sim/module_hil.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/modules/uxrce_dds_client/module.yaml
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/lib/parameters && /usr/bin/python3 /home/andrey/Documents/ap/PX4-Autopilot/Tools/module_config/generate_params.py --params-file /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/generated_params/module_params.c --board-with-io --timer-config /home/andrey/Documents/ap/PX4-Autopilot/boards/px4/fmu-v6c/src/timer_config.cpp --config-files /home/andrey/Documents/ap/PX4-Autopilot/src/lib/battery/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/distance_sensor/cm8jl65/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/distance_sensor/leddar_one/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/distance_sensor/lightware_laser_serial/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/distance_sensor/tfmini/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/distance_sensor/ulanding_radar/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/dshot/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/gps/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/ins/vectornav/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/optical_flow/thoneflow/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/pwm_out/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/px4io/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/telemetry/frsky_telemetry/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/telemetry/hott/hott_telemetry/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/drivers/uavcan/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/modules/battery_status/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/modules/control_allocator/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/modules/mavlink/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/modules/sensors/module.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/modules/simulation/pwm_out_sim/module_hil.yaml /home/andrey/Documents/ap/PX4-Autopilot/src/modules/uxrce_dds_client/module.yaml --board px4_fmu-v6c

generated_params/module_params.c: generated_params/serial_params.c
	@$(CMAKE_COMMAND) -E touch_nocreate generated_params/module_params.c

parameters_header: src/lib/parameters/CMakeFiles/parameters_header
parameters_header: src/lib/parameters/px4_parameters.hpp
parameters_header: parameters.xml
parameters_header: parameters.json
parameters_header: parameters.json.xz
parameters_header: generated_params/serial_params.c
parameters_header: generated_params/module_params.c
parameters_header: src/lib/parameters/CMakeFiles/parameters_header.dir/build.make

.PHONY : parameters_header

# Rule to build all files generated by this target.
src/lib/parameters/CMakeFiles/parameters_header.dir/build: parameters_header

.PHONY : src/lib/parameters/CMakeFiles/parameters_header.dir/build

src/lib/parameters/CMakeFiles/parameters_header.dir/clean:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/lib/parameters && $(CMAKE_COMMAND) -P CMakeFiles/parameters_header.dir/cmake_clean.cmake
.PHONY : src/lib/parameters/CMakeFiles/parameters_header.dir/clean

src/lib/parameters/CMakeFiles/parameters_header.dir/depend:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/andrey/Documents/ap/PX4-Autopilot /home/andrey/Documents/ap/PX4-Autopilot/src/lib/parameters /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/lib/parameters /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/lib/parameters/CMakeFiles/parameters_header.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/lib/parameters/CMakeFiles/parameters_header.dir/depend

