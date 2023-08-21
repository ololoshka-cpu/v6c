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
include platforms/nuttx/CMakeFiles/px4.dir/depend.make

# Include the progress variables for this target.
include platforms/nuttx/CMakeFiles/px4.dir/progress.make

# Include the compile flags for this target's objects.
include platforms/nuttx/CMakeFiles/px4.dir/flags.make

platforms/nuttx/CMakeFiles/px4.dir/__/common/empty.c.obj: platforms/nuttx/CMakeFiles/px4.dir/flags.make
platforms/nuttx/CMakeFiles/px4.dir/__/common/empty.c.obj: ../../platforms/common/empty.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object platforms/nuttx/CMakeFiles/px4.dir/__/common/empty.c.obj"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/platforms/nuttx && /usr/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/px4.dir/__/common/empty.c.obj   -c /home/andrey/Documents/ap/PX4-Autopilot/platforms/common/empty.c

platforms/nuttx/CMakeFiles/px4.dir/__/common/empty.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/px4.dir/__/common/empty.c.i"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/platforms/nuttx && /usr/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/andrey/Documents/ap/PX4-Autopilot/platforms/common/empty.c > CMakeFiles/px4.dir/__/common/empty.c.i

platforms/nuttx/CMakeFiles/px4.dir/__/common/empty.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/px4.dir/__/common/empty.c.s"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/platforms/nuttx && /usr/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/andrey/Documents/ap/PX4-Autopilot/platforms/common/empty.c -o CMakeFiles/px4.dir/__/common/empty.c.s

# Object files for target px4
px4_OBJECTS = \
"CMakeFiles/px4.dir/__/common/empty.c.obj"

# External object files for target px4
px4_EXTERNAL_OBJECTS =

px4_fmu-v6c_default.elf: platforms/nuttx/CMakeFiles/px4.dir/__/common/empty.c.obj
px4_fmu-v6c_default.elf: platforms/nuttx/CMakeFiles/px4.dir/build.make
px4_fmu-v6c_default.elf: NuttX/nuttx/boards/libboards.a
px4_fmu-v6c_default.elf: NuttX/nuttx/drivers/libdrivers.a
px4_fmu-v6c_default.elf: NuttX/nuttx/fs/libfs.a
px4_fmu-v6c_default.elf: NuttX/nuttx/sched/libsched.a
px4_fmu-v6c_default.elf: NuttX/nuttx/crypto/libcrypto.a
px4_fmu-v6c_default.elf: NuttX/nuttx/binfmt/libbinfmt.a
px4_fmu-v6c_default.elf: NuttX/nuttx/libs/libxx/libxx.a
px4_fmu-v6c_default.elf: NuttX/apps/libapps.a
px4_fmu-v6c_default.elf: NuttX/nuttx/arch/arm/src/libarch.a
px4_fmu-v6c_default.elf: NuttX/nuttx/mm/libmm.a
px4_fmu-v6c_default.elf: NuttX/nuttx/libs/libc/libc.a
px4_fmu-v6c_default.elf: src/drivers/adc/board_adc/libdrivers__board_adc.a
px4_fmu-v6c_default.elf: src/drivers/barometer/ms5611/libdrivers__barometer__ms5611.a
px4_fmu-v6c_default.elf: src/drivers/batt_smbus/libdrivers__batt_smbus.a
px4_fmu-v6c_default.elf: src/drivers/camera_capture/libdrivers__camera_capture.a
px4_fmu-v6c_default.elf: src/drivers/camera_trigger/libdrivers__camera_trigger.a
px4_fmu-v6c_default.elf: src/drivers/differential_pressure/ms4525do/libdrivers__differential_pressure__ms4525do.a
px4_fmu-v6c_default.elf: src/drivers/differential_pressure/ms5525dso/libdrivers__differential_pressure__ms5525dso.a
px4_fmu-v6c_default.elf: src/drivers/differential_pressure/sdp3x/libdrivers__differential_pressure__sdp3x.a
px4_fmu-v6c_default.elf: src/drivers/distance_sensor/cm8jl65/libdrivers__cm8jl65.a
px4_fmu-v6c_default.elf: src/drivers/distance_sensor/gy_us42/libdrivers__distance_sensor__gy_us42.a
px4_fmu-v6c_default.elf: src/drivers/distance_sensor/leddar_one/libdrivers__distance_sensor__leddar_one.a
px4_fmu-v6c_default.elf: src/drivers/distance_sensor/lightware_laser_i2c/libdrivers__distance_sensor__lightware_laser_i2c.a
px4_fmu-v6c_default.elf: src/drivers/distance_sensor/lightware_laser_serial/libdrivers__distance_sensor__lightware_laser_serial.a
px4_fmu-v6c_default.elf: src/drivers/distance_sensor/ll40ls/libdrivers__ll40ls.a
px4_fmu-v6c_default.elf: src/drivers/distance_sensor/ll40ls_pwm/libdrivers__ll40ls_pwm.a
px4_fmu-v6c_default.elf: src/drivers/distance_sensor/mappydot/libdrivers__mappydot.a
px4_fmu-v6c_default.elf: src/drivers/distance_sensor/mb12xx/libdrivers__mb12xx.a
px4_fmu-v6c_default.elf: src/drivers/distance_sensor/pga460/libdrivers__pga460.a
px4_fmu-v6c_default.elf: src/drivers/distance_sensor/srf02/libdrivers__distance_sensor__srf02.a
px4_fmu-v6c_default.elf: src/drivers/distance_sensor/teraranger/libdrivers__distance_sensor__teraranger.a
px4_fmu-v6c_default.elf: src/drivers/distance_sensor/tf02pro/libdrivers__distance_sensor__tf02pro.a
px4_fmu-v6c_default.elf: src/drivers/distance_sensor/tfmini/libdrivers__tfmini.a
px4_fmu-v6c_default.elf: src/drivers/distance_sensor/ulanding_radar/libdrivers__distance_sensor__ulanding_radar.a
px4_fmu-v6c_default.elf: src/drivers/distance_sensor/vl53l0x/libdrivers__distance_sensor__vl53l0x.a
px4_fmu-v6c_default.elf: src/drivers/distance_sensor/vl53l1x/libdrivers__distance_sensor__vl53l1x.a
px4_fmu-v6c_default.elf: src/drivers/dshot/libdrivers__dshot.a
px4_fmu-v6c_default.elf: src/drivers/gps/libdrivers__gps.a
px4_fmu-v6c_default.elf: src/drivers/heater/libdrivers__heater.a
px4_fmu-v6c_default.elf: src/drivers/imu/bosch/bmi055/libdrivers__imu__bosch__bmi055.a
px4_fmu-v6c_default.elf: src/drivers/imu/invensense/icm42688p/libdrivers__imu__invensense__icm42688p.a
px4_fmu-v6c_default.elf: src/drivers/ins/vectornav/libdrivers__ins__vectornav.a
px4_fmu-v6c_default.elf: src/drivers/lights/rgbled/libdrivers__rgbled.a
px4_fmu-v6c_default.elf: src/drivers/lights/rgbled_is31fl3195/libdrivers__rgbled_is31fl3195.a
px4_fmu-v6c_default.elf: src/drivers/lights/rgbled_lp5562/libdrivers__rgbled_lp5562.a
px4_fmu-v6c_default.elf: src/drivers/lights/rgbled_ncp5623c/libdrivers__rgbled_ncp5623c.a
px4_fmu-v6c_default.elf: src/drivers/magnetometer/akm/ak09916/libdrivers__magnetometer__akm__ak09916.a
px4_fmu-v6c_default.elf: src/drivers/magnetometer/akm/ak8963/libdrivers__magnetometer__akm__ak8963.a
px4_fmu-v6c_default.elf: src/drivers/magnetometer/bosch/bmm150/libdrivers__magnetometer__bosch__bmm150.a
px4_fmu-v6c_default.elf: src/drivers/magnetometer/hmc5883/libdrivers__hmc5883.a
px4_fmu-v6c_default.elf: src/drivers/magnetometer/isentek/ist8308/libdrivers__magnetometer__isentek__ist8308.a
px4_fmu-v6c_default.elf: src/drivers/magnetometer/isentek/ist8310/libdrivers__magnetometer__isentek__ist8310.a
px4_fmu-v6c_default.elf: src/drivers/magnetometer/lis2mdl/libdrivers__lis2mdl.a
px4_fmu-v6c_default.elf: src/drivers/magnetometer/lis3mdl/libdrivers__magnetometer__lis3mdl.a
px4_fmu-v6c_default.elf: src/drivers/magnetometer/lsm303agr/libdrivers__magnetometer__lsm303agr.a
px4_fmu-v6c_default.elf: src/drivers/magnetometer/qmc5883l/libdrivers__magnetometer__qmc5883l.a
px4_fmu-v6c_default.elf: src/drivers/magnetometer/rm3100/libdrivers__rm3100.a
px4_fmu-v6c_default.elf: src/drivers/magnetometer/vtrantech/vcm1193l/libdrivers__magnetometer__vcm1193l.a
px4_fmu-v6c_default.elf: src/drivers/optical_flow/paa3905/libdrivers__optical_flow__paa3905.a
px4_fmu-v6c_default.elf: src/drivers/optical_flow/paw3902/libdrivers__optical_flow__paw3902.a
px4_fmu-v6c_default.elf: src/drivers/optical_flow/pmw3901/libdrivers__optical_flow__pmw3901.a
px4_fmu-v6c_default.elf: src/drivers/optical_flow/px4flow/libdrivers__px4flow.a
px4_fmu-v6c_default.elf: src/drivers/optical_flow/thoneflow/libdrivers__thoneflow.a
px4_fmu-v6c_default.elf: src/drivers/power_monitor/ina226/libdrivers__ina226.a
px4_fmu-v6c_default.elf: src/drivers/power_monitor/ina228/libdrivers__ina228.a
px4_fmu-v6c_default.elf: src/drivers/power_monitor/ina238/libdrivers__ina238.a
px4_fmu-v6c_default.elf: src/drivers/pwm_out/libdrivers__pwm_out.a
px4_fmu-v6c_default.elf: src/drivers/px4io/libdrivers__px4io.a
px4_fmu-v6c_default.elf: src/drivers/telemetry/bst/libdrivers__bst.a
px4_fmu-v6c_default.elf: src/drivers/telemetry/frsky_telemetry/libdrivers__frsky_telemetry.a
px4_fmu-v6c_default.elf: src/drivers/telemetry/hott/hott_sensors/libdrivers__hott__hott_sensors.a
px4_fmu-v6c_default.elf: src/drivers/telemetry/hott/hott_telemetry/libdrivers__hott__hott_telemetry.a
px4_fmu-v6c_default.elf: src/drivers/tone_alarm/libdrivers__tone_alarm.a
px4_fmu-v6c_default.elf: src/drivers/uavcan/libdrivers__uavcan.a
px4_fmu-v6c_default.elf: src/modules/airspeed_selector/libmodules__airspeed_selector.a
px4_fmu-v6c_default.elf: src/modules/battery_status/libmodules__battery_status.a
px4_fmu-v6c_default.elf: src/modules/camera_feedback/libmodules__camera_feedback.a
px4_fmu-v6c_default.elf: src/modules/commander/libmodules__commander.a
px4_fmu-v6c_default.elf: src/modules/control_allocator/libmodules__control_allocator.a
px4_fmu-v6c_default.elf: src/modules/dataman/libmodules__dataman.a
px4_fmu-v6c_default.elf: src/modules/ekf2/libmodules__ekf2.a
px4_fmu-v6c_default.elf: src/modules/esc_battery/libmodules__esc_battery.a
px4_fmu-v6c_default.elf: src/modules/events/libmodules__events.a
px4_fmu-v6c_default.elf: src/modules/flight_mode_manager/libmodules__flight_mode_manager.a
px4_fmu-v6c_default.elf: src/modules/fw_att_control/libmodules__fw_att_control.a
px4_fmu-v6c_default.elf: src/modules/fw_autotune_attitude_control/libfw_autotune_attitude_control.a
px4_fmu-v6c_default.elf: src/modules/fw_pos_control/libmodules__fw_pos_control.a
px4_fmu-v6c_default.elf: src/modules/fw_rate_control/libmodules__fw_rate_control.a
px4_fmu-v6c_default.elf: src/modules/gimbal/libdrivers__gimbal.a
px4_fmu-v6c_default.elf: src/modules/gyro_calibration/libmodules__gyro_calibration.a
px4_fmu-v6c_default.elf: src/modules/gyro_fft/libmodules__gyro_fft.a
px4_fmu-v6c_default.elf: src/modules/land_detector/libmodules__land_detector.a
px4_fmu-v6c_default.elf: src/modules/landing_target_estimator/libmodules__landing_target_estimator.a
px4_fmu-v6c_default.elf: src/modules/load_mon/libmodules__load_mon.a
px4_fmu-v6c_default.elf: src/modules/logger/libmodules__logger.a
px4_fmu-v6c_default.elf: src/modules/mag_bias_estimator/libmodules__mag_bias_estimator.a
px4_fmu-v6c_default.elf: src/modules/manual_control/libmodules__manual_control.a
px4_fmu-v6c_default.elf: src/modules/mavlink/libmodules__mavlink.a
px4_fmu-v6c_default.elf: src/modules/mc_att_control/libmodules__mc_att_control.a
px4_fmu-v6c_default.elf: src/modules/mc_autotune_attitude_control/libmc_autotune_attitude_control.a
px4_fmu-v6c_default.elf: src/modules/mc_hover_thrust_estimator/libmodules__mc_hover_thrust_estimator.a
px4_fmu-v6c_default.elf: src/modules/mc_pos_control/libmodules__mc_pos_control.a
px4_fmu-v6c_default.elf: src/modules/mc_rate_control/libmodules__mc_rate_control.a
px4_fmu-v6c_default.elf: src/modules/navigator/libmodules__navigator.a
px4_fmu-v6c_default.elf: src/modules/rc_update/libmodules__rc_update.a
px4_fmu-v6c_default.elf: src/modules/rover_pos_control/libmodules__rover_pos_control.a
px4_fmu-v6c_default.elf: src/modules/sensors/libmodules__sensors.a
px4_fmu-v6c_default.elf: src/modules/simulation/pwm_out_sim/libmodules__simulation__pwm_out_sim.a
px4_fmu-v6c_default.elf: src/modules/simulation/sensor_baro_sim/libmodules__simulation__sensor_baro_sim.a
px4_fmu-v6c_default.elf: src/modules/simulation/sensor_gps_sim/libmodules__simulation__sensor_gps_sim.a
px4_fmu-v6c_default.elf: src/modules/simulation/sensor_mag_sim/libmodules__simulation__senosr_mag_sim.a
px4_fmu-v6c_default.elf: src/modules/simulation/simulator_sih/libmodules__simulation__simulator_sih.a
px4_fmu-v6c_default.elf: src/modules/temperature_compensation/libmodules__temperature_compensation.a
px4_fmu-v6c_default.elf: src/modules/uxrce_dds_client/libmodules__uxrce_dds_client.a
px4_fmu-v6c_default.elf: src/modules/vtol_att_control/libmodules__vtol_att_control.a
px4_fmu-v6c_default.elf: src/systemcmds/actuator_test/libsystemcmds__actuator_test.a
px4_fmu-v6c_default.elf: src/systemcmds/bsondump/libsystemcmds__bsondump.a
px4_fmu-v6c_default.elf: src/systemcmds/dmesg/libsystemcmds__dmesg.a
px4_fmu-v6c_default.elf: src/systemcmds/hardfault_log/libsystemcmds__hardfault_log.a
px4_fmu-v6c_default.elf: src/systemcmds/i2cdetect/libsystemcmds__i2cdetect.a
px4_fmu-v6c_default.elf: src/systemcmds/led_control/libsystemcmds__led_control.a
px4_fmu-v6c_default.elf: src/systemcmds/mft/libsystemcmds__mft.a
px4_fmu-v6c_default.elf: src/systemcmds/mtd/libsystemcmds__mtd.a
px4_fmu-v6c_default.elf: src/systemcmds/nshterm/libsystemcmds__nshterm.a
px4_fmu-v6c_default.elf: src/systemcmds/param/libsystemcmds__param.a
px4_fmu-v6c_default.elf: src/systemcmds/perf/libsystemcmds__perf.a
px4_fmu-v6c_default.elf: src/systemcmds/reboot/libsystemcmds__reboot.a
px4_fmu-v6c_default.elf: src/systemcmds/sd_bench/libsystemcmds__sd_bench.a
px4_fmu-v6c_default.elf: src/systemcmds/system_time/libsystemcmds__system_time.a
px4_fmu-v6c_default.elf: src/systemcmds/top/libsystemcmds__top.a
px4_fmu-v6c_default.elf: src/systemcmds/topic_listener/libsystemcmds__topic_listener.a
px4_fmu-v6c_default.elf: src/systemcmds/tune_control/libsystemcmds__tune_control.a
px4_fmu-v6c_default.elf: src/systemcmds/uorb/libsystemcmds__uorb.a
px4_fmu-v6c_default.elf: src/systemcmds/ver/libsystemcmds__ver.a
px4_fmu-v6c_default.elf: src/systemcmds/work_queue/libsystemcmds__work_queue.a
px4_fmu-v6c_default.elf: ROMFS/libromfs.a
px4_fmu-v6c_default.elf: src/lib/drivers/smbus/libdrivers__smbus.a
px4_fmu-v6c_default.elf: platforms/nuttx/src/px4/stm/stm32h7/dshot/libarch_dshot.a
px4_fmu-v6c_default.elf: src/drivers/ins/vectornav/libvnc/liblibvnc.a
px4_fmu-v6c_default.elf: platforms/nuttx/src/px4/stm/stm32h7/io_pins/libarch_io_pins.a
px4_fmu-v6c_default.elf: platforms/nuttx/src/px4/stm/stm32h7/px4io_serial/libarch_px4io_serial.a
px4_fmu-v6c_default.elf: src/drivers/telemetry/hott/libdrivers__hott.a
px4_fmu-v6c_default.elf: platforms/nuttx/src/px4/stm/stm32h7/tone_alarm/libarch_tone_alarm.a
px4_fmu-v6c_default.elf: src/lib/drivers/rangefinder/libdrivers_rangefinder.a
px4_fmu-v6c_default.elf: src/lib/led/libled.a
px4_fmu-v6c_default.elf: src/lib/button/libbutton_publisher.a
px4_fmu-v6c_default.elf: src/drivers/uavcan/libuavcan_drivers/libuavcan_stm32h7_driver.a
px4_fmu-v6c_default.elf: src/drivers/uavcan/libuavcan/libuavcan/libuavcan.a
px4_fmu-v6c_default.elf: src/lib/wind_estimator/libwind_estimator.a
px4_fmu-v6c_default.elf: src/modules/commander/failure_detector/libfailure_detector.a
px4_fmu-v6c_default.elf: src/modules/commander/HealthAndArmingChecks/libhealth_and_arming_checks.a
px4_fmu-v6c_default.elf: src/modules/commander/Arming/ArmAuthorization/libArmAuthorization.a
px4_fmu-v6c_default.elf: src/modules/commander/ModeUtil/libmode_util.a
px4_fmu-v6c_default.elf: src/modules/commander/failsafe/libfailsafe.a
px4_fmu-v6c_default.elf: src/modules/control_allocator/ActuatorEffectiveness/libActuatorEffectiveness.a
px4_fmu-v6c_default.elf: src/modules/control_allocator/ControlAllocation/libControlAllocation.a
px4_fmu-v6c_default.elf: src/lib/world_magnetic_model/libworld_magnetic_model.a
px4_fmu-v6c_default.elf: src/modules/ekf2/Utility/libEKF2Utility.a
px4_fmu-v6c_default.elf: src/lib/battery/libbattery.a
px4_fmu-v6c_default.elf: src/modules/flight_mode_manager/tasks/Descend/libFlightTaskDescend.a
px4_fmu-v6c_default.elf: src/modules/flight_mode_manager/tasks/Failsafe/libFlightTaskFailsafe.a
px4_fmu-v6c_default.elf: src/modules/flight_mode_manager/tasks/ManualAcceleration/libFlightTaskManualAcceleration.a
px4_fmu-v6c_default.elf: src/modules/flight_mode_manager/tasks/ManualPositionSmoothVel/libFlightTaskManualPositionSmoothVel.a
px4_fmu-v6c_default.elf: src/modules/flight_mode_manager/tasks/ManualPosition/libFlightTaskManualPosition.a
px4_fmu-v6c_default.elf: src/lib/collision_prevention/libCollisionPrevention.a
px4_fmu-v6c_default.elf: src/modules/flight_mode_manager/tasks/Transition/libFlightTaskTransition.a
px4_fmu-v6c_default.elf: src/modules/flight_mode_manager/tasks/AutoFollowTarget/libFlightTaskAutoFollowTarget.a
px4_fmu-v6c_default.elf: src/modules/flight_mode_manager/tasks/Auto/libFlightTaskAuto.a
px4_fmu-v6c_default.elf: src/lib/weather_vane/libWeatherVane.a
px4_fmu-v6c_default.elf: src/lib/avoidance/libavoidance.a
px4_fmu-v6c_default.elf: src/modules/flight_mode_manager/tasks/AutoFollowTarget/follow_target_estimator/libfollow_target_estimator.a
px4_fmu-v6c_default.elf: src/modules/flight_mode_manager/tasks/Orbit/libFlightTaskOrbit.a
px4_fmu-v6c_default.elf: src/modules/flight_mode_manager/tasks/ManualAltitudeSmoothVel/libFlightTaskManualAltitudeSmoothVel.a
px4_fmu-v6c_default.elf: src/modules/flight_mode_manager/tasks/ManualAltitude/libFlightTaskManualAltitude.a
px4_fmu-v6c_default.elf: src/modules/flight_mode_manager/tasks/Utility/libFlightTaskUtility.a
px4_fmu-v6c_default.elf: src/modules/flight_mode_manager/tasks/FlightTask/libFlightTask.a
px4_fmu-v6c_default.elf: src/lib/bezier/libbezier.a
px4_fmu-v6c_default.elf: src/modules/fw_pos_control/launchdetection/liblaunchdetection.a
px4_fmu-v6c_default.elf: src/lib/npfg/libnpfg.a
px4_fmu-v6c_default.elf: src/modules/fw_pos_control/runway_takeoff/librunway_takeoff.a
px4_fmu-v6c_default.elf: src/lib/tecs/libtecs.a
px4_fmu-v6c_default.elf: src/modules/mc_att_control/AttitudeControl/libAttitudeControl.a
px4_fmu-v6c_default.elf: src/lib/system_identification/libSystemIdentification.a
px4_fmu-v6c_default.elf: src/modules/mc_hover_thrust_estimator/libzero_order_hover_thrust_ekf.a
px4_fmu-v6c_default.elf: src/lib/slew_rate/libSlewRate.a
px4_fmu-v6c_default.elf: src/modules/mc_pos_control/PositionControl/libPositionControl.a
px4_fmu-v6c_default.elf: src/modules/mc_pos_control/Takeoff/libTakeoff.a
px4_fmu-v6c_default.elf: src/lib/controllib/libcontrollib.a
px4_fmu-v6c_default.elf: src/lib/rate_control/libRateControl.a
px4_fmu-v6c_default.elf: src/modules/navigator/MissionFeasibility/libmission_feasibility_checker.a
px4_fmu-v6c_default.elf: src/modules/navigator/libmodules__navigator.a
px4_fmu-v6c_default.elf: src/modules/navigator/MissionFeasibility/libmission_feasibility_checker.a
px4_fmu-v6c_default.elf: src/lib/motion_planning/libmotion_planning.a
px4_fmu-v6c_default.elf: src/lib/adsb/libadsb.a
px4_fmu-v6c_default.elf: src/lib/dataman_client/libdataman_client.a
px4_fmu-v6c_default.elf: src/modules/navigator/GeofenceBreachAvoidance/libgeofence_breach_avoidance.a
px4_fmu-v6c_default.elf: src/modules/dataman/libmodules__dataman.a
px4_fmu-v6c_default.elf: src/lib/hysteresis/libhysteresis.a
px4_fmu-v6c_default.elf: src/lib/l1/libl1.a
px4_fmu-v6c_default.elf: src/lib/pid/libpid.a
px4_fmu-v6c_default.elf: src/lib/airspeed/libairspeed.a
px4_fmu-v6c_default.elf: src/modules/sensors/vehicle_imu/libvehicle_imu.a
px4_fmu-v6c_default.elf: src/modules/sensors/vehicle_acceleration/libvehicle_acceleration.a
px4_fmu-v6c_default.elf: src/modules/sensors/vehicle_air_data/libvehicle_air_data.a
px4_fmu-v6c_default.elf: src/modules/sensors/vehicle_angular_velocity/libvehicle_angular_velocity.a
px4_fmu-v6c_default.elf: src/modules/sensors/vehicle_gps_position/libvehicle_gps_position.a
px4_fmu-v6c_default.elf: src/modules/sensors/vehicle_magnetometer/libvehicle_magnetometer.a
px4_fmu-v6c_default.elf: src/lib/sensor_calibration/libsensor_calibration.a
px4_fmu-v6c_default.elf: src/modules/sensors/data_validator/libdata_validator.a
px4_fmu-v6c_default.elf: src/modules/sensors/vehicle_optical_flow/libvehicle_optical_flow.a
px4_fmu-v6c_default.elf: src/lib/mixer_module/libmixer_module.a
px4_fmu-v6c_default.elf: src/lib/drivers/magnetometer/libdrivers_magnetometer.a
px4_fmu-v6c_default.elf: src/lib/geo/libgeo.a
px4_fmu-v6c_default.elf: src/lib/drivers/accelerometer/libdrivers_accelerometer.a
px4_fmu-v6c_default.elf: src/lib/drivers/gyroscope/libdrivers_gyroscope.a
px4_fmu-v6c_default.elf: src/lib/conversion/libconversion.a
px4_fmu-v6c_default.elf: src/lib/mathlib/libmathlib.a
px4_fmu-v6c_default.elf: src/lib/timesync/libtimesync.a
px4_fmu-v6c_default.elf: src/modules/uxrce_dds_client/lib/libmicroxrcedds_client.a
px4_fmu-v6c_default.elf: src/modules/uxrce_dds_client/lib/libmicrocdr.a
px4_fmu-v6c_default.elf: src/lib/tunes/libtunes.a
px4_fmu-v6c_default.elf: src/lib/circuit_breaker/libcircuit_breaker.a
px4_fmu-v6c_default.elf: src/lib/version/libversion.a
px4_fmu-v6c_default.elf: src/lib/systemlib/libsystemlib.a
px4_fmu-v6c_default.elf: src/lib/parameters/libparameters.a
px4_fmu-v6c_default.elf: src/lib/perf/libperf.a
px4_fmu-v6c_default.elf: src/lib/tinybson/libtinybson.a
px4_fmu-v6c_default.elf: src/lib/parameters/flashparams/libflashparams.a
px4_fmu-v6c_default.elf: NuttX/nuttx/arch/arm/src/libarch.a
px4_fmu-v6c_default.elf: boards/px4/fmu-v6c/src/libdrivers_board.a
px4_fmu-v6c_default.elf: platforms/nuttx/src/px4/stm/stm32h7/board_reset/libarch_board_reset.a
px4_fmu-v6c_default.elf: platforms/nuttx/src/px4/common/libpx4_layer.a
px4_fmu-v6c_default.elf: src/lib/drivers/device/libdrivers__device.a
px4_fmu-v6c_default.elf: platforms/nuttx/src/px4/stm/stm32h7/spi/libarch_spi.a
px4_fmu-v6c_default.elf: src/lib/drivers/led/libdrivers__led.a
px4_fmu-v6c_default.elf: NuttX/nuttx/arch/arm/src/libarch.a
px4_fmu-v6c_default.elf: boards/px4/fmu-v6c/src/libdrivers_board.a
px4_fmu-v6c_default.elf: platforms/nuttx/src/px4/stm/stm32h7/board_reset/libarch_board_reset.a
px4_fmu-v6c_default.elf: platforms/nuttx/src/px4/common/libpx4_layer.a
px4_fmu-v6c_default.elf: src/lib/drivers/device/libdrivers__device.a
px4_fmu-v6c_default.elf: platforms/nuttx/src/px4/stm/stm32h7/spi/libarch_spi.a
px4_fmu-v6c_default.elf: src/lib/drivers/led/libdrivers__led.a
px4_fmu-v6c_default.elf: platforms/nuttx/src/px4/stm/stm32h7/hrt/libarch_hrt.a
px4_fmu-v6c_default.elf: platforms/nuttx/src/px4/stm/stm32h7/board_hw_info/libarch_board_hw_info.a
px4_fmu-v6c_default.elf: platforms/nuttx/src/px4/stm/stm32h7/adc/libarch_adc.a
px4_fmu-v6c_default.elf: src/lib/crc/libcrc.a
px4_fmu-v6c_default.elf: platforms/common/libpx4_platform.a
px4_fmu-v6c_default.elf: platforms/common/px4_work_queue/libpx4_work_queue.a
px4_fmu-v6c_default.elf: platforms/nuttx/src/px4/stm/stm32h7/board_critmon/libarch_board_critmon.a
px4_fmu-v6c_default.elf: platforms/nuttx/src/px4/stm/stm32h7/version/libarch_version.a
px4_fmu-v6c_default.elf: platforms/common/uORB/libuORB.a
px4_fmu-v6c_default.elf: NuttX/nuttx/mm/libmm.a
px4_fmu-v6c_default.elf: src/lib/cdev/libcdev.a
px4_fmu-v6c_default.elf: NuttX/nuttx/fs/libfs.a
px4_fmu-v6c_default.elf: NuttX/nuttx/libs/libxx/libxx.a
px4_fmu-v6c_default.elf: NuttX/nuttx/drivers/libdrivers.a
px4_fmu-v6c_default.elf: NuttX/nuttx/libs/libc/libc.a
px4_fmu-v6c_default.elf: NuttX/nuttx/drivers/libdrivers.a
px4_fmu-v6c_default.elf: NuttX/nuttx/libs/libc/libc.a
px4_fmu-v6c_default.elf: NuttX/nuttx/sched/libsched.a
px4_fmu-v6c_default.elf: msg/libuorb_msgs.a
px4_fmu-v6c_default.elf: platforms/nuttx/CMakeFiles/px4.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../px4_fmu-v6c_default.elf"
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/platforms/nuttx && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/px4.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
platforms/nuttx/CMakeFiles/px4.dir/build: px4_fmu-v6c_default.elf

.PHONY : platforms/nuttx/CMakeFiles/px4.dir/build

platforms/nuttx/CMakeFiles/px4.dir/clean:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/platforms/nuttx && $(CMAKE_COMMAND) -P CMakeFiles/px4.dir/cmake_clean.cmake
.PHONY : platforms/nuttx/CMakeFiles/px4.dir/clean

platforms/nuttx/CMakeFiles/px4.dir/depend:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/andrey/Documents/ap/PX4-Autopilot /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/platforms/nuttx /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/platforms/nuttx/CMakeFiles/px4.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : platforms/nuttx/CMakeFiles/px4.dir/depend

