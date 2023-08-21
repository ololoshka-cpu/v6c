# Install script for directory: /home/andrey/Documents/ap/PX4-Autopilot

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "RelWithDebInfo")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/platforms/posix/src/px4/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/drivers/camera_trigger/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/drivers/gps/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/drivers/osd/msp_osd/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/drivers/tone_alarm/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/modules/airship_att_control/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/modules/airspeed_selector/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/modules/attitude_estimator_q/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/modules/camera_feedback/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/modules/commander/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/modules/control_allocator/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/modules/dataman/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/modules/ekf2/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/modules/events/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/modules/flight_mode_manager/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/modules/fw_att_control/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/modules/fw_autotune_attitude_control/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/modules/fw_pos_control/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/modules/fw_rate_control/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/modules/gimbal/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/modules/gyro_calibration/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/modules/gyro_fft/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/modules/land_detector/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/modules/landing_target_estimator/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/modules/load_mon/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/modules/local_position_estimator/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/modules/logger/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/modules/mag_bias_estimator/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/modules/manual_control/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/modules/mavlink/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/modules/mc_att_control/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/modules/mc_autotune_attitude_control/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/modules/mc_hover_thrust_estimator/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/modules/mc_pos_control/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/modules/mc_rate_control/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/modules/navigator/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/modules/payload_deliverer/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/modules/rc_update/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/modules/replay/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/modules/rover_pos_control/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/modules/sensors/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/modules/simulation/battery_simulator/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/modules/simulation/gz_bridge/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/modules/simulation/pwm_out_sim/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/modules/simulation/sensor_airspeed_sim/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/modules/simulation/sensor_baro_sim/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/modules/simulation/sensor_gps_sim/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/modules/simulation/sensor_mag_sim/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/modules/simulation/simulator_mavlink/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/modules/simulation/simulator_sih/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/modules/temperature_compensation/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/modules/uuv_att_control/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/modules/uuv_pos_control/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/modules/uxrce_dds_client/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/modules/vtol_att_control/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/systemcmds/actuator_test/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/systemcmds/bsondump/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/systemcmds/dyn/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/systemcmds/failure/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/systemcmds/led_control/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/systemcmds/param/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/systemcmds/perf/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/systemcmds/sd_bench/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/systemcmds/shutdown/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/systemcmds/system_time/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/systemcmds/tests/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/systemcmds/topic_listener/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/systemcmds/tune_control/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/systemcmds/uorb/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/systemcmds/ver/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/systemcmds/work_queue/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/examples/dyn_hello/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/examples/fake_gps/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/examples/fake_imu/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/examples/fake_magnetometer/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/examples/hello/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/examples/px4_mavlink_debug/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/examples/px4_simple_app/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/src/examples/work_item/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/boards/px4/sitl/src/cmake_install.cmake")
  include("/home/andrey/Documents/ap/PX4-Autopilot/build/platforms/posix/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/andrey/Documents/ap/PX4-Autopilot/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
