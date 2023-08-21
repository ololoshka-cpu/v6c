
if(NOT "/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr-stamp/microcdr-gitinfo.txt" IS_NEWER_THAN "/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr-stamp/microcdr-gitclone-lastrun.txt")
  message(STATUS "Avoiding repeated git clone, stamp file is up to date: '/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr-stamp/microcdr-gitclone-lastrun.txt'")
  return()
endif()

execute_process(
  COMMAND ${CMAKE_COMMAND} -E remove_directory "/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to remove directory: '/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr'")
endif()

# try the clone 3 times in case there is an odd git clone issue
set(error_code 1)
set(number_of_tries 0)
while(error_code AND number_of_tries LESS 3)
  execute_process(
    COMMAND "/usr/bin/git"  clone --no-checkout "https://github.com/eProsima/Micro-CDR.git" "microcdr"
    WORKING_DIRECTORY "/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src"
    RESULT_VARIABLE error_code
    )
  math(EXPR number_of_tries "${number_of_tries} + 1")
endwhile()
if(number_of_tries GREATER 1)
  message(STATUS "Had to git clone more than once:
          ${number_of_tries} times.")
endif()
if(error_code)
  message(FATAL_ERROR "Failed to clone repository: 'https://github.com/eProsima/Micro-CDR.git'")
endif()

execute_process(
  COMMAND "/usr/bin/git"  checkout v2.0.1 --
  WORKING_DIRECTORY "/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to checkout tag: 'v2.0.1'")
endif()

set(init_submodules TRUE)
if(init_submodules)
  execute_process(
    COMMAND "/usr/bin/git"  submodule update --recursive --init 
    WORKING_DIRECTORY "/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr"
    RESULT_VARIABLE error_code
    )
endif()
if(error_code)
  message(FATAL_ERROR "Failed to update submodules in: '/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr'")
endif()

# Complete success, update the script-last-run stamp file:
#
execute_process(
  COMMAND ${CMAKE_COMMAND} -E copy
    "/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr-stamp/microcdr-gitinfo.txt"
    "/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr-stamp/microcdr-gitclone-lastrun.txt"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to copy script-last-run stamp file: '/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr-stamp/microcdr-gitclone-lastrun.txt'")
endif()

