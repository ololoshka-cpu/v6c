file(REMOVE_RECURSE
  "../etc/extras/px4_io-v2_default.bin"
  "../etc/init.d/rc.autostart"
  "../etc/init.d/rc.autostart.post"
  "../etc/init.d/rc.board_arch_defaults"
  "../etc/init.d/rc.board_defaults"
  "../etc/init.d/rc.board_sensors"
  "../etc/init.d/rc.serial"
  "../romfs_files.tar"
  "CMakeFiles/romfs_gen_files_target"
  "px4_io-v2_default.bin.stamp"
  "rc.board_arch_defaults.stamp"
  "rc.board_defaults.stamp"
  "rc.board_sensors.stamp"
  "romfs_copy.stamp"
  "romfs_extras.stamp"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/romfs_gen_files_target.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
