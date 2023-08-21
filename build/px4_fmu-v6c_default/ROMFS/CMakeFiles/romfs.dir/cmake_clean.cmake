file(REMOVE_RECURSE
  "../etc/extras/px4_io-v2_default.bin"
  "../etc/init.d/rc.autostart"
  "../etc/init.d/rc.autostart.post"
  "../etc/init.d/rc.board_arch_defaults"
  "../etc/init.d/rc.board_defaults"
  "../etc/init.d/rc.board_sensors"
  "../etc/init.d/rc.serial"
  "../gencromfs"
  "../romfs_files.tar"
  "CMakeFiles/romfs.dir/nsh_romfsimg.c.obj"
  "libromfs.a"
  "libromfs.pdb"
  "nsh_romfsimg.c"
  "px4_io-v2_default.bin.stamp"
  "rc.board_arch_defaults.stamp"
  "rc.board_defaults.stamp"
  "rc.board_sensors.stamp"
  "romfs_copy.stamp"
  "romfs_extras.stamp"
  "romfs_pruned.stamp"
)

# Per-language clean rules from dependency scanning.
foreach(lang C)
  include(CMakeFiles/romfs.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
