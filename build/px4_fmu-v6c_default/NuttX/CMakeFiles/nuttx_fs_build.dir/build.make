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

# Utility rule file for nuttx_fs_build.

# Include the progress variables for this target.
include NuttX/CMakeFiles/nuttx_fs_build.dir/progress.make

NuttX/CMakeFiles/nuttx_fs_build: NuttX/nuttx/fs/libfs.a


NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/aio/aio.h
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/aio/aio_cancel.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/aio/aio_fsync.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/aio/aio_initialize.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/aio/aio_queue.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/aio/aio_read.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/aio/aio_signal.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/aio/aio_write.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/aio/aioc_contain.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/binfs/fs_binfs.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/cromfs/cromfs.h
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/cromfs/fs_cromfs.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/driver/driver.h
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/driver/fs_blockpartition.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/driver/fs_blockproxy.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/driver/fs_closeblockdriver.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/driver/fs_findblockdriver.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/driver/fs_findmtddriver.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/driver/fs_mtdpartition.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/driver/fs_mtdproxy.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/driver/fs_openblockdriver.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/driver/fs_registerblockdriver.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/driver/fs_registerdriver.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/driver/fs_registermtddriver.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/driver/fs_unregisterblockdriver.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/driver/fs_unregisterdriver.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/driver/fs_unregistermtddriver.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/fat/fs_fat32.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/fat/fs_fat32.h
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/fat/fs_fat32attrib.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/fat/fs_fat32dirent.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/fat/fs_fat32util.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/fs_initialize.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/hostfs/hostfs.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/hostfs/hostfs.h
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/inode/fs_files.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/inode/fs_foreachinode.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/inode/fs_inode.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/inode/fs_inodeaddref.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/inode/fs_inodebasename.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/inode/fs_inodefind.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/inode/fs_inodefree.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/inode/fs_inodegetpath.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/inode/fs_inoderelease.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/inode/fs_inoderemove.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/inode/fs_inodereserve.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/inode/fs_inodesearch.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/inode/inode.h
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/littlefs/lfs_vfs.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/littlefs/lfs_vfs.h
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/mmap/fs_mmap.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/mmap/fs_mmisc.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/mmap/fs_munmap.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/mmap/fs_rammap.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/mmap/fs_rammap.h
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/mount/fs_automount.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/mount/fs_foreachmountpoint.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/mount/fs_gettype.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/mount/fs_mount.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/mount/fs_procfs_mount.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/mount/fs_umount2.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/mount/mount.h
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/mqueue/mq_close.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/mqueue/mq_open.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/mqueue/mq_unlink.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/mqueue/mqueue.h
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/nfs/nfs.h
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/nfs/nfs_mount.h
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/nfs/nfs_node.h
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/nfs/nfs_proto.h
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/nfs/nfs_util.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/nfs/nfs_vfsops.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/nfs/rpc.h
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/nfs/rpc_clnt.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/nfs/xdr_subs.h
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/nxffs/nxffs.h
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/nxffs/nxffs_block.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/nxffs/nxffs_blockstats.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/nxffs/nxffs_cache.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/nxffs/nxffs_dirent.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/nxffs/nxffs_dump.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/nxffs/nxffs_initialize.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/nxffs/nxffs_inode.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/nxffs/nxffs_ioctl.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/nxffs/nxffs_open.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/nxffs/nxffs_pack.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/nxffs/nxffs_read.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/nxffs/nxffs_reformat.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/nxffs/nxffs_stat.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/nxffs/nxffs_truncate.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/nxffs/nxffs_unlink.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/nxffs/nxffs_util.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/nxffs/nxffs_write.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/partition/fs_gpt.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/partition/fs_mbr.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/partition/fs_partition.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/partition/fs_ptable.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/partition/partition.h
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/procfs/fs_procfs.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/procfs/fs_procfscpuload.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/procfs/fs_procfscritmon.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/procfs/fs_procfsiobinfo.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/procfs/fs_procfsmeminfo.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/procfs/fs_procfsproc.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/procfs/fs_procfstcbinfo.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/procfs/fs_procfsuptime.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/procfs/fs_procfsutil.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/procfs/fs_procfsversion.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/procfs/fs_skeleton.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/romfs/fs_romfs.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/romfs/fs_romfs.h
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/romfs/fs_romfsutil.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/rpmsgfs/rpmsgfs.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/rpmsgfs/rpmsgfs.h
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/rpmsgfs/rpmsgfs_client.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/rpmsgfs/rpmsgfs_server.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/semaphore/sem_close.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/semaphore/sem_open.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/semaphore/sem_unlink.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/semaphore/semaphore.h
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/smartfs/smartfs.h
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/smartfs/smartfs_procfs.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/smartfs/smartfs_smart.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/smartfs/smartfs_utils.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/socket/socket.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/spiffs/src/spiffs.h
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/spiffs/src/spiffs_cache.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/spiffs/src/spiffs_cache.h
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/spiffs/src/spiffs_check.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/spiffs/src/spiffs_check.h
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/spiffs/src/spiffs_core.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/spiffs/src/spiffs_core.h
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/spiffs/src/spiffs_gc.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/spiffs/src/spiffs_gc.h
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/spiffs/src/spiffs_mtd.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/spiffs/src/spiffs_mtd.h
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/spiffs/src/spiffs_vfs.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/spiffs/src/spiffs_volume.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/tmpfs/fs_tmpfs.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/tmpfs/fs_tmpfs.h
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/unionfs/fs_unionfs.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/userfs/fs_userfs.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/userfs/userfs.h
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/vfs/fs_chstat.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/vfs/fs_close.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/vfs/fs_dir.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/vfs/fs_dup.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/vfs/fs_dup2.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/vfs/fs_epoll.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/vfs/fs_eventfd.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/vfs/fs_fchstat.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/vfs/fs_fcntl.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/vfs/fs_fdopen.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/vfs/fs_fstat.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/vfs/fs_fstatfs.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/vfs/fs_fsync.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/vfs/fs_ioctl.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/vfs/fs_lseek.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/vfs/fs_mkdir.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/vfs/fs_open.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/vfs/fs_poll.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/vfs/fs_pread.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/vfs/fs_pwrite.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/vfs/fs_read.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/vfs/fs_readlink.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/vfs/fs_rename.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/vfs/fs_rmdir.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/vfs/fs_select.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/vfs/fs_sendfile.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/vfs/fs_stat.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/vfs/fs_statfs.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/vfs/fs_symlink.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/vfs/fs_timerfd.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/vfs/fs_truncate.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/vfs/fs_unlink.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/fs/vfs/fs_write.c
NuttX/nuttx/fs/libfs.a: ../../platforms/nuttx/NuttX/nuttx/include/nuttx/config.h
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating nuttx/fs/libfs.a"
	cd /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/nuttx && /usr/bin/cmake -E remove -f /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/nuttx/fs/libfs.a
	cd /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/nuttx && find fs -type f -name *.o -delete
	cd /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/nuttx && make -C fs --no-print-directory --silent all TOPDIR="/home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/nuttx" KERNEL=y EXTRAFLAGS=-D__KERNEL__
	cd /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/nuttx && /usr/bin/cmake -E copy_if_different /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX/nuttx/fs/libfs.a /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/NuttX/nuttx/fs/libfs.a

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

nuttx_fs_build: NuttX/CMakeFiles/nuttx_fs_build
nuttx_fs_build: NuttX/nuttx/fs/libfs.a
nuttx_fs_build: ../../platforms/nuttx/NuttX/nuttx/include/nuttx/config.h
nuttx_fs_build: ../../platforms/nuttx/NuttX/nuttx/include/arch/chip
nuttx_fs_build: ../../platforms/nuttx/NuttX/nuttx/.config
nuttx_fs_build: NuttX/CMakeFiles/nuttx_fs_build.dir/build.make

.PHONY : nuttx_fs_build

# Rule to build all files generated by this target.
NuttX/CMakeFiles/nuttx_fs_build.dir/build: nuttx_fs_build

.PHONY : NuttX/CMakeFiles/nuttx_fs_build.dir/build

NuttX/CMakeFiles/nuttx_fs_build.dir/clean:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/NuttX && $(CMAKE_COMMAND) -P CMakeFiles/nuttx_fs_build.dir/cmake_clean.cmake
.PHONY : NuttX/CMakeFiles/nuttx_fs_build.dir/clean

NuttX/CMakeFiles/nuttx_fs_build.dir/depend:
	cd /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/andrey/Documents/ap/PX4-Autopilot /home/andrey/Documents/ap/PX4-Autopilot/platforms/nuttx/NuttX /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/NuttX /home/andrey/Documents/ap/PX4-Autopilot/build/px4_fmu-v6c_default/NuttX/CMakeFiles/nuttx_fs_build.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : NuttX/CMakeFiles/nuttx_fs_build.dir/depend

