/****************************************************************************
 *
 *   Copyright (C) 2013-2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/* Auto-generated by genmsg_cpp from file /home/andrey/Documents/ap/PX4-Autopilot/msg/GimbalDeviceInformation.msg */


#pragma once


#include <uORB/uORB.h>


#ifndef __cplusplus
#define GIMBAL_DEVICE_INFORMATION_GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT 1
#define GIMBAL_DEVICE_INFORMATION_GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL 2
#define GIMBAL_DEVICE_INFORMATION_GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS 4
#define GIMBAL_DEVICE_INFORMATION_GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW 8
#define GIMBAL_DEVICE_INFORMATION_GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK 16
#define GIMBAL_DEVICE_INFORMATION_GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS 32
#define GIMBAL_DEVICE_INFORMATION_GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW 64
#define GIMBAL_DEVICE_INFORMATION_GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK 128
#define GIMBAL_DEVICE_INFORMATION_GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS 256
#define GIMBAL_DEVICE_INFORMATION_GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW 512
#define GIMBAL_DEVICE_INFORMATION_GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK 1024
#define GIMBAL_DEVICE_INFORMATION_GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW 2048

#endif


#ifdef __cplusplus
struct __EXPORT gimbal_device_information_s {
#else
struct gimbal_device_information_s {
#endif
	uint64_t timestamp;
	uint64_t uid;
	uint32_t firmware_version;
	uint32_t hardware_version;
	float roll_min;
	float roll_max;
	float pitch_min;
	float pitch_max;
	float yaw_min;
	float yaw_max;
	uint16_t cap_flags;
	uint16_t custom_cap_flags;
	uint8_t vendor_name[32];
	uint8_t model_name[32];
	uint8_t custom_name[32];
	uint8_t gimbal_device_compid;
	uint8_t _padding0[3]; // required for logger


#ifdef __cplusplus
	static constexpr uint32_t GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT = 1;
	static constexpr uint32_t GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL = 2;
	static constexpr uint32_t GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS = 4;
	static constexpr uint32_t GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW = 8;
	static constexpr uint32_t GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK = 16;
	static constexpr uint32_t GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS = 32;
	static constexpr uint32_t GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW = 64;
	static constexpr uint32_t GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK = 128;
	static constexpr uint32_t GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS = 256;
	static constexpr uint32_t GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW = 512;
	static constexpr uint32_t GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK = 1024;
	static constexpr uint32_t GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW = 2048;

#endif
};

#ifdef __cplusplus
namespace px4 {
	namespace msg {
		using GimbalDeviceInformation = gimbal_device_information_s;
	} // namespace msg
} // namespace px4
#endif

/* register this as object request broker structure */
ORB_DECLARE(gimbal_device_information);


#ifdef __cplusplus
void print_message(const orb_metadata *meta, const gimbal_device_information_s& message);
#endif
