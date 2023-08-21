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

/* Auto-generated by genmsg_cpp from file /home/andrey/Documents/ap/PX4-Autopilot/msg/EscStatus.msg */


#pragma once


#include <uORB/uORB.h>

#include <uORB/topics/esc_report.h>

#ifndef __cplusplus
#define ESC_STATUS_CONNECTED_ESC_MAX 8
#define ESC_STATUS_ESC_CONNECTION_TYPE_PPM 0
#define ESC_STATUS_ESC_CONNECTION_TYPE_SERIAL 1
#define ESC_STATUS_ESC_CONNECTION_TYPE_ONESHOT 2
#define ESC_STATUS_ESC_CONNECTION_TYPE_I2C 3
#define ESC_STATUS_ESC_CONNECTION_TYPE_CAN 4
#define ESC_STATUS_ESC_CONNECTION_TYPE_DSHOT 5

#endif


#ifdef __cplusplus
struct __EXPORT esc_status_s {
#else
struct esc_status_s {
#endif
	uint64_t timestamp;
	uint16_t counter;
	uint8_t esc_count;
	uint8_t esc_connectiontype;
	uint8_t esc_online_flags;
	uint8_t esc_armed_flags;
	uint8_t _padding0[2]; // required for logger
	struct esc_report_s esc[8];


#ifdef __cplusplus
	static constexpr uint8_t CONNECTED_ESC_MAX = 8;
	static constexpr uint8_t ESC_CONNECTION_TYPE_PPM = 0;
	static constexpr uint8_t ESC_CONNECTION_TYPE_SERIAL = 1;
	static constexpr uint8_t ESC_CONNECTION_TYPE_ONESHOT = 2;
	static constexpr uint8_t ESC_CONNECTION_TYPE_I2C = 3;
	static constexpr uint8_t ESC_CONNECTION_TYPE_CAN = 4;
	static constexpr uint8_t ESC_CONNECTION_TYPE_DSHOT = 5;

#endif
};

#ifdef __cplusplus
namespace px4 {
	namespace msg {
		using EscStatus = esc_status_s;
	} // namespace msg
} // namespace px4
#endif

/* register this as object request broker structure */
ORB_DECLARE(esc_status);


#ifdef __cplusplus
void print_message(const orb_metadata *meta, const esc_status_s& message);
#endif
