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

/* Auto-generated by genmsg_cpp from file /home/andrey/Documents/ap/PX4-Autopilot/msg/LoggerStatus.msg */


#pragma once


#include <uORB/uORB.h>


#ifndef __cplusplus
#define LOGGER_STATUS_LOGGER_TYPE_FULL 0
#define LOGGER_STATUS_LOGGER_TYPE_MISSION 1
#define LOGGER_STATUS_BACKEND_FILE 1
#define LOGGER_STATUS_BACKEND_MAVLINK 2
#define LOGGER_STATUS_BACKEND_ALL 3

#endif


#ifdef __cplusplus
struct __EXPORT logger_status_s {
#else
struct logger_status_s {
#endif
	uint64_t timestamp;
	float total_written_kb;
	float write_rate_kb_s;
	uint32_t dropouts;
	uint32_t message_gaps;
	uint32_t buffer_used_bytes;
	uint32_t buffer_size_bytes;
	uint8_t type;
	uint8_t backend;
	uint8_t num_messages;
	uint8_t _padding0[5]; // required for logger


#ifdef __cplusplus
	static constexpr uint8_t LOGGER_TYPE_FULL = 0;
	static constexpr uint8_t LOGGER_TYPE_MISSION = 1;
	static constexpr uint8_t BACKEND_FILE = 1;
	static constexpr uint8_t BACKEND_MAVLINK = 2;
	static constexpr uint8_t BACKEND_ALL = 3;

#endif
};

#ifdef __cplusplus
namespace px4 {
	namespace msg {
		using LoggerStatus = logger_status_s;
	} // namespace msg
} // namespace px4
#endif

/* register this as object request broker structure */
ORB_DECLARE(logger_status);


#ifdef __cplusplus
void print_message(const orb_metadata *meta, const logger_status_s& message);
#endif
