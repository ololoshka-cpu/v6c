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

/* Auto-generated by genmsg_cpp from file /home/andrey/Documents/ap/PX4-Autopilot/msg/AirspeedWind.msg */


#pragma once


#include <uORB/uORB.h>


#ifndef __cplusplus
#define AIRSPEED_WIND_SOURCE_AS_BETA_ONLY 0
#define AIRSPEED_WIND_SOURCE_AS_SENSOR_1 1
#define AIRSPEED_WIND_SOURCE_AS_SENSOR_2 2
#define AIRSPEED_WIND_SOURCE_AS_SENSOR_3 3

#endif


#ifdef __cplusplus
struct __EXPORT airspeed_wind_s {
#else
struct airspeed_wind_s {
#endif
	uint64_t timestamp;
	uint64_t timestamp_sample;
	float windspeed_north;
	float windspeed_east;
	float variance_north;
	float variance_east;
	float tas_innov;
	float tas_innov_var;
	float tas_scale_raw;
	float tas_scale_raw_var;
	float tas_scale_validated;
	float beta_innov;
	float beta_innov_var;
	uint8_t source;
	uint8_t _padding0[3]; // required for logger


#ifdef __cplusplus
	static constexpr uint8_t SOURCE_AS_BETA_ONLY = 0;
	static constexpr uint8_t SOURCE_AS_SENSOR_1 = 1;
	static constexpr uint8_t SOURCE_AS_SENSOR_2 = 2;
	static constexpr uint8_t SOURCE_AS_SENSOR_3 = 3;

#endif
};

#ifdef __cplusplus
namespace px4 {
	namespace msg {
		using AirspeedWind = airspeed_wind_s;
	} // namespace msg
} // namespace px4
#endif

/* register this as object request broker structure */
ORB_DECLARE(airspeed_wind);


#ifdef __cplusplus
void print_message(const orb_metadata *meta, const airspeed_wind_s& message);
#endif
