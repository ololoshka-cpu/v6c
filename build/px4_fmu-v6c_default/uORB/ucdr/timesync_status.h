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


// auto-generated file

#pragma once

#include <ucdr/microcdr.h>
#include <string.h>
#include <uORB/topics/timesync_status.h>


static inline constexpr int ucdr_topic_size_timesync_status()
{
	return 44;
}

bool ucdr_serialize_timesync_status(const timesync_status_s& topic, ucdrBuffer& buf, int64_t time_offset = 0)
{
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	const uint64_t timestamp_adjusted = topic.timestamp + time_offset;
	memcpy(buf.iterator, &timestamp_adjusted, sizeof(topic.timestamp));
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.source_protocol) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.source_protocol, sizeof(topic.source_protocol));
	buf.iterator += sizeof(topic.source_protocol);
	buf.offset += sizeof(topic.source_protocol);
	buf.iterator += 7; // padding
	buf.offset += 7; // padding
	static_assert(sizeof(topic.remote_timestamp) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.remote_timestamp, sizeof(topic.remote_timestamp));
	buf.iterator += sizeof(topic.remote_timestamp);
	buf.offset += sizeof(topic.remote_timestamp);
	static_assert(sizeof(topic.observed_offset) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.observed_offset, sizeof(topic.observed_offset));
	buf.iterator += sizeof(topic.observed_offset);
	buf.offset += sizeof(topic.observed_offset);
	static_assert(sizeof(topic.estimated_offset) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.estimated_offset, sizeof(topic.estimated_offset));
	buf.iterator += sizeof(topic.estimated_offset);
	buf.offset += sizeof(topic.estimated_offset);
	static_assert(sizeof(topic.round_trip_time) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.round_trip_time, sizeof(topic.round_trip_time));
	buf.iterator += sizeof(topic.round_trip_time);
	buf.offset += sizeof(topic.round_trip_time);
	return true;
}

bool ucdr_deserialize_timesync_status(ucdrBuffer& buf, timesync_status_s& topic, int64_t time_offset = 0)
{
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	memcpy(&topic.timestamp, buf.iterator, sizeof(topic.timestamp));
	if (topic.timestamp == 0) topic.timestamp = hrt_absolute_time();
	else topic.timestamp = math::min(topic.timestamp - time_offset, hrt_absolute_time());
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.source_protocol) == 1, "size mismatch");
	memcpy(&topic.source_protocol, buf.iterator, sizeof(topic.source_protocol));
	buf.iterator += sizeof(topic.source_protocol);
	buf.offset += sizeof(topic.source_protocol);
	buf.iterator += 7; // padding
	buf.offset += 7; // padding
	static_assert(sizeof(topic.remote_timestamp) == 8, "size mismatch");
	memcpy(&topic.remote_timestamp, buf.iterator, sizeof(topic.remote_timestamp));
	buf.iterator += sizeof(topic.remote_timestamp);
	buf.offset += sizeof(topic.remote_timestamp);
	static_assert(sizeof(topic.observed_offset) == 8, "size mismatch");
	memcpy(&topic.observed_offset, buf.iterator, sizeof(topic.observed_offset));
	buf.iterator += sizeof(topic.observed_offset);
	buf.offset += sizeof(topic.observed_offset);
	static_assert(sizeof(topic.estimated_offset) == 8, "size mismatch");
	memcpy(&topic.estimated_offset, buf.iterator, sizeof(topic.estimated_offset));
	buf.iterator += sizeof(topic.estimated_offset);
	buf.offset += sizeof(topic.estimated_offset);
	static_assert(sizeof(topic.round_trip_time) == 4, "size mismatch");
	memcpy(&topic.round_trip_time, buf.iterator, sizeof(topic.round_trip_time));
	buf.iterator += sizeof(topic.round_trip_time);
	buf.offset += sizeof(topic.round_trip_time);
	return true;
}