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
#include <uORB/topics/mission.h>


static inline constexpr int ucdr_topic_size_mission()
{
	return 22;
}

bool ucdr_serialize_mission(const mission_s& topic, ucdrBuffer& buf, int64_t time_offset = 0)
{
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	const uint64_t timestamp_adjusted = topic.timestamp + time_offset;
	memcpy(buf.iterator, &timestamp_adjusted, sizeof(topic.timestamp));
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.dataman_id) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.dataman_id, sizeof(topic.dataman_id));
	buf.iterator += sizeof(topic.dataman_id);
	buf.offset += sizeof(topic.dataman_id);
	buf.iterator += 1; // padding
	buf.offset += 1; // padding
	static_assert(sizeof(topic.count) == 2, "size mismatch");
	memcpy(buf.iterator, &topic.count, sizeof(topic.count));
	buf.iterator += sizeof(topic.count);
	buf.offset += sizeof(topic.count);
	static_assert(sizeof(topic.current_seq) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.current_seq, sizeof(topic.current_seq));
	buf.iterator += sizeof(topic.current_seq);
	buf.offset += sizeof(topic.current_seq);
	static_assert(sizeof(topic.mission_update_counter) == 2, "size mismatch");
	memcpy(buf.iterator, &topic.mission_update_counter, sizeof(topic.mission_update_counter));
	buf.iterator += sizeof(topic.mission_update_counter);
	buf.offset += sizeof(topic.mission_update_counter);
	static_assert(sizeof(topic.geofence_update_counter) == 2, "size mismatch");
	memcpy(buf.iterator, &topic.geofence_update_counter, sizeof(topic.geofence_update_counter));
	buf.iterator += sizeof(topic.geofence_update_counter);
	buf.offset += sizeof(topic.geofence_update_counter);
	static_assert(sizeof(topic.safe_points_update_counter) == 2, "size mismatch");
	memcpy(buf.iterator, &topic.safe_points_update_counter, sizeof(topic.safe_points_update_counter));
	buf.iterator += sizeof(topic.safe_points_update_counter);
	buf.offset += sizeof(topic.safe_points_update_counter);
	return true;
}

bool ucdr_deserialize_mission(ucdrBuffer& buf, mission_s& topic, int64_t time_offset = 0)
{
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	memcpy(&topic.timestamp, buf.iterator, sizeof(topic.timestamp));
	if (topic.timestamp == 0) topic.timestamp = hrt_absolute_time();
	else topic.timestamp = math::min(topic.timestamp - time_offset, hrt_absolute_time());
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.dataman_id) == 1, "size mismatch");
	memcpy(&topic.dataman_id, buf.iterator, sizeof(topic.dataman_id));
	buf.iterator += sizeof(topic.dataman_id);
	buf.offset += sizeof(topic.dataman_id);
	buf.iterator += 1; // padding
	buf.offset += 1; // padding
	static_assert(sizeof(topic.count) == 2, "size mismatch");
	memcpy(&topic.count, buf.iterator, sizeof(topic.count));
	buf.iterator += sizeof(topic.count);
	buf.offset += sizeof(topic.count);
	static_assert(sizeof(topic.current_seq) == 4, "size mismatch");
	memcpy(&topic.current_seq, buf.iterator, sizeof(topic.current_seq));
	buf.iterator += sizeof(topic.current_seq);
	buf.offset += sizeof(topic.current_seq);
	static_assert(sizeof(topic.mission_update_counter) == 2, "size mismatch");
	memcpy(&topic.mission_update_counter, buf.iterator, sizeof(topic.mission_update_counter));
	buf.iterator += sizeof(topic.mission_update_counter);
	buf.offset += sizeof(topic.mission_update_counter);
	static_assert(sizeof(topic.geofence_update_counter) == 2, "size mismatch");
	memcpy(&topic.geofence_update_counter, buf.iterator, sizeof(topic.geofence_update_counter));
	buf.iterator += sizeof(topic.geofence_update_counter);
	buf.offset += sizeof(topic.geofence_update_counter);
	static_assert(sizeof(topic.safe_points_update_counter) == 2, "size mismatch");
	memcpy(&topic.safe_points_update_counter, buf.iterator, sizeof(topic.safe_points_update_counter));
	buf.iterator += sizeof(topic.safe_points_update_counter);
	buf.offset += sizeof(topic.safe_points_update_counter);
	return true;
}
