/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

/**
 * @file mission_route_test_helpers.h
 *
 * Shared helpers for MissionRouteCache and MissionRoutePlanner unit tests.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#pragma once

#include <gtest/gtest.h>

#include "mission_route_cache.h"
#include "mission_route_planner.h"
#include "navigator_dataman_test.h"
#include "vector_mission_item_store.h"

#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <px4_platform_common/log.h>
#include <uORB/topics/vtol_vehicle_status.h>

#include <inttypes.h>
#include <cmath>
#include <vector>

using namespace time_literals;

namespace navigator_test
{

class VectorMissionRouteProvider : public MissionRoutePlanner::Provider
{
public:
	VectorMissionRouteProvider(const std::vector<mission_item_s> &mission_items,
				   const std::vector<mission_item_s> &safe_point_items,
				   const std::vector<int32_t> &faulty_mission_indices = {},
				   const std::vector<int32_t> &faulty_safe_point_indices = {})
	{
		_mission_items.setItems(mission_items);
		_safe_point_items.setItems(safe_point_items);
		_mission_items.setLoadFailureIndices(faulty_mission_indices);
		_safe_point_items.setLoadFailureIndices(faulty_safe_point_indices);
	}

	int missionCount() const override { return static_cast<int>(_mission_items.itemCount()); }

	bool loadMissionItem(int index, mission_item_s &mission_item) const override
	{
		++_mission_load_count;
		return _mission_items.loadItem(index, mission_item);
	}

	int safePointCount() const override { return static_cast<int>(_safe_point_items.itemCount()); }

	bool loadSafePointItem(int index, mission_item_s &safe_point_item) const override
	{
		++_safe_point_load_count;
		return _safe_point_items.loadItem(index, safe_point_item);
	}

	void resetCounters() const
	{
		_mission_load_count = 0;
		_safe_point_load_count = 0;
	}

	int missionLoadCount() const { return _mission_load_count; }
	int safePointLoadCount() const { return _safe_point_load_count; }

private:
	VectorMissionItemStore _mission_items{};
	VectorMissionItemStore _safe_point_items{};
	mutable int _mission_load_count{0};
	mutable int _safe_point_load_count{0};
};

static inline mission_item_s makePositionItem(double lat, double lon, float alt,
		uint16_t nav_cmd = NAV_CMD_WAYPOINT)
{
	mission_item_s item{};
	item.lat = lat;
	item.lon = lon;
	item.altitude = alt;
	item.nav_cmd = nav_cmd;
	item.frame = NAV_FRAME_GLOBAL;
	item.altitude_is_relative = false;
	item.autocontinue = true;
	return item;
}

static inline mission_item_s makePositionItemFromOffset(double base_lat, double base_lon,
		float north_m, float east_m, float alt, uint16_t nav_cmd = NAV_CMD_WAYPOINT, bool autocontinue = true)
{
	double lat = base_lat;
	double lon = base_lon;
	add_vector_to_global_position(base_lat, base_lon, north_m, east_m, &lat, &lon);

	mission_item_s item = makePositionItem(lat, lon, alt, nav_cmd);
	item.autocontinue = autocontinue;
	return item;
}

static inline mission_item_s makeTakeoffItemFromOffset(double base_lat, double base_lon,
		float north_m, float east_m, float alt)
{
	return makePositionItemFromOffset(base_lat, base_lon, north_m, east_m, alt, NAV_CMD_TAKEOFF, false);
}

static inline mission_item_s makeLandItemFromOffset(double base_lat, double base_lon,
		float north_m, float east_m, float alt)
{
	return makePositionItemFromOffset(base_lat, base_lon, north_m, east_m, alt, NAV_CMD_LAND, false);
}

static inline mission_item_s makeTakeoffItem(double lat, double lon, float alt)
{
	mission_item_s item = makePositionItem(lat, lon, alt, NAV_CMD_TAKEOFF);
	item.autocontinue = false;
	return item;
}

static inline mission_item_s makeLandItem(double lat, double lon, float alt)
{
	mission_item_s item = makePositionItem(lat, lon, alt, NAV_CMD_LAND);
	item.autocontinue = false;
	return item;
}

static inline mission_item_s makeDoJump(int16_t jump_target_index, uint16_t repeat_count,
					uint16_t current_count = 0)
{
	mission_item_s item{};
	item.nav_cmd = NAV_CMD_DO_JUMP;
	item.do_jump_mission_index = jump_target_index;
	item.do_jump_repeat_count = repeat_count;
	item.do_jump_current_count = current_count;
	return item;
}

static inline mission_item_s makeVtolTransitionItem(uint8_t target_state)
{
	mission_item_s item{};
	item.nav_cmd = NAV_CMD_DO_VTOL_TRANSITION;
	item.params[0] = static_cast<float>(target_state);
	return item;
}

static inline mission_item_s makeSafePointFromOffset(double base_lat, double base_lon,
		float north_m, float east_m, float alt, uint8_t frame = NAV_FRAME_GLOBAL)
{
	mission_item_s item = makePositionItemFromOffset(base_lat, base_lon, north_m, east_m, alt, NAV_CMD_RALLY_POINT);
	item.frame = frame;
	return item;
}

static inline mission_item_s makeSafePointAbsolute(double lat, double lon, float alt, uint8_t frame = NAV_FRAME_GLOBAL)
{
	mission_item_s item = makePositionItem(lat, lon, alt, NAV_CMD_RALLY_POINT);
	item.frame = frame;
	return item;
}

static inline MissionRoutePlanner::Position makePositionFromOffset(double base_lat, double base_lon,
		float north_m, float east_m, float alt)
{
	MissionRoutePlanner::Position position{};
	add_vector_to_global_position(base_lat, base_lon, north_m, east_m, &position.lat, &position.lon);
	position.alt = alt;
	return position;
}

static inline MissionRoutePlanner::Position makePositionAbsolute(double lat, double lon, float alt)
{
	return MissionRoutePlanner::Position{lat, lon, alt};
}

static inline MissionRoutePlanner::Config defaultConfig()
{
	MissionRoutePlanner::Config config{};
	config.parameters.vehicle_projection_search_dist = 60.f;
	config.parameters.safe_point_projection_search_dist = 60.f;
	config.parameters.acceptance_radius = 10.f;
	config.parameters.direct_acceptance_radius = 10.f;
	config.parameters.home_altitude_amsl = 500.f;
	return config;
}

static inline MissionRoutePlanner::Config fwConfig()
{
	MissionRoutePlanner::Config config = defaultConfig();
	config.state.is_fixed_wing = true;
	config.parameters.u_turn_penalty_m = 4000.f;
	return config;
}

namespace route_test_reference
{
static constexpr double kBaseLat = 47.397742;
static constexpr double kBaseLon = 8.545594;
static constexpr float kAlt = 500.f;
}

} // namespace navigator_test

using VectorProvider = navigator_test::VectorMissionRouteProvider;
using navigator_test::makeDoJump;
using navigator_test::makeLandItem;
using navigator_test::makeLandItemFromOffset;
using navigator_test::makePositionAbsolute;
using navigator_test::makePositionFromOffset;
using navigator_test::makePositionItem;
using navigator_test::makePositionItemFromOffset;
using navigator_test::makeSafePointAbsolute;
using navigator_test::makeSafePointFromOffset;
using navigator_test::makeTakeoffItem;
using navigator_test::makeTakeoffItemFromOffset;
using navigator_test::makeVtolTransitionItem;
using navigator_test::defaultConfig;
using navigator_test::fwConfig;
namespace rtl_test_reference = navigator_test::route_test_reference;

static constexpr double kLatLonToleranceDeg = 1e-7;     // ~1 cm at equator
static constexpr float kAltitudeTolerance = 2.0f;       // meters
static constexpr float kDistanceTolerance = 5.0f;       // meters

class DatamanClientTestPeer
{
public:
	static bool waitForOperation(DatamanClient &client, hrt_abstime timeout)
	{
		if (client._state != DatamanClient::State::RequestSent) {
			return true;
		}

		const hrt_abstime start_time = hrt_absolute_time();

		do {
			client.update();

			if (client._state != DatamanClient::State::RequestSent) {
				return true;
			}

			const hrt_abstime elapsed = hrt_elapsed_time(&start_time);

			if (elapsed >= timeout) {
				break;
			}

			const hrt_abstime remaining = timeout - elapsed;
			const uint32_t timeout_ms = (remaining >= 100_ms) ? 100U :
						    (remaining > 1000) ? static_cast<uint32_t>(remaining / 1000) : 1U;
			const int32_t ret = px4_poll(&client._fds, 1, timeout_ms);

			if (ret < 0) {
				PX4_ERR("px4_poll returned error: %" PRIi32, ret);
				break;
			}

		} while (true);

		client.update();
		return client._state != DatamanClient::State::RequestSent;
	}
};

class MissionRoutePlannerTestBase : public ::testing::Test
{
protected:
	MissionRoutePlanner::Config config = defaultConfig();
	MissionRoutePlanner::ProjectionContext ctx{};
	MissionRoutePlanner::FailureReason reason{};
};

class DatamanCacheTestPeer
{
public:
	static bool processNextBlocking(DatamanCache &cache, hrt_abstime timeout)
	{
		if (cache._item_counter == 0) {
			return true;
		}

		cache.update();

		if (cache._item_counter == 0) {
			return true;
		}

		if (cache._items[cache._update_index].cache_state == DatamanCache::State::RequestSent) {
			if (!DatamanClientTestPeer::waitForOperation(cache._client, timeout)) {
				return false;
			}

			cache.update();
		}

		return true;
	}
};

class MissionRouteCacheTestPeer
{
public:
	static bool missionRetryScheduled(const MissionRouteCache &cache)
	{
		return cache._mission.retry_at != 0;
	}

	static uint8_t missionRetryCount(const MissionRouteCache &cache)
	{
		return cache._mission.retry_count;
	}

	static bool safePointRetryScheduled(const MissionRouteCache &cache)
	{
		return cache._safe_point.retry_at != 0;
	}

	static bool safePointLoadInProgress(const MissionRouteCache &cache)
	{
		return cache._safe_point.dataman_state == MissionRouteCache::SafePointDatamanState::Load
		       && cache._dataman_cache_safepoint.isLoading();
	}

	static bool queueMissionCacheLoads(MissionRouteCache &cache, const mission_s &mission)
	{
		return cache.queueMissionCacheLoads(mission);
	}

	static bool preparePartialMissionCache(MissionRouteCache &cache, const mission_s &mission, uint32_t cached_index)
	{
		cache._mission = {};
		cache._mission.id = mission.mission_id;
		cache._mission.count = mission.count;
		cache._mission.dataman_id = mission.mission_dataman_id;
		cache._mission.validation_pending = true;
		cache._dataman_cache_mission.invalidate();

		if (static_cast<int32_t>(cache._dataman_cache_mission.size()) < mission.count) {
			return false;
		}

		return cache._dataman_cache_mission.load(static_cast<dm_item_t>(mission.mission_dataman_id), cached_index);
	}

	template<typename Predicate>
	static bool updateUntil(MissionRouteCache &cache, const mission_s &mission, Predicate &&predicate,
				hrt_abstime timeout = 5_s)
	{
		const hrt_abstime start_time = hrt_absolute_time();

		while (hrt_elapsed_time(&start_time) < timeout) {
			cache.update(mission);

			if (predicate()) {
				return true;
			}

			const hrt_abstime elapsed = hrt_elapsed_time(&start_time);
			const hrt_abstime remaining = (elapsed < timeout) ? (timeout - elapsed) : 0;

			if (!progressOneEvent(cache, mission, remaining)) {
				break;
			}

			if (predicate()) {
				return true;
			}
		}

		cache.update(mission);
		return predicate();
	}

private:
	static bool progressOneEvent(MissionRouteCache &cache, const mission_s &mission, hrt_abstime timeout)
	{
		if (cache._mission.validation_pending && cache._dataman_cache_mission.isLoading()) {
			return DatamanCacheTestPeer::processNextBlocking(cache._dataman_cache_mission, timeout);
		}

		if (cache._mission_land.index >= 0 && cache._dataman_cache_land_item.isLoading()) {
			return DatamanCacheTestPeer::processNextBlocking(cache._dataman_cache_land_item, timeout);
		}

		if (cache._safe_point.dataman_state == MissionRouteCache::SafePointDatamanState::UpdateRequestWait
		    && cache._safe_point.update_requested
		    && cache._safe_point.retry_at == 0) {
			cache.update(mission);
			return true;
		}

		if (cache._safe_point.dataman_state == MissionRouteCache::SafePointDatamanState::Read) {
			cache.update(mission);
			return true;
		}

		if (cache._safe_point.dataman_state == MissionRouteCache::SafePointDatamanState::ReadWait) {
			if (!DatamanClientTestPeer::waitForOperation(cache._dataman_client_safepoint, timeout)) {
				return false;
			}

			cache.update(mission);
			return true;
		}

		if (cache._safe_point.dataman_state == MissionRouteCache::SafePointDatamanState::Load
		    && cache._dataman_cache_safepoint.isLoading()) {
			return DatamanCacheTestPeer::processNextBlocking(cache._dataman_cache_safepoint, timeout);
		}

		bool forced_retry = false;

		if (cache._mission.retry_at != 0) {
			cache._mission.retry_at = 0;
			forced_retry = true;
		}

		if (cache._safe_point.retry_at != 0) {
			cache._safe_point.retry_at = 0;
			forced_retry = true;
		}

		if (forced_retry) {
			cache.update(mission);
		}

		return forced_retry;
	}
};
