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
 * @file test_navigator_smart_mission_join.cpp
 *
 * Navigator-level regression tests for Mission smart route rejoin.
 */

#include <gtest/gtest.h>

#include <dataman_client/DatamanClient.hpp>
#include <drivers/drv_hrt.h>
#include <parameters/param.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/home_position.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vtol_vehicle_status.h>

#include "mission.h"
#include "navigator.h"
#include "test_RTL_helpers.h"

using rtl_test_reference::kAlt;
using rtl_test_reference::kBaseLat;
using rtl_test_reference::kBaseLon;

class MissionPeer : public Mission
{
public:
	explicit MissionPeer(Navigator *navigator) : Mission(navigator) {}

	using Mission::trySetRouteJoinOnActivation;
	using MissionBase::VtolTransitionAction;
	using MissionBase::WorkItemType;

	WorkItemType workItemTypeForTest() const { return _work_item_type; }
	int32_t currentSequenceForTest() const { return _mission.current_seq; }
	const MissionRoutePlanner::JoinContext &joinContextForTest() const { return _route_join_context; }
	VtolTransitionAction joinTransitionActionForTest() const { return _route_join_context.transition_action; }
};

class NavigatorSmartMissionJoinTest : public NavigatorDatamanTestBase
{
protected:
	void SetUp() override
	{
		const hrt_abstime now = hrt_absolute_time();

		mission_s mission{};
		mission.timestamp = now;
		_mission_pub.publish(mission);

		publishVehicleStatus(false, vehicle_status_s::VEHICLE_TYPE_UNSPECIFIED);
		publishLandDetected(true);
		publishGlobalPosition(MissionRoutePlanner::Position{NAN, NAN, NAN});
		publishLocalPosition(NAN, NAN, NAN);

		home_position_s home_position{};
		home_position.timestamp = now;
		_home_position_pub.publish(home_position);
		_home_position = home_position;
	}

	void setIntParam(const char *name, int32_t value)
	{
		const param_t handle = param_find(name);
		ASSERT_NE(handle, PARAM_INVALID) << name;
		ASSERT_EQ(param_set(handle, &value), PX4_OK) << name;
	}

	void writeMissionItems(const std::vector<mission_item_s> &items, dm_item_t dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0)
	{
		for (size_t i = 0; i < items.size(); ++i) {
			mission_item_s item = items[i];
			ASSERT_TRUE(_dataman_client.writeSync(dataman_id, i,
							      reinterpret_cast<uint8_t *>(&item), sizeof(item)));
		}
	}

	void writeSafePointState(uint16_t num_items, uint32_t opaque_id, dm_item_t dataman_id = DM_KEY_SAFE_POINTS_0)
	{
		mission_stats_entry_s stats{};
		stats.num_items = num_items;
		stats.opaque_id = opaque_id;
		stats.dataman_id = static_cast<uint8_t>(dataman_id);
		ASSERT_TRUE(_dataman_client.writeSync(DM_KEY_SAFE_POINTS_STATE, 0,
						      reinterpret_cast<uint8_t *>(&stats), sizeof(stats)));
	}

	void publishMission(const mission_s &mission)
	{
		_mission_pub.publish(mission);
	}

	void publishVehicleStatus(bool is_vtol, uint8_t vehicle_type, bool in_transition_to_fw = false)
	{
		vehicle_status_s status{};
		status.timestamp = hrt_absolute_time();
		status.is_vtol = is_vtol;
		status.vehicle_type = vehicle_type;
		status.in_transition_mode = in_transition_to_fw;
		status.in_transition_to_fw = in_transition_to_fw;
		status.arming_state = vehicle_status_s::ARMING_STATE_ARMED;
		_vehicle_status_pub.publish(status);
		_vehicle_status = status;
	}

	void publishLandDetected(bool landed)
	{
		vehicle_land_detected_s land_detected{};
		land_detected.timestamp = hrt_absolute_time();
		land_detected.landed = landed;
		_land_detected_pub.publish(land_detected);
		_land_detected = land_detected;
	}

	void publishGlobalPosition(const MissionRoutePlanner::Position &position)
	{
		vehicle_global_position_s global_position{};
		global_position.timestamp = hrt_absolute_time();
		global_position.lat = position.lat;
		global_position.lon = position.lon;
		global_position.alt = position.alt;
		_vehicle_global_position_pub.publish(global_position);
		_global_position = global_position;
	}

	void publishLocalPosition(float heading_rad, float vx, float vy)
	{
		vehicle_local_position_s local_position{};
		local_position.timestamp = hrt_absolute_time();
		local_position.xy_valid = true;
		local_position.z_valid = true;
		local_position.heading = heading_rad;
		local_position.vx = vx;
		local_position.vy = vy;
		_vehicle_local_position_pub.publish(local_position);
		_local_position = local_position;
	}

	void publishHomePosition(const MissionRoutePlanner::Position &position)
	{
		home_position_s home_position{};
		home_position.timestamp = hrt_absolute_time();
		home_position.valid_hpos = true;
		home_position.valid_alt = true;
		home_position.lat = position.lat;
		home_position.lon = position.lon;
		home_position.alt = position.alt;
		_home_position_pub.publish(home_position);
		_home_position = home_position;
	}

	void primeNavigatorState(Navigator &navigator)
	{
		*navigator.get_vstatus() = _vehicle_status;
		*navigator.get_land_detected() = _land_detected;
		*navigator.get_global_position() = _global_position;
		*navigator.get_local_position() = _local_position;
		*navigator.get_home_position() = _home_position;
	}

	void markMissionResultValid(Navigator &navigator)
	{
		*navigator.get_mission_result() = mission_result_s{};
		navigator.get_mission_result()->valid = true;
	}

	void updateRouteCacheUntilReady(Navigator &navigator, const mission_s &mission)
	{
		MissionRouteCache *route_cache = navigator.get_mission_route_cache();
		ASSERT_NE(route_cache, nullptr);
		ASSERT_TRUE(MissionRouteCacheTestPeer::updateUntil(*route_cache, mission,
				[&] { return route_cache->isReady(mission) && route_cache->safePointsReady(); }))
				<< "MissionRouteCache did not become ready";
	}

	mission_s makeMissionState(uint32_t mission_id, uint32_t safe_points_id,
				   uint16_t count, int32_t current_seq, int32_t land_index = -1)
	{
		mission_s mission_state{};
		mission_state.timestamp = hrt_absolute_time();
		mission_state.current_seq = current_seq;
		mission_state.land_start_index = land_index;
		mission_state.land_index = land_index;
		mission_state.mission_id = mission_id;
		mission_state.safe_points_id = safe_points_id;
		mission_state.count = count;
		mission_state.mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0;
		mission_state.fence_dataman_id = DM_KEY_FENCE_POINTS_0;
		mission_state.safepoint_dataman_id = DM_KEY_SAFE_POINTS_0;
		return mission_state;
	}

	DatamanClient _dataman_client{};
	uORB::Publication<mission_s> _mission_pub{ORB_ID(mission)};
	uORB::Publication<vehicle_status_s> _vehicle_status_pub{ORB_ID(vehicle_status)};
	uORB::Publication<vehicle_land_detected_s> _land_detected_pub{ORB_ID(vehicle_land_detected)};
	uORB::Publication<vehicle_global_position_s> _vehicle_global_position_pub{ORB_ID(vehicle_global_position)};
	uORB::Publication<vehicle_local_position_s> _vehicle_local_position_pub{ORB_ID(vehicle_local_position)};
	uORB::Publication<home_position_s> _home_position_pub{ORB_ID(home_position)};

	vehicle_status_s _vehicle_status{};
	vehicle_land_detected_s _land_detected{};
	vehicle_global_position_s _global_position{};
	vehicle_local_position_s _local_position{};
	home_position_s _home_position{};
};

// WHY: Smart mission rejoin must pick the nearest usable loop exit instead of restarting an exhausted loop.
// WHAT: A completed DO_JUMP loop and a vehicle near WP2 target idx 2 and arm JOIN_ROUTE.
TEST_F(NavigatorSmartMissionJoinTest, UsesShortestExhaustedLoopExit)
{
	setIntParam("MIS_ROUTE_JOIN", 1);
	Navigator navigator;
	MissionPeer mission(&navigator);

	std::vector<mission_item_s> mission_items = {
		makePositionItemFromOffset(kBaseLat, kBaseLon,   0.f,   0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f,   0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 100.f, kAlt),
		makeDoJump(0, 2, 2),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 200.f,   0.f, kAlt - 10.f),
	};

	writeMissionItems(mission_items);
	writeSafePointState(0, 11);
	const mission_s mission_state = makeMissionState(21, 11, mission_items.size(), 0, 4);
	publishMission(mission_state);

	publishVehicleStatus(false, vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
	publishLandDetected(false);
	publishGlobalPosition(makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 95.f, kAlt));
	publishLocalPosition(0.f, 12.f, 0.f);
	publishHomePosition(makePositionFromOffset(kBaseLat, kBaseLon, -200.f, 0.f, kAlt));
	primeNavigatorState(navigator);

	updateRouteCacheUntilReady(navigator, mission_state);
	mission.on_inactive();

	ASSERT_TRUE(mission.trySetRouteJoinOnActivation(false));
	EXPECT_EQ(mission.currentSequenceForTest(), 2);
	EXPECT_EQ(mission.workItemTypeForTest(), MissionPeer::WorkItemType::WORK_ITEM_TYPE_JOIN_ROUTE);
	EXPECT_TRUE(mission.joinContextForTest().valid());
	EXPECT_EQ(mission.joinTransitionActionForTest(), MissionPeer::VtolTransitionAction::None);
}

// WHY: Near-landing rejoin must not climb back to the mission-route altitude.
// WHAT: A vehicle below the landing segment gets a live-altitude branch-in waypoint.
TEST_F(NavigatorSmartMissionJoinTest, NearLandingSkipsAltitudeRequirement)
{
	setIntParam("MIS_ROUTE_JOIN", 1);
	Navigator navigator;
	MissionPeer mission(&navigator);

	std::vector<mission_item_s> mission_items = {
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon,   0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 120.f, 0.f, kAlt - 10.f),
	};

	writeMissionItems(mission_items);
	writeSafePointState(0, 13);
	const mission_s mission_state = makeMissionState(23, 13, mission_items.size(), 2, 2);
	publishMission(mission_state);

	const MissionRoutePlanner::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 118.f, 0.f, kAlt - 6.f);

	publishVehicleStatus(false, vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
	publishLandDetected(false);
	publishGlobalPosition(vehicle_position);
	publishLocalPosition(0.f, 8.f, 0.f);
	publishHomePosition(makePositionFromOffset(kBaseLat, kBaseLon, -100.f, 0.f, kAlt));
	primeNavigatorState(navigator);

	updateRouteCacheUntilReady(navigator, mission_state);
	mission.on_inactive();

	ASSERT_TRUE(mission.trySetRouteJoinOnActivation(false));
	EXPECT_EQ(mission.currentSequenceForTest(), 2);
	EXPECT_EQ(mission.workItemTypeForTest(), MissionPeer::WorkItemType::WORK_ITEM_TYPE_JOIN_ROUTE);
	EXPECT_TRUE(mission.joinContextForTest().skip_altitude_requirement);
	EXPECT_NEAR(mission.joinContextForTest().projection.alt, vehicle_position.alt, 0.01f);
}

// WHY: Rejoining into a fixed-wing VTOL segment from multicopter state needs an explicit post-join transition.
// WHAT: Mission activation arms JOIN_ROUTE, then reaching the branch-in promotes to TRANSITION_AFTER_JOIN.
TEST_F(NavigatorSmartMissionJoinTest, UsesTransitionAfterJoinForFrontTransition)
{
	setIntParam("MIS_ROUTE_JOIN", 1);
	Navigator navigator;
	MissionPeer mission(&navigator);

	std::vector<mission_item_s> mission_items = {
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon,   0.f, 0.f, kAlt),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 150.f, 0.f, kAlt + 20.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 300.f, 0.f, kAlt + 20.f),
	};

	writeMissionItems(mission_items);
	writeSafePointState(0, 31);
	const mission_s mission_state = makeMissionState(31, 31, mission_items.size(), 2);
	publishMission(mission_state);

	publishVehicleStatus(true, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
	publishLandDetected(false);
	publishGlobalPosition(makePositionFromOffset(kBaseLat, kBaseLon, 60.f, 15.f, kAlt + 5.f));
	publishLocalPosition(0.f, 5.f, 0.f);
	publishHomePosition(makePositionFromOffset(kBaseLat, kBaseLon, -50.f, 0.f, kAlt - 20.f));
	primeNavigatorState(navigator);

	updateRouteCacheUntilReady(navigator, mission_state);
	mission.on_inactive();
	markMissionResultValid(navigator);

	mission.on_activation();
	EXPECT_EQ(mission.joinTransitionActionForTest(), MissionPeer::VtolTransitionAction::FrontTransition);
	EXPECT_EQ(mission.workItemTypeForTest(), MissionPeer::WorkItemType::WORK_ITEM_TYPE_JOIN_ROUTE);

	const MissionRoutePlanner::Position join_projection = mission.joinContextForTest().projection;
	publishGlobalPosition(join_projection);
	publishLocalPosition(0.f, 0.f, 0.f);
	primeNavigatorState(navigator);

	mission.on_active();

	EXPECT_EQ(mission.workItemTypeForTest(), MissionPeer::WorkItemType::WORK_ITEM_TYPE_TRANSITION_AFTER_JOIN);
}
