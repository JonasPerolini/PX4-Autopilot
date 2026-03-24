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
 * @file test_navigator_route_rejoin_and_rtl.cpp
 *
 * Navigator-level regression tests for mission smart rejoin and RTL type 6.
 * Uses real Mission / RTL classes with dataman-backed mission data so the
 * orchestration path is exercised end to end.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#include <gtest/gtest.h>

#include <dataman_client/DatamanClient.hpp>
#include <parameters/param.h>
#include <px4_platform_common/px4_work_queue/WorkQueueManager.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/home_position.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/rtl_status.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

#include <unistd.h>

#include "mission.h"
#include "navigator.h"
#include "rtl.h"
#include "test_RTL_helpers.h"

extern "C" __EXPORT int dataman_main(int argc, char *argv[]);

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
	VtolTransitionAction joinTransitionActionForTest() const { return _join_transition_action; }
};

class NavigatorRouteRejoinAndRtlTest : public ::testing::Test
{
protected:
	static void SetUpTestSuite()
	{
		param_control_autosave(false);
		px4::WorkQueueManagerStart();
		char start[] = "start";
		char ram[] = "-r";
		char name[] = "dataman";
		char *argv[] = {name, start, ram};
		dataman_main(3, argv);
	}

	static void TearDownTestSuite()
	{
		param_control_autosave(true);
		char stop[] = "stop";
		char name[] = "dataman";
		char *argv[] = {name, stop};
		dataman_main(2, argv);
		px4::WorkQueueManagerStop();
	}

	static constexpr double kBaseLat = 47.397742;
	static constexpr double kBaseLon = 8.545594;
	static constexpr float kBaseAlt = 500.f;

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

	void writeSafePointState(uint16_t num_items, uint32_t opaque_id = 1, dm_item_t dataman_id = DM_KEY_SAFE_POINTS_0)
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

	void publishLocalPosition(float heading_rad = 0.f, float vx = 0.f, float vy = 0.f)
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

	void updateRouteCacheUntilReady(Navigator &navigator, const mission_s &mission)
	{
		MissionRouteCache *route_cache = navigator.get_mission_route_cache();
		ASSERT_NE(route_cache, nullptr);

		for (int i = 0; i < 200; ++i) {
			route_cache->update(mission);

			if (route_cache->isReady(mission) && route_cache->safePointsReady()) {
				return;
			}

			usleep(1000);
		}

		FAIL() << "MissionRouteCache did not become ready";
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

TEST_F(NavigatorRouteRejoinAndRtlTest, MissionSmartRejoinUsesShortestLoopExit)
{
	setIntParam("MIS_ROUTE_JOIN", 1);
	Navigator navigator;
	MissionPeer mission(&navigator);

	std::vector<mission_item_s> mission_items = {
		makePositionItemFromOffset(kBaseLat, kBaseLon,   0.f,   0.f, kBaseAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f,   0.f, kBaseAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 100.f, kBaseAlt),
		makeDoJump(0, 2, 2),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 200.f,   0.f, kBaseAlt - 10.f),
	};

	writeMissionItems(mission_items);
	writeSafePointState(0, 11);

	mission_s mission_state{};
	mission_state.timestamp = hrt_absolute_time();
	mission_state.current_seq = 0;
	mission_state.land_start_index = 4;
	mission_state.land_index = 4;
	mission_state.mission_id = 21;
	mission_state.safe_points_id = 11;
	mission_state.count = static_cast<uint16_t>(mission_items.size());
	mission_state.mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0;
	mission_state.fence_dataman_id = DM_KEY_FENCE_POINTS_0;
	mission_state.safepoint_dataman_id = DM_KEY_SAFE_POINTS_0;
	publishMission(mission_state);

	publishVehicleStatus(false, vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
	publishLandDetected(false);
	publishGlobalPosition(makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 95.f, kBaseAlt));
	publishLocalPosition(0.f, 12.f, 0.f);
	publishHomePosition(makePositionFromOffset(kBaseLat, kBaseLon, -200.f, 0.f, kBaseAlt));
	primeNavigatorState(navigator);

	updateRouteCacheUntilReady(navigator, mission_state);
	mission.on_inactive();

	ASSERT_TRUE(mission.trySetRouteJoinOnActivation(false));
	EXPECT_EQ(mission.currentSequenceForTest(), 2);
	EXPECT_EQ(mission.workItemTypeForTest(), MissionPeer::WorkItemType::WORK_ITEM_TYPE_JOIN_ROUTE);
	EXPECT_TRUE(mission.joinContextForTest().valid());
	EXPECT_EQ(mission.joinTransitionActionForTest(), MissionPeer::VtolTransitionAction::None);
}

TEST_F(NavigatorRouteRejoinAndRtlTest, RtlType6PlanningFailureFallsBackToDirectMissionLand)
{
	setIntParam("RTL_TYPE", 6);
	Navigator navigator;
	RTL rtl(&navigator);
	uORB::SubscriptionData<rtl_status_s> rtl_status_sub{ORB_ID(rtl_status)};

	std::vector<mission_item_s> mission_items = {
		makePositionItem(0.0, 0.0, kBaseAlt),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 120.f, 0.f, kBaseAlt - 10.f),
	};

	writeMissionItems(mission_items);
	writeSafePointState(0, 12);

	mission_s mission_state{};
	mission_state.timestamp = hrt_absolute_time();
	mission_state.current_seq = 1;
	mission_state.land_start_index = 1;
	mission_state.land_index = 1;
	mission_state.mission_id = 22;
	mission_state.safe_points_id = 12;
	mission_state.count = static_cast<uint16_t>(mission_items.size());
	mission_state.mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0;
	mission_state.fence_dataman_id = DM_KEY_FENCE_POINTS_0;
	mission_state.safepoint_dataman_id = DM_KEY_SAFE_POINTS_0;
	publishMission(mission_state);

	publishVehicleStatus(false, vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
	publishLandDetected(false);
	publishGlobalPosition(makePositionFromOffset(kBaseLat, kBaseLon, 115.f, 0.f, kBaseAlt));
	publishLocalPosition(0.f, 0.f, 0.f);
	publishHomePosition(makePositionFromOffset(kBaseLat, kBaseLon, -1000.f, 0.f, kBaseAlt));
	primeNavigatorState(navigator);
	*navigator.get_mission_result() = mission_result_s{};
	navigator.get_mission_result()->valid = true;

	updateRouteCacheUntilReady(navigator, mission_state);
	rtl.on_inactive();

	ASSERT_TRUE(rtl_status_sub.update());
	const rtl_status_s &rtl_status = rtl_status_sub.get();
	EXPECT_EQ(rtl_status.rtl_type, rtl_status_s::RTL_STATUS_TYPE_DIRECT_MISSION_LAND);
	EXPECT_EQ(rtl_status.safe_point_index, UINT8_MAX);
}
