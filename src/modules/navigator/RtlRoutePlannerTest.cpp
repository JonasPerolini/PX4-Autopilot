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

#include <gtest/gtest.h>

#include "rtl_route_planner.h"

#include <lib/geo/geo.h>
#include <uORB/topics/vtol_vehicle_status.h>

#include <vector>

/** @brief In-memory planner provider backed by mission and safe-point vectors. */
class VectorProvider : public RtlRoutePlanner::Provider
{
public:
	VectorProvider(std::vector<mission_item_s> mission_items, std::vector<mission_item_s> safe_point_items) :
		_mission_items(std::move(mission_items)),
		_safe_point_items(std::move(safe_point_items))
	{
	}

	int missionCount() const override { return static_cast<int>(_mission_items.size()); }

	bool loadMissionItem(int index, mission_item_s &mission_item) const override
	{
		if (index < 0 || index >= missionCount()) {
			return false;
		}

		mission_item = _mission_items[index];
		return true;
	}

	int safePointCount() const override { return static_cast<int>(_safe_point_items.size()); }

	bool loadSafePointItem(int index, mission_item_s &safe_point_item) const override
	{
		if (index < 0 || index >= safePointCount()) {
			return false;
		}

		safe_point_item = _safe_point_items[index];
		return true;
	}

private:
	std::vector<mission_item_s> _mission_items;
	std::vector<mission_item_s> _safe_point_items;
};

/** @brief Provider that counts dataman-style reads so batching behavior can be observed externally. */
class CountingProvider : public RtlRoutePlanner::Provider
{
public:
	CountingProvider(std::vector<mission_item_s> mission_items, std::vector<mission_item_s> safe_point_items) :
		_mission_items(std::move(mission_items)),
		_safe_point_items(std::move(safe_point_items))
	{
	}

	int missionCount() const override { return static_cast<int>(_mission_items.size()); }

	bool loadMissionItem(int index, mission_item_s &mission_item) const override
	{
		++_mission_load_count;

		if (index < 0 || index >= missionCount()) {
			return false;
		}

		mission_item = _mission_items[index];
		return true;
	}

	int safePointCount() const override { return static_cast<int>(_safe_point_items.size()); }

	bool loadSafePointItem(int index, mission_item_s &safe_point_item) const override
	{
		++_safe_point_load_count;

		if (index < 0 || index >= safePointCount()) {
			return false;
		}

		safe_point_item = _safe_point_items[index];
		return true;
	}

	void resetCounters() const
	{
		_mission_load_count = 0;
		_safe_point_load_count = 0;
	}

	int missionLoadCount() const { return _mission_load_count; }
	int safePointLoadCount() const { return _safe_point_load_count; }

private:
	std::vector<mission_item_s> _mission_items;
	std::vector<mission_item_s> _safe_point_items;
	mutable int _mission_load_count{0};
	mutable int _safe_point_load_count{0};
};

/** @brief Build an absolute mission item at the given coordinates. */
static mission_item_s makePositionItem(double lat, double lon, float alt, uint16_t nav_cmd = NAV_CMD_WAYPOINT)
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

/** @brief Build a mission item from a local offset relative to the reference point. */
static mission_item_s makePositionItemFromOffset(double base_lat, double base_lon,
		float north_m, float east_m, float alt, uint16_t nav_cmd = NAV_CMD_WAYPOINT, bool autocontinue = true)
{
	double lat = base_lat;
	double lon = base_lon;
	add_vector_to_global_position(base_lat, base_lon, north_m, east_m, &lat, &lon);

	mission_item_s item = makePositionItem(lat, lon, alt, nav_cmd);
	item.autocontinue = autocontinue;
	return item;
}

/** @brief Build a takeoff mission item with a fixed landing-style altitude. */
static mission_item_s makeTakeoffItemFromOffset(double base_lat, double base_lon,
		float north_m, float east_m, float alt)
{
	return makePositionItemFromOffset(base_lat, base_lon, north_m, east_m, alt, NAV_CMD_TAKEOFF, false);
}

/** @brief Build a land mission item with an absolute altitude. */
static mission_item_s makeLandItemFromOffset(double base_lat, double base_lon,
		float north_m, float east_m, float alt)
{
	return makePositionItemFromOffset(base_lat, base_lon, north_m, east_m, alt, NAV_CMD_LAND, false);
}

/** @brief Build a DO_JUMP item for loop handling tests. */
static mission_item_s makeDoJump(int16_t jump_target_index, uint16_t repeat_count, uint16_t current_count)
{
	mission_item_s item{};
	item.nav_cmd = NAV_CMD_DO_JUMP;
	item.do_jump_mission_index = jump_target_index;
	item.do_jump_repeat_count = repeat_count;
	item.do_jump_current_count = current_count;
	return item;
}

/** @brief Build a VTOL transition command for route-state tests. */
static mission_item_s makeVtolTransitionItem(uint8_t target_state)
{
	mission_item_s item{};
	item.nav_cmd = NAV_CMD_DO_VTOL_TRANSITION;
	item.params[0] = static_cast<float>(target_state);
	return item;
}

/** @brief Build a rally point from a local offset relative to the reference point. */
static mission_item_s makeSafePointFromOffset(double base_lat, double base_lon,
		float north_m, float east_m, float alt)
{
	mission_item_s item = makePositionItemFromOffset(base_lat, base_lon, north_m, east_m, alt, NAV_CMD_RALLY_POINT);
	item.frame = NAV_FRAME_GLOBAL;
	return item;
}

/** @brief Build a route-planner position from a local offset relative to the reference point. */
static RtlRoutePlanner::Position makePositionFromOffset(double base_lat, double base_lon,
		float north_m, float east_m, float alt)
{
	RtlRoutePlanner::Position position{};
	add_vector_to_global_position(base_lat, base_lon, north_m, east_m, &position.lat, &position.lon);
	position.alt = alt;
	return position;
}

/** @brief Default route-planner config for route-following tests. */
static RtlRoutePlanner::Config defaultConfig()
{
	RtlRoutePlanner::Config config{};
	config.vehicle_projection_search_dist = 60.f;
	config.safe_point_projection_search_dist = 60.f;
	config.acceptance_radius = 10.f;
	config.direct_acceptance_radius = 10.f;
	config.home_altitude_amsl = 500.f;
	config.is_multicopter = true;
	return config;
}

TEST(RtlRoutePlannerTest, TransitionActionReverseUsesSegmentEndAnchorForBackTransition)
{
	static constexpr double kBaseLat = 47.397742;
	static constexpr double kBaseLon = 8.545594;
	static constexpr float kAlt = 500.f;

	std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
	};

	VectorProvider provider{mission, {}};
	RtlRoutePlanner planner{provider};
	RtlRoutePlanner::Config config = defaultConfig();
	config.vehicle_is_vtol = true;
	config.vehicle_is_fixed_wing = true;
	config.is_multicopter = false;

	EXPECT_EQ(planner.transitionActionForTargetIndex(2, true, config),
		  RtlRoutePlanner::TransitionAction::BackTransition);
}

TEST(RtlRoutePlannerTest, TransitionActionReverseUsesSegmentEndAnchorForFrontTransition)
{
	static constexpr double kBaseLat = 47.397742;
	static constexpr double kBaseLon = 8.545594;
	static constexpr float kAlt = 500.f;

	std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
	};

	VectorProvider provider{mission, {}};
	RtlRoutePlanner planner{provider};
	RtlRoutePlanner::Config config = defaultConfig();
	config.vehicle_is_vtol = true;
	config.is_multicopter = true;

	EXPECT_EQ(planner.transitionActionForTargetIndex(1, true, config),
		  RtlRoutePlanner::TransitionAction::FrontTransition);
}

TEST(RtlRoutePlannerTest, CollectVehicleProjectionPrefersCurrentMissionSegment)
{
	static constexpr double kBaseLat = 47.397742;
	static constexpr double kBaseLon = 8.545594;
	static constexpr float kAlt = 500.f;

	std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 100.f, kAlt),
	};

	VectorProvider provider{mission, {}};
	RtlRoutePlanner planner{provider};
	RtlRoutePlanner::ProjectionContext projection_context{};
	const RtlRoutePlanner::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 90.f, 10.f, kAlt);

	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, 1, defaultConfig(), projection_context, nullptr));
	EXPECT_EQ(projection_context.projection.segment.start.idx, 0);
	EXPECT_EQ(projection_context.projection.segment.end.idx, 1);
	EXPECT_NEAR(projection_context.projection.dist.xtrack, 10.f, 0.5f);
}

TEST(RtlRoutePlannerTest, CollectVehicleProjectionClampsOutOfRangeMissionIndex)
{
	static constexpr double kBaseLat = 47.397742;
	static constexpr double kBaseLon = 8.545594;
	static constexpr float kAlt = 500.f;

	std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 100.f, kAlt),
	};

	VectorProvider provider{mission, {}};
	RtlRoutePlanner planner{provider};
	const RtlRoutePlanner::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 60.f, kAlt);

	RtlRoutePlanner::ProjectionContext clamped_context{};
	RtlRoutePlanner::ProjectionContext out_of_range_context{};
	RtlRoutePlanner::Config config = defaultConfig();

	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, 0, config, clamped_context, nullptr));
	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, -42, config, out_of_range_context, nullptr));

	EXPECT_EQ(out_of_range_context.projection.segment.start.idx, clamped_context.projection.segment.start.idx);
	EXPECT_EQ(out_of_range_context.projection.segment.end.idx, clamped_context.projection.segment.end.idx);
}

TEST(RtlRoutePlannerTest, SelectSafePointPrefersShortestAlongRoutePath)
{
	static constexpr double kBaseLat = 47.397742;
	static constexpr double kBaseLon = 8.545594;
	static constexpr float kAlt = 500.f;

	std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 100.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 100.f, kAlt),
	};

	std::vector<mission_item_s> safe_points{
		makeSafePointFromOffset(kBaseLat, kBaseLon, 95.f, 80.f, kAlt),
		makeSafePointFromOffset(kBaseLat, kBaseLon, 10.f, 20.f, kAlt),
	};

	VectorProvider provider{mission, safe_points};
	RtlRoutePlanner planner{provider};
	RtlRoutePlanner::ProjectionContext projection_context{};
	const RtlRoutePlanner::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 10.f, 5.f, kAlt);

	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, 1, defaultConfig(), projection_context, nullptr));

	const RtlRoutePlanner::Selection selection = planner.selectSafePoint(projection_context, defaultConfig());

	ASSERT_TRUE(selection.found);
	EXPECT_TRUE(selection.safe_point_found);
	EXPECT_EQ(selection.goal_type, RtlRoutePlanner::GoalType::SafePoint);
	EXPECT_EQ(selection.safe_point_index, 1);
	EXPECT_EQ(selection.branch_off_segment.start.idx, 0);
	EXPECT_EQ(selection.branch_off_segment.end.idx, 1);
}

TEST(RtlRoutePlannerTest, SelectSafePointSupportsDirectToSafePoint)
{
	static constexpr double kBaseLat = 47.397742;
	static constexpr double kBaseLon = 8.545594;
	static constexpr float kAlt = 500.f;

	std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
	};

	std::vector<mission_item_s> safe_points{
		makeSafePointFromOffset(kBaseLat, kBaseLon, 14.f, 3.f, kAlt),
	};

	VectorProvider provider{mission, safe_points};
	RtlRoutePlanner planner{provider};
	RtlRoutePlanner::ProjectionContext projection_context{};
	const RtlRoutePlanner::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 10.f, 0.f, kAlt);

	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, 1, defaultConfig(), projection_context, nullptr));

	const RtlRoutePlanner::Selection selection = planner.selectSafePoint(projection_context, defaultConfig());

	ASSERT_TRUE(selection.found);
	EXPECT_TRUE(selection.direct_to_safe_point);
	EXPECT_EQ(selection.safe_point_index, 0);
}

TEST(RtlRoutePlannerTest, PlanRouteToGoalFallsBackToMissionLandWhenNoSafePointsExist)
{
	static constexpr double kBaseLat = 47.397742;
	static constexpr double kBaseLon = 8.545594;
	static constexpr float kAlt = 500.f;

	std::vector<mission_item_s> mission{
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 300.f, 0.f, kAlt),
	};

	VectorProvider provider{mission, {}};
	RtlRoutePlanner planner{provider};
	RtlRoutePlanner::Plan plan{};
	RtlRoutePlanner::FailureReason failure_reason{RtlRoutePlanner::FailureReason::Unknown};
	RtlRoutePlanner::Config config = defaultConfig();
	config.is_multicopter = false;
	config.vehicle_is_vtol = true;
	config.vehicle_is_fixed_wing = true;

	const RtlRoutePlanner::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 260.f, 0.f, kAlt);

	ASSERT_TRUE(planner.planRouteToGoal(vehicle_position, 2, config, plan, &failure_reason));
	ASSERT_TRUE(plan.valid());

	EXPECT_FALSE(plan.selection.safe_point_found);
	EXPECT_EQ(plan.selection.goal_type, RtlRoutePlanner::GoalType::MissionLand);
	EXPECT_EQ(plan.selection.goal_position.lat, mission.back().lat);
	EXPECT_EQ(plan.selection.goal_position.lon, mission.back().lon);
	EXPECT_FLOAT_EQ(plan.selection.goal_position.alt, mission.back().altitude);
	EXPECT_FALSE(plan.selection.path.direction_reversed);
}

TEST(RtlRoutePlannerTest, PlanRouteToGoalFallsBackToMissionTakeoffWhenThatPathIsShorter)
{
	static constexpr double kBaseLat = 47.397742;
	static constexpr double kBaseLon = 8.545594;
	static constexpr float kAlt = 500.f;

	std::vector<mission_item_s> mission{
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 5100.f, 0.f, kAlt),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 5200.f, 0.f, kAlt),
	};

	VectorProvider provider{mission, {}};
	RtlRoutePlanner planner{provider};
	RtlRoutePlanner::Plan plan{};
	RtlRoutePlanner::FailureReason failure_reason{RtlRoutePlanner::FailureReason::Unknown};
	RtlRoutePlanner::Config config = defaultConfig();
	config.is_multicopter = false;
	config.vehicle_is_vtol = true;
	config.vehicle_is_fixed_wing = true;
	config.vehicle_velocity_valid = true;
	config.vehicle_velocity_north = 15.f;
	config.vehicle_velocity_east = 0.f;

	const RtlRoutePlanner::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 40.f, 0.f, kAlt);

	ASSERT_TRUE(planner.planRouteToGoal(vehicle_position, 0, config, plan, &failure_reason));
	ASSERT_TRUE(plan.valid());

	EXPECT_FALSE(plan.selection.safe_point_found);
	EXPECT_EQ(plan.selection.goal_type, RtlRoutePlanner::GoalType::MissionTakeoff);
	EXPECT_EQ(plan.selection.goal_position.lat, mission.front().lat);
	EXPECT_EQ(plan.selection.goal_position.lon, mission.front().lon);
	EXPECT_FLOAT_EQ(plan.selection.goal_position.alt, mission.front().altitude);
	EXPECT_TRUE(plan.selection.path.direction_reversed);
	EXPECT_TRUE(plan.selection.path.u_turn_required);
	EXPECT_NEAR(plan.selection.path.dist, 4040.f, 5.f);
}

TEST(RtlRoutePlannerTest, PlanRouteToGoalSkipsAltitudeRequirementForLandingInsideAcceptanceRadius)
{
	static constexpr double kBaseLat = 47.397742;
	static constexpr double kBaseLon = 8.545594;
	static constexpr float kAlt = 500.f;

	std::vector<mission_item_s> mission{
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
	};

	VectorProvider provider{mission, {}};
	RtlRoutePlanner planner{provider};
	RtlRoutePlanner::Plan plan{};
	RtlRoutePlanner::FailureReason failure_reason{RtlRoutePlanner::FailureReason::Unknown};
	RtlRoutePlanner::Config config = defaultConfig();
	config.is_multicopter = false;
	config.vehicle_is_vtol = true;
	config.vehicle_is_fixed_wing = true;
	config.acceptance_radius = 20.f;

	const RtlRoutePlanner::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, 523.f);

	ASSERT_TRUE(planner.planRouteToGoal(vehicle_position, 1, config, plan, &failure_reason));
	ASSERT_TRUE(plan.valid());

	EXPECT_EQ(plan.selection.goal_type, RtlRoutePlanner::GoalType::MissionLand);
	EXPECT_TRUE(plan.join_context.skip_altitude_requirement);
	EXPECT_FLOAT_EQ(plan.join_context.projection.alt, vehicle_position.alt);
}

TEST(RtlRoutePlannerTest, CloseToBranchOffSegmentUsesStoredBranchOffLeg)
{
	static constexpr double kBaseLat = 47.397742;
	static constexpr double kBaseLon = 8.545594;
	static constexpr float kAlt = 500.f;

	std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 100.f, kAlt),
	};

	VectorProvider provider{mission, {}};
	RtlRoutePlanner planner{provider};
	RtlRoutePlanner::Selection selection{};
	selection.found = true;
	selection.safe_point_found = true;
	selection.goal_type = RtlRoutePlanner::GoalType::SafePoint;
	selection.path.direction_reversed = false;
	selection.branch_off_segment.start.idx = 1;
	selection.branch_off_segment.start.nav_cmd = NAV_CMD_WAYPOINT;
	selection.branch_off_segment.end.idx = 2;
	selection.branch_off_segment.end.nav_cmd = NAV_CMD_WAYPOINT;
	selection.branch_off_projection = makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 60.f, kAlt);
	selection.safe_point_position = makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 120.f, kAlt);
	selection.goal_position = selection.safe_point_position;

	EXPECT_TRUE(planner.closeToBranchOffSegment(makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 90.f, kAlt),
			selection, 20.f));
	EXPECT_FALSE(planner.closeToBranchOffSegment(makePositionFromOffset(kBaseLat, kBaseLon, 40.f, 0.f, kAlt),
			selection, 20.f));
}

TEST(RtlRoutePlannerTest, SelectSafePointHandlesLoopProjectionAndReverseJumpChoice)
{
	static constexpr double kBaseLat = 47.397742;
	static constexpr double kBaseLon = 8.545594;
	static constexpr float kAlt = 500.f;

	std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 100.f, kAlt),
		makeDoJump(0, 1, 1),
	};

	std::vector<mission_item_s> safe_points{
		makeSafePointFromOffset(kBaseLat, kBaseLon, 95.f, 50.f, kAlt),
	};

	VectorProvider provider{mission, safe_points};
	RtlRoutePlanner planner{provider};
	RtlRoutePlanner::ProjectionContext projection_context{};
	const RtlRoutePlanner::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 50.f, 50.f, kAlt);
	const RtlRoutePlanner::Position loop_start =
		makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 100.f, kAlt);
	const RtlRoutePlanner::Position loop_end =
		makePositionFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);
	const float loop_segment_length = get_distance_to_next_waypoint(loop_start.lat, loop_start.lon,
					  loop_end.lat, loop_end.lon);

	projection_context.vehicle_pos = vehicle_position;
	projection_context.mission_index = 2;
	projection_context.projection.segment.start.idx = 2;
	projection_context.projection.segment.start.nav_cmd = NAV_CMD_WAYPOINT;
	projection_context.projection.segment.end.idx = 0;
	projection_context.projection.segment.end.nav_cmd = NAV_CMD_WAYPOINT;
	projection_context.projection.segment.is_loop = true;
	projection_context.projection.segment_positions.start = loop_start;
	projection_context.projection.segment_positions.end = loop_end;
	projection_context.projection.projection = vehicle_position;
	projection_context.projection.dist.xtrack = 0.f;
	projection_context.projection.dist.along = 200.f + loop_segment_length / 2.f;
	projection_context.projection.dist.segment_length = loop_segment_length;
	projection_context.projection.dist.on_segment = loop_segment_length / 2.f;
	projection_context.loop_ctx.segment = projection_context.projection.segment;
	projection_context.loop_ctx.segment_positions = projection_context.projection.segment_positions;
	projection_context.loop_ctx.along.start = 200.f;
	projection_context.loop_ctx.along.end = 0.f;

	ASSERT_TRUE(projection_context.valid());
	ASSERT_TRUE(projection_context.loop_ctx.valid());

	const RtlRoutePlanner::Selection selection = planner.selectSafePoint(projection_context, defaultConfig());

	ASSERT_TRUE(selection.found);
	EXPECT_EQ(selection.path.first_item_index, 2);
	EXPECT_TRUE(selection.path.direction_reversed);
	EXPECT_EQ(selection.branch_off_segment.start.idx, 1);
	EXPECT_EQ(selection.branch_off_segment.end.idx, 2);
}

TEST(RtlRoutePlannerTest, SelectSafePointScansMissionOnceForBatchedSafePointProjection)
{
	static constexpr double kBaseLat = 47.397742;
	static constexpr double kBaseLon = 8.545594;
	static constexpr float kAlt = 500.f;

	std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 100.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 100.f, kAlt),
	};

	std::vector<mission_item_s> safe_points{
		makeSafePointFromOffset(kBaseLat, kBaseLon, 10.f, -20.f, kAlt),
		makeSafePointFromOffset(kBaseLat, kBaseLon, 40.f, 30.f, kAlt),
		makeSafePointFromOffset(kBaseLat, kBaseLon, 90.f, 50.f, kAlt),
		makeSafePointFromOffset(kBaseLat, kBaseLon, 110.f, 10.f, kAlt),
		makeSafePointFromOffset(kBaseLat, kBaseLon, 70.f, 120.f, kAlt),
		makeSafePointFromOffset(kBaseLat, kBaseLon, 10.f, 110.f, kAlt),
	};

	CountingProvider provider{mission, safe_points};
	RtlRoutePlanner planner{provider};
	RtlRoutePlanner::ProjectionContext projection_context{};
	const RtlRoutePlanner::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 20.f, 5.f, kAlt);

	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, 1, defaultConfig(), projection_context, nullptr));

	provider.resetCounters();
	const RtlRoutePlanner::Selection selection = planner.selectSafePoint(projection_context, defaultConfig());

	ASSERT_TRUE(selection.found);
	EXPECT_LT(provider.missionLoadCount(), 30) << "safe-point projection should scan the mission in one batch";
	EXPECT_EQ(provider.safePointLoadCount(), static_cast<int>(safe_points.size()));
}

TEST(RtlRoutePlannerTest, CollectVehicleProjectionPrefersStoredLoopAnchor)
{
	static constexpr double kBaseLat = 47.397742;
	static constexpr double kBaseLon = 8.545594;
	static constexpr float kAlt = 500.f;

	std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 100.f, kAlt),
		makeDoJump(0, 2, 0),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 100.f, kAlt),
	};

	VectorProvider provider{mission, {}};
	RtlRoutePlanner planner{provider};
	RtlRoutePlanner::ProjectionContext projection_context{};
	RtlRoutePlanner::Config config = defaultConfig();
	config.vehicle_projection_search_dist = 80.f;
	config.last_flown_loop_segment.start.idx = 2;
	config.last_flown_loop_segment.start.nav_cmd = NAV_CMD_WAYPOINT;
	config.last_flown_loop_segment.end.idx = 0;
	config.last_flown_loop_segment.end.nav_cmd = NAV_CMD_WAYPOINT;
	config.last_flown_loop_segment.is_loop = true;
	config.last_flown_loop_segment.loops_remaining = 1;

	const RtlRoutePlanner::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 75.f, 10.f, kAlt);

	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, 1, config, projection_context, nullptr));
	EXPECT_TRUE(projection_context.projection.segment.is_loop);
	EXPECT_EQ(projection_context.projection.segment.start.idx, 2);
	EXPECT_EQ(projection_context.projection.segment.end.idx, 0);
}

TEST(RtlRoutePlannerTest, PlanRouteToGoalFailsWithSingleWaypoint)
{
	static constexpr double kBaseLat = 47.397742;
	static constexpr double kBaseLon = 8.545594;
	static constexpr float kAlt = 500.f;

	std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
	};

	VectorProvider provider{mission, {}};
	RtlRoutePlanner planner{provider};
	RtlRoutePlanner::Plan plan{};
	RtlRoutePlanner::FailureReason failure_reason{RtlRoutePlanner::FailureReason::Unknown};

	const RtlRoutePlanner::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 10.f, 0.f, kAlt);

	EXPECT_FALSE(planner.planRouteToGoal(vehicle_position, 0, defaultConfig(), plan, &failure_reason));
	EXPECT_EQ(failure_reason, RtlRoutePlanner::FailureReason::NoValidWaypoints);
}

TEST(RtlRoutePlannerTest, SelectSafePointReturnsEmptyWhenAllSafePointsInvalid)
{
	static constexpr double kBaseLat = 47.397742;
	static constexpr double kBaseLon = 8.545594;
	static constexpr float kAlt = 500.f;

	std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 100.f, kAlt),
	};

	// Create safe points with unsupported frame (will be rejected by extractSafePointPosition).
	std::vector<mission_item_s> invalid_safe_points;

	for (int i = 0; i < 3; ++i) {
		mission_item_s item{};
		item.nav_cmd = NAV_CMD_RALLY_POINT;
		item.lat = kBaseLat;
		item.lon = kBaseLon;
		item.altitude = kAlt;
		item.frame = 15; // Invalid frame (not a recognized NAV_FRAME_*)
		invalid_safe_points.push_back(item);
	}

	VectorProvider provider{mission, invalid_safe_points};
	RtlRoutePlanner planner{provider};
	RtlRoutePlanner::ProjectionContext projection_context{};
	const RtlRoutePlanner::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 10.f, 0.f, kAlt);

	ASSERT_TRUE(planner.collectVehicleProjection(vehicle_position, 1, defaultConfig(), projection_context, nullptr));

	const RtlRoutePlanner::Selection selection = planner.selectSafePoint(projection_context, defaultConfig());
	EXPECT_FALSE(selection.found);
}

TEST(RtlRoutePlannerTest, PlanRouteToGoalFallsBackWhenAllSafePointsInvalid)
{
	static constexpr double kBaseLat = 47.397742;
	static constexpr double kBaseLon = 8.545594;
	static constexpr float kAlt = 500.f;

	std::vector<mission_item_s> mission{
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
	};

	std::vector<mission_item_s> invalid_safe_points;
	mission_item_s item{};
	item.nav_cmd = NAV_CMD_RALLY_POINT;
	item.lat = kBaseLat;
	item.lon = kBaseLon;
	item.altitude = kAlt;
	item.frame = 15; // Invalid frame (not a recognized NAV_FRAME_*)
	invalid_safe_points.push_back(item);

	VectorProvider provider{mission, invalid_safe_points};
	RtlRoutePlanner planner{provider};
	RtlRoutePlanner::Plan plan{};
	RtlRoutePlanner::FailureReason failure_reason{RtlRoutePlanner::FailureReason::Unknown};

	const RtlRoutePlanner::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 50.f, 0.f, kAlt);

	ASSERT_TRUE(planner.planRouteToGoal(vehicle_position, 1, defaultConfig(), plan, &failure_reason));
	ASSERT_TRUE(plan.valid());

	// Should fall back to mission endpoint since no valid safe points exist.
	EXPECT_FALSE(plan.selection.safe_point_found);
	EXPECT_TRUE(plan.selection.goal_type == RtlRoutePlanner::GoalType::MissionLand
		    || plan.selection.goal_type == RtlRoutePlanner::GoalType::MissionTakeoff);
}

TEST(RtlRoutePlannerTest, SelectSafePointHandlesLoopWithRemainingIterations)
{
	static constexpr double kBaseLat = 47.397742;
	static constexpr double kBaseLon = 8.545594;
	static constexpr float kAlt = 500.f;

	std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 100.f, kAlt),
		makeDoJump(0, 3, 1),  // 2 loops remaining
		makeLandItemFromOffset(kBaseLat, kBaseLon, 200.f, 100.f, kAlt),
	};

	std::vector<mission_item_s> safe_points{
		makeSafePointFromOffset(kBaseLat, kBaseLon, 50.f, 50.f, kAlt),
	};

	VectorProvider provider{mission, safe_points};
	RtlRoutePlanner planner{provider};
	RtlRoutePlanner::Plan plan{};
	RtlRoutePlanner::FailureReason failure_reason{RtlRoutePlanner::FailureReason::Unknown};

	const RtlRoutePlanner::Position vehicle_position =
		makePositionFromOffset(kBaseLat, kBaseLon, 50.f, 0.f, kAlt);

	// planRouteToGoal forces loops_remaining=0 for SRP (line 1759 in rtl_route_planner.cpp),
	// so the planner should still find a valid plan even with pending loop iterations.
	ASSERT_TRUE(planner.planRouteToGoal(vehicle_position, 1, defaultConfig(), plan, &failure_reason));
	ASSERT_TRUE(plan.valid());
	EXPECT_TRUE(plan.selection.found);
}

TEST(RtlRoutePlannerTest, TransitionActionNoneForNonVtol)
{
	static constexpr double kBaseLat = 47.397742;
	static constexpr double kBaseLon = 8.545594;
	static constexpr float kAlt = 500.f;

	std::vector<mission_item_s> mission{
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
	};

	VectorProvider provider{mission, {}};
	RtlRoutePlanner planner{provider};
	RtlRoutePlanner::Config config = defaultConfig();
	// Non-VTOL vehicle should always return None.
	config.vehicle_is_vtol = false;
	config.vehicle_is_fixed_wing = false;
	config.is_multicopter = true;

	EXPECT_EQ(planner.transitionActionForTargetIndex(2, false, config),
		  RtlRoutePlanner::TransitionAction::None);
	EXPECT_EQ(planner.transitionActionForTargetIndex(2, true, config),
		  RtlRoutePlanner::TransitionAction::None);
}

TEST(RtlRoutePlannerTest, PlanRouteToGoalFailsWithEmptyMission)
{
	std::vector<mission_item_s> mission{};
	std::vector<mission_item_s> safe_points{};

	VectorProvider provider{mission, safe_points};
	RtlRoutePlanner planner{provider};
	RtlRoutePlanner::Plan plan{};
	RtlRoutePlanner::FailureReason failure_reason{RtlRoutePlanner::FailureReason::Unknown};

	const RtlRoutePlanner::Position vehicle_position{47.397742, 8.545594, 500.f};

	EXPECT_FALSE(planner.planRouteToGoal(vehicle_position, 0, defaultConfig(), plan, &failure_reason));
	EXPECT_EQ(failure_reason, RtlRoutePlanner::FailureReason::NoValidWaypoints);
}
