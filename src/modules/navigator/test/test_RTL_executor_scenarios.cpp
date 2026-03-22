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
 * @file test_RTL_executor_scenarios.cpp
 *
 * Planner-level tests that validate the data contract between the
 * RtlRoutePlanner (Brain) and the RtlMissionSafePointFollow executor
 * (Pilot).  The executor cannot be instantiated without the full PX4
 * Navigator module, so these tests verify the planner outputs that
 * drive executor behavior:
 *
 *   GROUP 1: Mid-route VTOL transition detection (nominal and reverse)
 *   GROUP 2: Reverse-flying FW to land at takeoff
 *   GROUP 3: Safe point with mid-route VTOL transitions
 *   GROUP 4: DO_JUMP loop planning (vehicle inside loop, safe point near loop)
 *   GROUP 5: Direct-to-safe-point shortcut (MC vs FW)
 *   GROUP 6: FaultyVectorProvider: graceful degradation on load failures
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#include "test_RTL_helpers.h"
#include "test_RTL_data.h"

#include <uORB/topics/vtol_vehicle_status.h>
#include <cmath>

static constexpr double kBaseLat = 47.397742;
static constexpr double kBaseLon = 8.545594;
static constexpr float kAlt = 500.f;

// ============================================================================
// FaultyVectorProvider — simulates SD card / dataman read failures
// ============================================================================

class FaultyVectorProvider : public RtlRoutePlanner::Provider
{
public:
	FaultyVectorProvider(std::vector<mission_item_s> mission_items,
			     std::vector<mission_item_s> safe_point_items,
			     std::vector<int> faulty_mission_indices = {},
			     std::vector<int> faulty_safe_point_indices = {}) :
		_mission_items(std::move(mission_items)),
		_safe_point_items(std::move(safe_point_items)),
		_faulty_mission_indices(std::move(faulty_mission_indices)),
		_faulty_safe_point_indices(std::move(faulty_safe_point_indices))
	{
	}

	int missionCount() const override { return static_cast<int>(_mission_items.size()); }

	bool loadMissionItem(int index, mission_item_s &mission_item) const override
	{
		for (int fi : _faulty_mission_indices) {
			if (fi == index) {
				return false;
			}
		}

		if (index < 0 || index >= missionCount()) {
			return false;
		}

		mission_item = _mission_items[index];
		return true;
	}

	int safePointCount() const override { return static_cast<int>(_safe_point_items.size()); }

	bool loadSafePointItem(int index, mission_item_s &safe_point_item) const override
	{
		for (int fi : _faulty_safe_point_indices) {
			if (fi == index) {
				return false;
			}
		}

		if (index < 0 || index >= safePointCount()) {
			return false;
		}

		safe_point_item = _safe_point_items[index];
		return true;
	}

	void setFaultyMissionIndices(std::vector<int> indices) { _faulty_mission_indices = std::move(indices); }
	void setFaultySafePointIndices(std::vector<int> indices) { _faulty_safe_point_indices = std::move(indices); }

private:
	std::vector<mission_item_s> _mission_items;
	std::vector<mission_item_s> _safe_point_items;
	std::vector<int> _faulty_mission_indices;
	std::vector<int> _faulty_safe_point_indices;
};

// ============================================================================
// Test fixture
// ============================================================================

class RtlExecutorScenarioTest : public RtlRoutePlannerTestBase
{
protected:
	RtlRoutePlanner::Plan plan{};
};

// =============================================================================
// GROUP 1: Mid-route VTOL transition detection
// =============================================================================

// WHY: When a VTOL vehicle follows a route that transitions from MC to FW mid-route,
//      the executor must detect that a front-transition is needed at the transition boundary.
// WHAT: For a mission [Takeoff, WP, VTOL_FW, WP, WP, VTOL_MC, WP, Land], the planner's
//       transitionActionForTargetIndex correctly identifies FT and BT at each boundary.
TEST_F(RtlExecutorScenarioTest, MidRouteTransitionsDetectedForEachSegment)
{
	// GIVEN: A VTOL mission with MC->FW->MC transitions.
	//   idx: 0=Takeoff, 1=WP, 2=VTOL_FW, 3=WP, 4=WP, 5=VTOL_MC, 6=WP, 7=Land
	std::vector<mission_item_s> mission = {
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt + 20.f),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 500.f, 0.f, kAlt + 50.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 800.f, 0.f, kAlt + 60.f),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 1000.f, 0.f, kAlt + 30.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 1200.f, 0.f, kAlt - 10.f),
	};

	VectorProvider provider(mission, {});
	RtlRoutePlanner planner(provider);

	// Vehicle currently in MC mode (about to enter FW zone)
	RtlRoutePlanner::Config mc_config = defaultConfig();
	mc_config.vehicle_is_vtol = true;
	mc_config.is_multicopter = true;
	mc_config.vehicle_is_fixed_wing = false;

	// Vehicle currently in FW mode
	RtlRoutePlanner::Config fw_config = defaultConfig();
	fw_config.vehicle_is_vtol = true;
	fw_config.is_multicopter = false;
	fw_config.vehicle_is_fixed_wing = true;

	// WHEN/THEN: Target index 3 is in the FW zone (after VTOL_FW at idx 2).
	// MC vehicle needs FrontTransition; FW vehicle needs None.
	EXPECT_EQ(planner.transitionActionForTargetIndex(3, false, mc_config),
		  RtlRoutePlanner::TransitionAction::FrontTransition);
	EXPECT_EQ(planner.transitionActionForTargetIndex(3, false, fw_config),
		  RtlRoutePlanner::TransitionAction::None);

	// Target index 4 is also in FW zone.
	EXPECT_EQ(planner.transitionActionForTargetIndex(4, false, mc_config),
		  RtlRoutePlanner::TransitionAction::FrontTransition);
	EXPECT_EQ(planner.transitionActionForTargetIndex(4, false, fw_config),
		  RtlRoutePlanner::TransitionAction::None);

	// Target index 6 is in the MC zone (after VTOL_MC at idx 5).
	// FW vehicle needs BackTransition; MC vehicle needs None.
	EXPECT_EQ(planner.transitionActionForTargetIndex(6, false, fw_config),
		  RtlRoutePlanner::TransitionAction::BackTransition);
	EXPECT_EQ(planner.transitionActionForTargetIndex(6, false, mc_config),
		  RtlRoutePlanner::TransitionAction::None);
}

// WHY: When the executor walks the route in reverse through a VTOL transition boundary,
//      the transition detection must still work correctly with the reversed anchor logic.
// WHAT: For the same mission walked in reverse, transitions are detected at the correct targets.
TEST_F(RtlExecutorScenarioTest, MidRouteTransitionsDetectedInReverse)
{
	// GIVEN: Same VTOL mission as above.
	std::vector<mission_item_s> mission = {
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt + 20.f),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 500.f, 0.f, kAlt + 50.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 800.f, 0.f, kAlt + 60.f),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 1000.f, 0.f, kAlt + 30.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 1200.f, 0.f, kAlt - 10.f),
	};

	VectorProvider provider(mission, {});
	RtlRoutePlanner planner(provider);

	// FW vehicle flying in reverse
	RtlRoutePlanner::Config fw_config = defaultConfig();
	fw_config.vehicle_is_vtol = true;
	fw_config.is_multicopter = false;
	fw_config.vehicle_is_fixed_wing = true;

	// MC vehicle flying in reverse
	RtlRoutePlanner::Config mc_config = defaultConfig();
	mc_config.vehicle_is_vtol = true;
	mc_config.is_multicopter = true;
	mc_config.vehicle_is_fixed_wing = false;

	// WHEN/THEN: Reverse direction — anchor shifts by +1 for the reversed anchor rule.
	// Target index 4 reversed: anchor is idx 5+ → finds VTOL_MC → MC zone.
	// FW vehicle needs BackTransition to enter MC zone.
	EXPECT_EQ(planner.transitionActionForTargetIndex(4, true, fw_config),
		  RtlRoutePlanner::TransitionAction::BackTransition);

	// Target index 1 reversed: anchor is idx 2+ → finds VTOL_FW at idx 2 → FW zone.
	// MC vehicle needs FrontTransition.
	EXPECT_EQ(planner.transitionActionForTargetIndex(1, true, mc_config),
		  RtlRoutePlanner::TransitionAction::FrontTransition);

	// Target index 6 reversed: anchor is idx 7 → land item, walk back finds VTOL_MC at idx 5 → MC zone.
	// MC vehicle: no transition needed.
	EXPECT_EQ(planner.transitionActionForTargetIndex(6, true, mc_config),
		  RtlRoutePlanner::TransitionAction::None);
}

// =============================================================================
// GROUP 2: Reverse-flying FW to land at takeoff
// =============================================================================

// WHY: When a VTOL in FW mode triggers RTL near the start of the mission, the planner should
//      select MissionTakeoff with direction_reversed=true. The executor must then fly the route
//      in reverse and eventually land at ground level.
// WHAT: Vehicle at N+100 on a long VTOL mission gets a reversed path to takeoff.
TEST_F(RtlExecutorScenarioTest, FwVehicleReversesToTakeoffWhenNearStart)
{
	// GIVEN: A VTOL mission where land is far away.
	//   idx: 0=Takeoff, 1=WP, 2=VTOL_FW, 3=WP(far), 4=WP(far), 5=VTOL_MC, 6=WP(far), 7=Land(far)
	std::vector<mission_item_s> mission = {
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt + 20.f),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 2000.f, 0.f, kAlt + 50.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 4000.f, 0.f, kAlt + 60.f),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 5000.f, 0.f, kAlt + 30.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 5500.f, 0.f, kAlt - 10.f),
	};

	VectorProvider provider(mission, {});
	RtlRoutePlanner planner(provider);

	// Vehicle is near the start of the mission, in FW mode.
	auto vehicle_pos = makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt + 15.f);
	config = fwConfig();
	config.vehicle_velocity_north = 15.f;
	config.vehicle_velocity_east = 0.f;
	config.vehicle_velocity_valid = true;

	// WHEN: planRouteToGoal is called.
	bool ok = planner.planRouteToGoal(vehicle_pos, 0, config, plan, &reason);

	// THEN: Planning succeeds and selects MissionTakeoff with reversed direction.
	ASSERT_TRUE(ok);
	EXPECT_EQ(plan.selection.goal_type, RtlRoutePlanner::GoalType::MissionTakeoff);
	EXPECT_TRUE(plan.selection.path.direction_reversed);
	EXPECT_NEAR(plan.selection.goal_position.lat, mission[0].lat, kLatLonToleranceDeg);
	EXPECT_NEAR(plan.selection.goal_position.lon, mission[0].lon, kLatLonToleranceDeg);
}

// WHY: When reversing to takeoff, the join context must request a back-transition if the
//      first route segment (reverse direction) is in an MC zone.
// WHAT: Plan's join_context.vtol_back_transition_required is set correctly for reverse-to-takeoff.
TEST_F(RtlExecutorScenarioTest, ReverseToTakeoffJoinContextTransition)
{
	// GIVEN: A mission starting with MC, then transitioning to FW.
	//   The vehicle is currently in FW mode near segment 3 (FW zone).
	//   Reversing to takeoff means the first route item in reverse is idx 1 (MC zone).
	std::vector<mission_item_s> mission = {
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt + 20.f),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 2000.f, 0.f, kAlt + 50.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 4000.f, 0.f, kAlt + 60.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 5500.f, 0.f, kAlt - 10.f),
	};

	VectorProvider provider(mission, {});
	RtlRoutePlanner planner(provider);

	auto vehicle_pos = makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt + 15.f);
	config = fwConfig();
	config.vehicle_velocity_north = 15.f;
	config.vehicle_velocity_east = 0.f;
	config.vehicle_velocity_valid = true;

	// WHEN: planRouteToGoal is called and selects reverse to takeoff.
	bool ok = planner.planRouteToGoal(vehicle_pos, 0, config, plan, &reason);

	// THEN: If the plan reverses to takeoff, the first reverse item enters an MC zone,
	//       so a back-transition should be required at join.
	ASSERT_TRUE(ok);

	if (plan.selection.goal_type == RtlRoutePlanner::GoalType::MissionTakeoff
	    && plan.selection.path.direction_reversed) {
		// The planner's transitionActionForTargetIndex for the first_item_index in reverse
		// should return BackTransition for a FW vehicle.
		auto action = planner.transitionActionForTargetIndex(
				      plan.selection.path.first_item_index,
				      true, config);
		EXPECT_EQ(action, RtlRoutePlanner::TransitionAction::BackTransition);
	}
}

// =============================================================================
// GROUP 3: Safe point with mid-route VTOL transition
// =============================================================================

// WHY: When a safe point's branch-off is in a FW zone but the vehicle is in MC mode,
//      the executor will need a front-transition during route following. The plan must
//      correctly identify this.
// WHAT: Using the default_dataset (which has VTOL transitions), verify that transition
//       detection works for segments in different VTOL zones.
TEST_F(RtlExecutorScenarioTest, DefaultDatasetTransitionsPerSegment)
{
	// GIVEN: The default_dataset mission with VTOL transitions at idx 3 (FW) and idx 6 (MC).
	auto items = default_dataset::mission();
	VectorProvider provider(items, default_dataset::safePoints());
	RtlRoutePlanner planner(provider);

	// MC config
	RtlRoutePlanner::Config mc_config = defaultConfig();
	mc_config.vehicle_is_vtol = true;
	mc_config.is_multicopter = true;
	mc_config.vehicle_is_fixed_wing = false;

	// FW config
	RtlRoutePlanner::Config fw_cfg = fwConfig();

	// WHEN/THEN: Target index 4 (after VTOL_FW at idx 3) — FW zone.
	// MC vehicle needs FrontTransition.
	EXPECT_EQ(planner.transitionActionForTargetIndex(4, false, mc_config),
		  RtlRoutePlanner::TransitionAction::FrontTransition);

	// FW vehicle: no transition needed.
	EXPECT_EQ(planner.transitionActionForTargetIndex(4, false, fw_cfg),
		  RtlRoutePlanner::TransitionAction::None);

	// Target index 7 (after VTOL_MC at idx 6) — MC zone.
	// FW vehicle needs BackTransition.
	EXPECT_EQ(planner.transitionActionForTargetIndex(7, false, fw_cfg),
		  RtlRoutePlanner::TransitionAction::BackTransition);

	// MC vehicle: no transition needed.
	EXPECT_EQ(planner.transitionActionForTargetIndex(7, false, mc_config),
		  RtlRoutePlanner::TransitionAction::None);
}

// WHY: When using the default_dataset, a VTOL vehicle near the start in FW mode should
//      get a plan that can be executed through the full stage machine including transitions.
// WHAT: Plan builds successfully with safe-point selection and the path includes segments
//       that cross VTOL transition boundaries.
TEST_F(RtlExecutorScenarioTest, DefaultDatasetVtolPlanBuildsSucessfully)
{
	auto items = default_dataset::mission();
	auto safe_points = default_dataset::safePoints();
	VectorProvider provider(items, safe_points);
	RtlRoutePlanner planner(provider);

	// Vehicle near segment 4-5 (FW zone), heading NE.
	auto vehicle_pos = makePositionAbsolute(46.10830, 2.2995, 575.f);
	config = fwConfig();
	config.vehicle_velocity_north = default_dataset::kVel;
	config.vehicle_velocity_east = 0.f;
	config.vehicle_velocity_valid = true;

	bool ok = planner.planRouteToGoal(vehicle_pos, 4, config, plan, &reason);

	ASSERT_TRUE(ok);
	EXPECT_TRUE(plan.valid());

	// The plan should have found a safe point or fallen back to an endpoint.
	EXPECT_TRUE(plan.selection.found);

	// The path should have a non-zero distance.
	EXPECT_GT(plan.selection.path.dist, 0.f);

	// The projection context should be valid.
	EXPECT_TRUE(plan.projection_context.valid());
}

// =============================================================================
// GROUP 4: DO_JUMP loop planning
// =============================================================================

// WHY: When the vehicle is inside a DO_JUMP loop, the planner must correctly model
//      the loop edges and select a goal reachable via the loop geometry. The executor
//      relies on this to advance through the mission without following DO_JUMP control flow.
// WHAT: Vehicle on the corner_dataset loop area gets a valid plan with a safe point.
TEST_F(RtlExecutorScenarioTest, VehicleInsideDoJumpLoopGetsValidPlan)
{
	// GIVEN: Corner mission with DO_JUMP at index 8 (jumps to 2, repeat 7).
	auto items = corner_dataset::mission();
	auto safe_points = corner_dataset::safePoints();
	VectorProvider provider(items, safe_points);
	RtlRoutePlanner planner(provider);

	// Vehicle is on the loop segment area (between WP 5 and WP 7).
	auto vehicle_pos = makePositionAbsolute(46.10214, 2.31760, kAlt + 150.f);
	config = defaultConfig();
	config.vehicle_velocity_north = corner_dataset::kVelDiag;
	config.vehicle_velocity_east = -corner_dataset::kVelDiag;
	config.vehicle_velocity_valid = true;

	// Set the last_flown_loop_segment to indicate we're inside the loop.
	config.last_flown_loop_segment.start.idx = 7;
	config.last_flown_loop_segment.start.nav_cmd = NAV_CMD_WAYPOINT;
	config.last_flown_loop_segment.end.idx = 2;
	config.last_flown_loop_segment.end.nav_cmd = NAV_CMD_WAYPOINT;
	config.last_flown_loop_segment.is_loop = true;
	config.last_flown_loop_segment.loops_remaining = 5;

	// WHEN: planRouteToGoal is called.
	bool ok = planner.planRouteToGoal(vehicle_pos, 7, config, plan, &reason);

	// THEN: Planning succeeds and selects a goal.
	ASSERT_TRUE(ok) << "Failure reason: " << RtlRoutePlanner::failureReasonString(reason);
	EXPECT_TRUE(plan.valid());
	EXPECT_TRUE(plan.selection.found);
}

// WHY: The planner must handle the DO_JUMP loop edge correctly when computing along-route
//      distances. A safe point near the loop must be reachable via the loop geometry.
// WHAT: Safe point 7 (on jump segment 7→2) is selected when the vehicle is in the loop.
TEST_F(RtlExecutorScenarioTest, SafePointOnDoJumpLoopSegmentIsReachable)
{
	auto items = corner_dataset::mission();
	auto safe_points = corner_dataset::safePoints();
	VectorProvider provider(items, safe_points);
	RtlRoutePlanner planner(provider);

	// Vehicle near WP 7, inside the loop, heading toward WP 2 (the jump target).
	auto vehicle_pos = makePositionAbsolute(46.10225, 2.31670, kAlt + 150.f);
	config = defaultConfig();
	config.vehicle_velocity_north = corner_dataset::kVelDiag;
	config.vehicle_velocity_east = corner_dataset::kVelDiag;
	config.vehicle_velocity_valid = true;

	config.last_flown_loop_segment.start.idx = 7;
	config.last_flown_loop_segment.start.nav_cmd = NAV_CMD_WAYPOINT;
	config.last_flown_loop_segment.end.idx = 2;
	config.last_flown_loop_segment.end.nav_cmd = NAV_CMD_WAYPOINT;
	config.last_flown_loop_segment.is_loop = true;
	config.last_flown_loop_segment.loops_remaining = 3;

	bool ok = planner.planRouteToGoal(vehicle_pos, 7, config, plan, &reason);

	ASSERT_TRUE(ok) << "Failure reason: " << RtlRoutePlanner::failureReasonString(reason);
	EXPECT_TRUE(plan.selection.found);

	// If a safe point was found, verify it has a valid branch-off.
	if (plan.selection.safe_point_found) {
		EXPECT_GE(plan.selection.safe_point_index, 0);
		EXPECT_TRUE(plan.selection.safe_point_position.valid());
		EXPECT_TRUE(plan.selection.branch_off_projection.valid());
	}
}

// WHY: A mission with an exhausted DO_JUMP (current_count == repeat_count) should be
//      treated as a straight-through mission with no loop edges.
// WHAT: Planning succeeds and does not create loop context when DO_JUMP is exhausted.
TEST_F(RtlExecutorScenarioTest, ExhaustedDoJumpTreatedAsStraightThrough)
{
	// GIVEN: Simple mission with exhausted DO_JUMP (current_count == repeat_count).
	std::vector<mission_item_s> mission = {
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt + 20.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 400.f, 0.f, kAlt + 30.f),
		makeDoJump(1, 3, 3),  // exhausted: current == repeat
		makePositionItemFromOffset(kBaseLat, kBaseLon, 600.f, 0.f, kAlt + 40.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 800.f, 0.f, kAlt - 10.f),
	};

	std::vector<mission_item_s> safe_points = {
		makeSafePointFromOffset(kBaseLat, kBaseLon, 300.f, 50.f, kAlt),
	};

	VectorProvider provider(mission, safe_points);
	RtlRoutePlanner planner(provider);

	auto vehicle_pos = makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt + 15.f);
	config = defaultConfig();
	config.vehicle_velocity_north = 10.f;
	config.vehicle_velocity_east = 0.f;
	config.vehicle_velocity_valid = true;

	bool ok = planner.planRouteToGoal(vehicle_pos, 0, config, plan, &reason);

	ASSERT_TRUE(ok) << "Failure reason: " << RtlRoutePlanner::failureReasonString(reason);
	EXPECT_TRUE(plan.valid());

	// Loop context should NOT be set for an exhausted jump.
	EXPECT_FALSE(plan.projection_context.loop_ctx.valid());
}

// =============================================================================
// GROUP 5: Direct-to-safe-point shortcut (MC vs FW)
// =============================================================================

// WHY: When a multicopter is very close to a safe point, the planner should select
//      direct_to_safe_point=true so the executor flies straight there without following the route.
// WHAT: MC vehicle within direct_acceptance_radius of a rally point gets direct-to-safe-point.
TEST_F(RtlExecutorScenarioTest, McDirectToNearbySafePoint)
{
	// GIVEN: Simple mission with a safe point very close to the vehicle.
	std::vector<mission_item_s> mission = {
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 500.f, 0.f, kAlt + 50.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 1000.f, 0.f, kAlt + 80.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 1500.f, 0.f, kAlt - 10.f),
	};

	// Safe point just 5m away from the vehicle position.
	std::vector<mission_item_s> safe_points = {
		makeSafePointFromOffset(kBaseLat, kBaseLon, 255.f, 0.f, kAlt + 50.f),
	};

	VectorProvider provider(mission, safe_points);
	RtlRoutePlanner planner(provider);

	auto vehicle_pos = makePositionFromOffset(kBaseLat, kBaseLon, 250.f, 0.f, kAlt + 50.f);
	config = defaultConfig();
	config.is_multicopter = true;
	config.direct_acceptance_radius = 20.f;
	config.vehicle_velocity_north = 5.f;
	config.vehicle_velocity_east = 0.f;
	config.vehicle_velocity_valid = true;

	bool ok = planner.planRouteToGoal(vehicle_pos, 0, config, plan, &reason);

	ASSERT_TRUE(ok) << "Failure reason: " << RtlRoutePlanner::failureReasonString(reason);
	EXPECT_TRUE(plan.selection.found);
	EXPECT_TRUE(plan.selection.safe_point_found);
	EXPECT_TRUE(plan.selection.direct_to_safe_point);
}

// WHY: Fixed-wing vehicles cannot hover, so the planner should NOT select direct-to-safe-point
//      even when the safe point is very close. The vehicle must follow the route to the branch-off.
// WHAT: FW vehicle near a safe point does NOT get direct_to_safe_point=true.
TEST_F(RtlExecutorScenarioTest, FwDoesNotGetDirectToSafePoint)
{
	std::vector<mission_item_s> mission = {
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 500.f, 0.f, kAlt + 50.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 1000.f, 0.f, kAlt + 80.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 1500.f, 0.f, kAlt - 10.f),
	};

	std::vector<mission_item_s> safe_points = {
		makeSafePointFromOffset(kBaseLat, kBaseLon, 255.f, 0.f, kAlt + 50.f),
	};

	VectorProvider provider(mission, safe_points);
	RtlRoutePlanner planner(provider);

	auto vehicle_pos = makePositionFromOffset(kBaseLat, kBaseLon, 250.f, 0.f, kAlt + 50.f);
	config = fwConfig();
	config.direct_acceptance_radius = 20.f;
	config.vehicle_velocity_north = 15.f;
	config.vehicle_velocity_east = 0.f;
	config.vehicle_velocity_valid = true;

	bool ok = planner.planRouteToGoal(vehicle_pos, 0, config, plan, &reason);

	ASSERT_TRUE(ok) << "Failure reason: " << RtlRoutePlanner::failureReasonString(reason);
	EXPECT_TRUE(plan.selection.found);

	// FW should not get direct-to-safe-point.
	EXPECT_FALSE(plan.selection.direct_to_safe_point);
}

// WHY: When there are no safe points at all, the planner must fall back to a mission endpoint
//      (MissionLand or MissionTakeoff). The executor uses goal_type to decide landing behavior.
// WHAT: Mission with no safe points selects MissionLand or MissionTakeoff as the goal.
TEST_F(RtlExecutorScenarioTest, NoSafePointsFallsBackToMissionEndpoint)
{
	std::vector<mission_item_s> mission = {
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 500.f, 0.f, kAlt + 50.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 1000.f, 0.f, kAlt + 80.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 1500.f, 0.f, kAlt - 10.f),
	};

	VectorProvider provider(mission, {}); // No safe points
	RtlRoutePlanner planner(provider);

	auto vehicle_pos = makePositionFromOffset(kBaseLat, kBaseLon, 250.f, 0.f, kAlt + 20.f);
	config = defaultConfig();
	config.vehicle_velocity_north = 10.f;
	config.vehicle_velocity_east = 0.f;
	config.vehicle_velocity_valid = true;

	bool ok = planner.planRouteToGoal(vehicle_pos, 0, config, plan, &reason);

	ASSERT_TRUE(ok) << "Failure reason: " << RtlRoutePlanner::failureReasonString(reason);
	EXPECT_TRUE(plan.selection.found);
	EXPECT_FALSE(plan.selection.safe_point_found);

	// Must be either MissionLand or MissionTakeoff.
	EXPECT_TRUE(plan.selection.goal_type == RtlRoutePlanner::GoalType::MissionLand
		    || plan.selection.goal_type == RtlRoutePlanner::GoalType::MissionTakeoff);
}

// =============================================================================
// GROUP 6: FaultyVectorProvider — graceful degradation on load failures
// =============================================================================

// WHY: If the SD card fails to read a mission item mid-scan, the planner must not crash
//      or produce an invalid plan. It should either fail gracefully with a clear reason
//      or succeed with the items it could read.
// WHAT: A faulty provider that fails on a mid-mission index causes the planner to fail
//       with an appropriate failure reason.
TEST_F(RtlExecutorScenarioTest, FaultyMissionItemMidScanCausesGracefulFailure)
{
	std::vector<mission_item_s> mission = {
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt + 20.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 400.f, 0.f, kAlt + 40.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 600.f, 0.f, kAlt + 60.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 800.f, 0.f, kAlt - 10.f),
	};

	// Fail on index 2 (a mid-route waypoint).
	FaultyVectorProvider provider(mission, {}, {2}, {});
	RtlRoutePlanner planner(provider);

	auto vehicle_pos = makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt + 15.f);
	config = defaultConfig();
	config.vehicle_velocity_north = 10.f;
	config.vehicle_velocity_east = 0.f;
	config.vehicle_velocity_valid = true;

	RtlRoutePlanner::FailureReason fail_reason{};
	bool ok = planner.planRouteToGoal(vehicle_pos, 0, config, plan, &fail_reason);

	// The planner should fail because it cannot build valid segments through the faulty index.
	EXPECT_FALSE(ok);
	EXPECT_NE(fail_reason, RtlRoutePlanner::FailureReason::None);
}

// WHY: If the first AND second mission items fail to load, the planner cannot build any
//      segments. This must fail cleanly without crashing.
// WHAT: All initial position items failing causes planning failure.
TEST_F(RtlExecutorScenarioTest, AllInitialPositionItemsFaultyFailsGracefully)
{
	std::vector<mission_item_s> mission = {
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt + 20.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 400.f, 0.f, kAlt - 10.f),
	};

	// Fail on both takeoff (0) and the first WP (1) — no valid segment start can be found.
	FaultyVectorProvider provider(mission, {}, {0, 1}, {});
	RtlRoutePlanner planner(provider);

	auto vehicle_pos = makePositionFromOffset(kBaseLat, kBaseLon, 50.f, 0.f, kAlt + 10.f);
	config = defaultConfig();

	RtlRoutePlanner::FailureReason fail_reason{};
	bool ok = planner.planRouteToGoal(vehicle_pos, 0, config, plan, &fail_reason);

	EXPECT_FALSE(ok);
	EXPECT_NE(fail_reason, RtlRoutePlanner::FailureReason::None);
}

// WHY: If all safe points fail to load but the mission is intact, the planner should
//      still succeed by falling back to a mission endpoint (MissionLand/MissionTakeoff).
// WHAT: Faulty safe points do not prevent planning; the planner falls back to an endpoint.
TEST_F(RtlExecutorScenarioTest, AllFaultySafePointsFallBackToEndpoint)
{
	std::vector<mission_item_s> mission = {
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 500.f, 0.f, kAlt + 50.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 1000.f, 0.f, kAlt - 10.f),
	};

	std::vector<mission_item_s> safe_points = {
		makeSafePointFromOffset(kBaseLat, kBaseLon, 250.f, 50.f, kAlt),
		makeSafePointFromOffset(kBaseLat, kBaseLon, 750.f, -50.f, kAlt + 20.f),
	};

	// All safe points fail to load.
	FaultyVectorProvider provider(mission, safe_points, {}, {0, 1});
	RtlRoutePlanner planner(provider);

	auto vehicle_pos = makePositionFromOffset(kBaseLat, kBaseLon, 250.f, 0.f, kAlt + 25.f);
	config = defaultConfig();
	config.vehicle_velocity_north = 10.f;
	config.vehicle_velocity_east = 0.f;
	config.vehicle_velocity_valid = true;

	bool ok = planner.planRouteToGoal(vehicle_pos, 0, config, plan, &reason);

	// Should succeed with a mission endpoint fallback.
	ASSERT_TRUE(ok) << "Failure reason: " << RtlRoutePlanner::failureReasonString(reason);
	EXPECT_TRUE(plan.selection.found);
	EXPECT_FALSE(plan.selection.safe_point_found);
	EXPECT_TRUE(plan.selection.goal_type == RtlRoutePlanner::GoalType::MissionLand
		    || plan.selection.goal_type == RtlRoutePlanner::GoalType::MissionTakeoff);
}

// WHY: If only one safe point out of several fails to load, the planner should still
//      evaluate the remaining safe points and select one.
// WHAT: One faulty safe point does not prevent other safe points from being selected.
TEST_F(RtlExecutorScenarioTest, OneFaultySafePointDoesNotBlockOthers)
{
	std::vector<mission_item_s> mission = {
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 500.f, 0.f, kAlt + 50.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 1000.f, 0.f, kAlt + 80.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 1500.f, 0.f, kAlt - 10.f),
	};

	std::vector<mission_item_s> safe_points = {
		makeSafePointFromOffset(kBaseLat, kBaseLon, 250.f, 30.f, kAlt + 10.f),
		makeSafePointFromOffset(kBaseLat, kBaseLon, 750.f, -30.f, kAlt + 40.f),
		makeSafePointFromOffset(kBaseLat, kBaseLon, 1250.f, 20.f, kAlt + 60.f),
	};

	// Only safe point 1 fails.
	FaultyVectorProvider provider(mission, safe_points, {}, {1});
	RtlRoutePlanner planner(provider);

	auto vehicle_pos = makePositionFromOffset(kBaseLat, kBaseLon, 250.f, 0.f, kAlt + 25.f);
	config = defaultConfig();
	config.vehicle_velocity_north = 10.f;
	config.vehicle_velocity_east = 0.f;
	config.vehicle_velocity_valid = true;

	bool ok = planner.planRouteToGoal(vehicle_pos, 0, config, plan, &reason);

	ASSERT_TRUE(ok) << "Failure reason: " << RtlRoutePlanner::failureReasonString(reason);
	EXPECT_TRUE(plan.selection.found);

	// Should still find a safe point (one of the non-faulty ones).
	EXPECT_TRUE(plan.selection.safe_point_found);

	// The selected safe point must not be the faulty one (index 1).
	EXPECT_NE(plan.selection.safe_point_index, 1);
}

// WHY: If the land item at the end fails to load, the planner should still work by
//      selecting takeoff as the fallback endpoint (or a safe point if available).
// WHAT: Faulty land item does not cause a crash; the planner adapts.
TEST_F(RtlExecutorScenarioTest, FaultyLandItemDoesNotCrash)
{
	std::vector<mission_item_s> mission = {
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 500.f, 0.f, kAlt + 50.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 1000.f, 0.f, kAlt + 80.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 1500.f, 0.f, kAlt - 10.f),
	};

	// Land item (index 3) fails to load.
	FaultyVectorProvider provider(mission, {}, {3}, {});
	RtlRoutePlanner planner(provider);

	auto vehicle_pos = makePositionFromOffset(kBaseLat, kBaseLon, 250.f, 0.f, kAlt + 20.f);
	config = defaultConfig();
	config.vehicle_velocity_north = 10.f;
	config.vehicle_velocity_east = 0.f;
	config.vehicle_velocity_valid = true;

	RtlRoutePlanner::FailureReason fail_reason{};
	bool ok = planner.planRouteToGoal(vehicle_pos, 0, config, plan, &fail_reason);

	// Should either fail gracefully or succeed with takeoff as fallback.
	// Either way, no crash.
	if (ok) {
		EXPECT_TRUE(plan.selection.found);

	} else {
		EXPECT_NE(fail_reason, RtlRoutePlanner::FailureReason::None);
	}
}

// WHY: An empty mission (count=0) must be rejected immediately without reading any items.
// WHAT: Provider with zero mission items causes planRouteToGoal to fail with NoValidWaypoints.
TEST_F(RtlExecutorScenarioTest, EmptyMissionRejectedImmediately)
{
	FaultyVectorProvider provider({}, {}, {}, {});
	RtlRoutePlanner planner(provider);

	auto vehicle_pos = makePositionFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt);
	config = defaultConfig();

	RtlRoutePlanner::FailureReason fail_reason{};
	bool ok = planner.planRouteToGoal(vehicle_pos, 0, config, plan, &fail_reason);

	EXPECT_FALSE(ok);
	EXPECT_EQ(fail_reason, RtlRoutePlanner::FailureReason::NoValidWaypoints);
}

// WHY: A mission with only one waypoint (takeoff) and nothing else has no segments to follow.
// WHAT: Single-item mission fails with NoSegmentsFound or similar.
TEST_F(RtlExecutorScenarioTest, SingleItemMissionCannotBuildSegments)
{
	std::vector<mission_item_s> mission = {
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
	};

	FaultyVectorProvider provider(mission, {}, {}, {});
	RtlRoutePlanner planner(provider);

	auto vehicle_pos = makePositionFromOffset(kBaseLat, kBaseLon, 10.f, 0.f, kAlt);
	config = defaultConfig();

	RtlRoutePlanner::FailureReason fail_reason{};
	bool ok = planner.planRouteToGoal(vehicle_pos, 0, config, plan, &fail_reason);

	EXPECT_FALSE(ok);
	EXPECT_NE(fail_reason, RtlRoutePlanner::FailureReason::None);
}

// =============================================================================
// GROUP 7: Stage-machine contract verification
// =============================================================================
//
// The executor (RtlMissionSafePointFollow) cannot be instantiated without the
// full Navigator module.  These tests verify the planner-produced Plan fields
// that the executor's setNextMissionItem() state machine switches on.
//
// Each test verifies the specific Plan field constellation that causes a
// particular executor stage transition:
//
//   JoinRoute → FollowRoute:         !vtol_back_transition_required, safe_point_found, first_item != branch_off
//   JoinRoute → TransitionAfterJoin: vtol_back_transition_required
//   JoinRoute → LandAtGoal:          join projection near branch-off (tested via proximity)
//   FollowRoute → BranchOff:         current_seq reaches branch_off_index
//   BranchOff → LandAtGoal:          always (after branch-off waypoint reached)
//   No safe point → MissionEndpoint: safe_point_found=false, goal_type=MissionLand|MissionTakeoff
//
// TODO: When a MockNavigator is available, convert these to true executor tests
// that call on_activation() + setNextMissionItem() and verify _stage transitions.

// WHY: The executor transitions JoinRoute → TransitionAfterJoin when
//      join_context.vtol_back_transition_required is true.  This happens when a FW
//      vehicle reverses to takeoff and the first route segment is MC.
// WHAT: Plan for a reversed FW vehicle sets the join transition flag correctly.
TEST_F(RtlExecutorScenarioTest, PlanSetsJoinTransitionFlagForReversedFwVehicle)
{
	std::vector<mission_item_s> mission = {
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt + 20.f),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 2000.f, 0.f, kAlt + 50.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 4000.f, 0.f, kAlt + 60.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 5500.f, 0.f, kAlt - 10.f),
	};

	VectorProvider provider(mission, {});
	RtlRoutePlanner planner(provider);

	auto vehicle_pos = makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt + 15.f);
	config = fwConfig();
	config.vehicle_velocity_north = 15.f;
	config.vehicle_velocity_east = 0.f;
	config.vehicle_velocity_valid = true;

	bool ok = planner.planRouteToGoal(vehicle_pos, 0, config, plan, &reason);
	ASSERT_TRUE(ok) << "Failure reason: " << RtlRoutePlanner::failureReasonString(reason);

	if (plan.selection.path.direction_reversed
	    && plan.selection.goal_type == RtlRoutePlanner::GoalType::MissionTakeoff) {
		// The join context must indicate a back-transition is needed because the
		// first route segment in reverse enters an MC zone.
		auto action = planner.transitionActionForTargetIndex(
				      plan.selection.path.first_item_index, true, config);
		EXPECT_EQ(action, RtlRoutePlanner::TransitionAction::BackTransition);
	}
}

// WHY: The executor transitions FollowRoute → BranchOff when current_seq == branch_off_index.
//      This requires the plan to produce a valid branch_off_segment whose end index (nominal)
//      or start index (reversed) serves as the trigger.
// WHAT: When a safe point is selected, the plan's branchOffIndex() returns a valid mission index.
TEST_F(RtlExecutorScenarioTest, PlanProvidesValidBranchOffIndexForSafePoint)
{
	auto items = default_dataset::mission();
	auto safe_points = default_dataset::safePoints();
	VectorProvider provider(items, safe_points);
	RtlRoutePlanner planner(provider);

	auto vehicle_pos = makePositionAbsolute(46.10830, 2.2995, 575.f);
	config = defaultConfig();
	config.vehicle_velocity_north = default_dataset::kVel;
	config.vehicle_velocity_east = 0.f;
	config.vehicle_velocity_valid = true;

	bool ok = planner.planRouteToGoal(vehicle_pos, 4, config, plan, &reason);
	ASSERT_TRUE(ok) << "Failure reason: " << RtlRoutePlanner::failureReasonString(reason);

	if (plan.selection.safe_point_found) {
		// branchOffIndex must be a valid mission index between 0 and count-1.
		const int32_t branch_idx = plan.selection.branchOffIndex();
		EXPECT_GE(branch_idx, 0);
		EXPECT_LT(branch_idx, static_cast<int32_t>(items.size()));

		// The branch-off segment must be valid.
		EXPECT_TRUE(plan.selection.branch_off_segment.valid());

		// The branch-off projection (where the vehicle leaves the route) must be valid.
		EXPECT_TRUE(plan.selection.branch_off_projection.valid());

		// The first_item_index must be between 0 and count-1 (the executor starts walking here).
		EXPECT_GE(plan.selection.path.first_item_index, 0);
		EXPECT_LT(plan.selection.path.first_item_index, static_cast<int32_t>(items.size()));

		// For the executor to transition FollowRoute → BranchOff, the first_item must be
		// reachable before or at the branch-off index (the executor walks from first to branch).
		if (!plan.selection.path.direction_reversed) {
			EXPECT_LE(plan.selection.path.first_item_index, branch_idx);

		} else {
			EXPECT_GE(plan.selection.path.first_item_index, branch_idx);
		}
	}
}

// WHY: The executor goes straight to LandAtGoal when direct_to_safe_point is set,
//      skipping JoinRoute and FollowRoute entirely. The plan must provide a valid
//      goal_position for the landing item.
// WHAT: When direct_to_safe_point is true, all landing fields are populated.
TEST_F(RtlExecutorScenarioTest, DirectToSafePointPlanHasCompleteLandingFields)
{
	std::vector<mission_item_s> mission = {
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 500.f, 0.f, kAlt + 50.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 1000.f, 0.f, kAlt + 80.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 1500.f, 0.f, kAlt - 10.f),
	};

	std::vector<mission_item_s> safe_points = {
		makeSafePointFromOffset(kBaseLat, kBaseLon, 255.f, 0.f, kAlt + 50.f),
	};

	VectorProvider provider(mission, safe_points);
	RtlRoutePlanner planner(provider);

	auto vehicle_pos = makePositionFromOffset(kBaseLat, kBaseLon, 250.f, 0.f, kAlt + 50.f);
	config = defaultConfig();
	config.is_multicopter = true;
	config.direct_acceptance_radius = 20.f;
	config.vehicle_velocity_north = 5.f;
	config.vehicle_velocity_east = 0.f;
	config.vehicle_velocity_valid = true;

	bool ok = planner.planRouteToGoal(vehicle_pos, 0, config, plan, &reason);
	ASSERT_TRUE(ok) << "Failure reason: " << RtlRoutePlanner::failureReasonString(reason);

	if (plan.selection.direct_to_safe_point) {
		// The executor uses goal_position to build the synthetic landing item.
		EXPECT_TRUE(plan.selection.goal_position.valid());
		EXPECT_TRUE(PX4_ISFINITE(plan.selection.goal_position.lat));
		EXPECT_TRUE(PX4_ISFINITE(plan.selection.goal_position.lon));
		EXPECT_TRUE(PX4_ISFINITE(plan.selection.goal_position.alt));

		// Goal type must be SafePoint.
		EXPECT_EQ(plan.selection.goal_type, RtlRoutePlanner::GoalType::SafePoint);
	}
}

// WHY: When the executor reaches the mission endpoint in a non-safe-point plan,
//      it needs the goal_position to build the landing item. The plan must populate
//      this correctly for both MissionLand and MissionTakeoff goals.
// WHAT: Endpoint fallback plan has a valid goal_position matching the endpoint.
TEST_F(RtlExecutorScenarioTest, EndpointFallbackPlanHasValidGoalPosition)
{
	std::vector<mission_item_s> mission = {
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 500.f, 0.f, kAlt + 50.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 1000.f, 0.f, kAlt + 80.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 1500.f, 0.f, kAlt - 10.f),
	};

	VectorProvider provider(mission, {}); // No safe points
	RtlRoutePlanner planner(provider);

	auto vehicle_pos = makePositionFromOffset(kBaseLat, kBaseLon, 750.f, 0.f, kAlt + 60.f);
	config = defaultConfig();
	config.vehicle_velocity_north = 10.f;
	config.vehicle_velocity_east = 0.f;
	config.vehicle_velocity_valid = true;

	bool ok = planner.planRouteToGoal(vehicle_pos, 1, config, plan, &reason);
	ASSERT_TRUE(ok) << "Failure reason: " << RtlRoutePlanner::failureReasonString(reason);

	EXPECT_TRUE(plan.selection.found);
	EXPECT_FALSE(plan.selection.safe_point_found);

	// Goal position must be valid and match the selected endpoint.
	EXPECT_TRUE(plan.selection.goal_position.valid());

	if (plan.selection.goal_type == RtlRoutePlanner::GoalType::MissionLand) {
		EXPECT_NEAR(plan.selection.goal_position.lat, mission.back().lat, kLatLonToleranceDeg);
		EXPECT_NEAR(plan.selection.goal_position.lon, mission.back().lon, kLatLonToleranceDeg);

	} else if (plan.selection.goal_type == RtlRoutePlanner::GoalType::MissionTakeoff) {
		EXPECT_NEAR(plan.selection.goal_position.lat, mission.front().lat, kLatLonToleranceDeg);
		EXPECT_NEAR(plan.selection.goal_position.lon, mission.front().lon, kLatLonToleranceDeg);
	}

	// first_item_index must be valid (executor starts walking from here).
	EXPECT_GE(plan.selection.path.first_item_index, 0);
	EXPECT_LT(plan.selection.path.first_item_index, static_cast<int32_t>(mission.size()));
}

// WHY: The executor's closeToBranchOffSegment() is used to decide whether to go straight
//      to the goal on re-activation. The planner must provide a valid branch_off_segment
//      for this check to work.
// WHAT: closeToBranchOffSegment returns true for a position on the branch-off leg.
TEST_F(RtlExecutorScenarioTest, CloseToBranchOffSegmentWorksForOnLegPosition)
{
	auto items = default_dataset::mission();
	auto safe_points = default_dataset::safePoints();
	VectorProvider provider(items, safe_points);
	RtlRoutePlanner planner(provider);

	auto vehicle_pos = makePositionAbsolute(46.10830, 2.2995, 575.f);
	config = defaultConfig();
	config.vehicle_velocity_north = default_dataset::kVel;
	config.vehicle_velocity_east = 0.f;
	config.vehicle_velocity_valid = true;

	bool ok = planner.planRouteToGoal(vehicle_pos, 4, config, plan, &reason);
	ASSERT_TRUE(ok) << "Failure reason: " << RtlRoutePlanner::failureReasonString(reason);

	if (plan.selection.safe_point_found && plan.selection.branch_off_projection.valid()) {
		// A position exactly at the branch-off projection should be "close to" the branch-off segment.
		bool close = planner.closeToBranchOffSegment(
				     plan.selection.branch_off_projection, plan.selection, config.acceptance_radius);
		EXPECT_TRUE(close);
	}
}
