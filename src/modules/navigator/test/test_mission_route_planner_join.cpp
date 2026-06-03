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
 * @file test_mission_route_planner_join.cpp
 *
 * Unit tests for MissionRoutePlanner::planMissionResumeJoin().
 */

#include "test_RTL_helpers.h"

using rtl_test_reference::kAlt;
using rtl_test_reference::kBaseLat;
using rtl_test_reference::kBaseLon;

class MissionRoutePlannerJoinTest : public MissionRoutePlannerTestBase
{
protected:
	static MissionRoutePlanner::Segment makeLoopSegment(uint8_t loops_remaining)
	{
		MissionRoutePlanner::Segment segment{};
		segment.start.idx = 2;
		segment.start.nav_cmd = NAV_CMD_WAYPOINT;
		segment.end.idx = 0;
		segment.end.nav_cmd = NAV_CMD_WAYPOINT;
		segment.is_loop = true;
		segment.loops_remaining = loops_remaining;
		return segment;
	}

	static std::vector<mission_item_s> makeLoopMission(uint16_t current_count)
	{
		return {
			makePositionItemFromOffset(kBaseLat, kBaseLon,   0.f,   0.f, kAlt),
			makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f,   0.f, kAlt),
			makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 100.f, kAlt),
			makeDoJump(0, 2, current_count),
			makeLandItemFromOffset(kBaseLat, kBaseLon, 200.f,   0.f, kAlt - 10.f),
		};
	}
};

// WHY: Mission resume must be allowed to exit an exhausted DO_JUMP loop from the shorter side.
// WHAT: A vehicle on loop segment [2->0] with no repeats left targets idx 2.
TEST_F(MissionRoutePlannerJoinTest, ChoosesShortestExhaustedLoopExit)
{
	VectorProvider provider(makeLoopMission(2), {});
	MissionRoutePlanner planner(provider);
	config.last_flown_loop_segment = makeLoopSegment(0);

	MissionRoutePlanner::JoinPlan join_plan{};
	const auto vehicle_pos = makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 95.f, kAlt);

	ASSERT_TRUE(planner.planMissionResumeJoin(vehicle_pos, 0, config, join_plan, reason));
	EXPECT_EQ(join_plan.path.first_item_index, 2);
	EXPECT_FALSE(join_plan.path.direction_reversed);
	EXPECT_TRUE(join_plan.join_context.valid());
}

// WHY: Mission resume must preserve active loop progress while repeats remain.
// WHAT: A vehicle on loop segment [2->0] with one repeat left targets the loop end idx 0.
TEST_F(MissionRoutePlannerJoinTest, KeepsLoopEndWhileRepeatsRemain)
{
	VectorProvider provider(makeLoopMission(1), {});
	MissionRoutePlanner planner(provider);
	config.last_flown_loop_segment = makeLoopSegment(1);

	MissionRoutePlanner::JoinPlan join_plan{};
	const auto vehicle_pos = makePositionFromOffset(kBaseLat, kBaseLon, 100.f, 95.f, kAlt);

	ASSERT_TRUE(planner.planMissionResumeJoin(vehicle_pos, 0, config, join_plan, reason));
	EXPECT_EQ(join_plan.path.first_item_index, 0);
	EXPECT_FALSE(join_plan.path.direction_reversed);
	EXPECT_TRUE(join_plan.join_context.valid());
}

// WHY: Near-landing resume should not command a climb back to the interpolated route altitude.
// WHAT: The planner marks skip_altitude_requirement and rewrites the join altitude to vehicle altitude.
TEST_F(MissionRoutePlannerJoinTest, NearLandSkipsAltitudeRequirement)
{
	std::vector<mission_item_s> items = {
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon,   0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt + 20.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt - 10.f),
	};
	VectorProvider provider(items, {});
	MissionRoutePlanner planner(provider);
	config = fwConfig();
	config.parameters.acceptance_radius = 20.f;

	MissionRoutePlanner::JoinPlan join_plan{};
	const auto vehicle_pos = makePositionFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt + 3.f);

	ASSERT_TRUE(planner.planMissionResumeJoin(vehicle_pos, 1, config, join_plan, reason));
	EXPECT_EQ(join_plan.path.first_item_index, 2);
	EXPECT_EQ(join_plan.path.first_item_cmd, NAV_CMD_LAND);
	EXPECT_TRUE(join_plan.join_context.skip_altitude_requirement);
	EXPECT_NEAR(join_plan.join_context.projection.alt, vehicle_pos.alt, kAltitudeTolerance);
}
