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
 * @file RtlTransitionTest.cpp
 *
 * Unit tests for VTOL transition detection in RtlRoutePlanner.
 * Verifies that transitionActionForTargetIndex correctly identifies
 * front-transitions, back-transitions, and no-ops for various mission
 * layouts and vehicle configurations.
 */

#include "RtlRoutePlannerTestHelpers.h"
#include <uORB/topics/vtol_vehicle_status.h>

static constexpr double kBaseLat = 47.397742;
static constexpr double kBaseLon = 8.545594;
static constexpr float kAlt = 500.f;

// WHY: When flying reverse through a FW segment, the planner must detect that a back-transition
//      to MC is needed before the vehicle can hover/reverse.
// WHAT: transitionActionForTargetIndex returns BackTransition when a FW->MC transition sits
//       between the anchor and the target.
TEST(RtlTransitionTest, BackTransitionDetectedInReverse)
{
	// GIVEN: A mission with a VTOL_FW transition followed by a VTOL_MC transition.
	//        [WP(0,0), VTOL_FW, WP(N+100,0), VTOL_MC, WP(N+200,0)]
	std::vector<mission_item_s> mission = {
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
	};

	VectorProvider provider(mission, {});
	RtlRoutePlanner planner(provider);

	RtlRoutePlanner::Config config = defaultConfig();
	config.vehicle_is_vtol = true;
	config.vehicle_is_fixed_wing = true;
	config.is_multicopter = false;

	// WHEN: We query the transition action for target index 2, flying in reverse.
	auto action = planner.transitionActionForTargetIndex(2, true, config);

	// THEN: A back-transition is required because the vehicle is in FW mode and must
	//       transition to MC to fly in reverse.
	EXPECT_EQ(action, RtlRoutePlanner::TransitionAction::BackTransition);
}

// WHY: When flying reverse from a MC segment into a FW segment, the planner must detect that a
//      front-transition to FW is needed.
// WHAT: transitionActionForTargetIndex returns FrontTransition when a MC->FW transition sits
//       ahead in the reverse direction.
TEST(RtlTransitionTest, FrontTransitionDetectedInReverse)
{
	// GIVEN: A mission with a VTOL_FW transition between two position items.
	//        [WP(0,0), WP(N+100,0), VTOL_FW, WP(N+200,0)]
	std::vector<mission_item_s> mission = {
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),
	};

	VectorProvider provider(mission, {});
	RtlRoutePlanner planner(provider);

	RtlRoutePlanner::Config config = defaultConfig();
	config.vehicle_is_vtol = true;
	config.is_multicopter = true;

	// WHEN: We query the transition action for target index 1, flying in reverse.
	auto action = planner.transitionActionForTargetIndex(1, true, config);

	// THEN: A front-transition is required because the vehicle is in MC mode and must
	//       transition to FW to enter the FW segment.
	EXPECT_EQ(action, RtlRoutePlanner::TransitionAction::FrontTransition);
}

// WHY: Non-VTOL vehicles (pure multicopters or fixed-wing) never need airframe transitions.
// WHAT: transitionActionForTargetIndex returns None regardless of direction for non-VTOL vehicles.
TEST(RtlTransitionTest, NonVtolAlwaysReturnsNone)
{
	// GIVEN: A mission with a VTOL_FW command, but a non-VTOL vehicle configuration.
	//        [WP(0,0), VTOL_FW, WP(N+100,0)]
	std::vector<mission_item_s> mission = {
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),
	};

	VectorProvider provider(mission, {});
	RtlRoutePlanner planner(provider);

	RtlRoutePlanner::Config config = defaultConfig();
	config.vehicle_is_vtol = false;

	// WHEN: We query the transition action in both forward and reverse directions.
	auto action_forward = planner.transitionActionForTargetIndex(2, false, config);
	auto action_reverse = planner.transitionActionForTargetIndex(2, true, config);

	// THEN: No transition action is returned in either direction.
	EXPECT_EQ(action_forward, RtlRoutePlanner::TransitionAction::None);
	EXPECT_EQ(action_reverse, RtlRoutePlanner::TransitionAction::None);
}

// WHY: Real VTOL missions often have multiple mode transitions. The planner must correctly
//      identify which transition affects each target segment.
// WHAT: In a mission with 2 FW->MC transitions and 1 MC->FW transition, each target index
//       yields the correct action depending on the vehicle's current flight mode.
TEST(RtlTransitionTest, MultiTransitionMissionDetectsCorrectAction)
{
	// GIVEN: A mission with multiple VTOL transitions:
	//        [WP(0,0), VTOL_FW, WP(N+100,0), WP(N+200,0), VTOL_MC, WP(N+300,0),
	//         VTOL_FW, WP(N+400,0), VTOL_MC, WP(N+500,0)]
	std::vector<mission_item_s> mission = {
		makePositionItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),       // idx 0
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),  // idx 1
		makePositionItemFromOffset(kBaseLat, kBaseLon, 100.f, 0.f, kAlt),     // idx 2
		makePositionItemFromOffset(kBaseLat, kBaseLon, 200.f, 0.f, kAlt),     // idx 3
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC),  // idx 4
		makePositionItemFromOffset(kBaseLat, kBaseLon, 300.f, 0.f, kAlt),     // idx 5
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW),  // idx 6
		makePositionItemFromOffset(kBaseLat, kBaseLon, 400.f, 0.f, kAlt),     // idx 7
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC),  // idx 8
		makePositionItemFromOffset(kBaseLat, kBaseLon, 500.f, 0.f, kAlt),     // idx 9
	};

	VectorProvider provider(mission, {});
	RtlRoutePlanner planner(provider);

	// Config A: vehicle currently in FW mode
	RtlRoutePlanner::Config configA = defaultConfig();
	configA.vehicle_is_vtol = true;
	configA.vehicle_is_fixed_wing = true;
	configA.is_multicopter = false;

	// Config B: vehicle currently in MC mode
	RtlRoutePlanner::Config configB = defaultConfig();
	configB.vehicle_is_vtol = true;
	configB.vehicle_is_fixed_wing = false;
	configB.is_multicopter = true;

	// WHEN/THEN: Target index 2 (in FW zone after VTOL_FW at idx 1), reverse direction.
	//   The anchor (next position item after target) is idx 3 → getVtolStateAtAnchor
	//   scans back: idx 1 is VTOL_FW → segment state is FW.
	//   FW vehicle in FW zone → no transition needed (same mode).
	//   MC vehicle in FW zone → front-transition needed.
	EXPECT_EQ(planner.transitionActionForTargetIndex(2, true, configA),
		  RtlRoutePlanner::TransitionAction::None);
	EXPECT_EQ(planner.transitionActionForTargetIndex(2, true, configB),
		  RtlRoutePlanner::TransitionAction::FrontTransition);

	// WHEN/THEN: Target index 7 (in FW zone after VTOL_FW at idx 6), reverse direction.
	//   The anchor is idx 9 (skipping VTOL_MC at idx 8 which is not a position item).
	//   getVtolStateAtAnchor scans back: idx 8 is VTOL_MC → segment state is MC.
	//   FW vehicle in MC zone → back-transition needed.
	//   MC vehicle in MC zone → no transition needed (same mode).
	EXPECT_EQ(planner.transitionActionForTargetIndex(7, true, configA),
		  RtlRoutePlanner::TransitionAction::BackTransition);
	EXPECT_EQ(planner.transitionActionForTargetIndex(7, true, configB),
		  RtlRoutePlanner::TransitionAction::None);
}
