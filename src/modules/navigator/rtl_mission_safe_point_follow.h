/***************************************************************************
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

#pragma once

#include "rtl_base.h"

class RtlMissionSafePointFollow : public RtlBase
{
public:
	RtlMissionSafePointFollow(Navigator *navigator, mission_s mission);
	~RtlMissionSafePointFollow() = default;

	void on_inactivation() override;
	void on_inactive() override;
	void on_activation() override;

	bool isLanding() override { return _stage == Stage::LandAtGoal; }
	bool shouldGoStraightToGoal() const override { return _should_go_straight_to_goal; }
	RtlRoutePlanner::Segment lastFlownLoopSegment() const override { return _last_flown_loop_segment; }
	rtl_time_estimate_s calc_rtl_time_estimate() override;
	void setRoutePlan(const RtlRoutePlanner::Plan &plan) override;

private:
	enum class Stage {
		Idle = 0,
		JoinRoute,
		TransitionAfterJoin,
		FollowRoute,
		BranchOff,
		LandAtGoal
	};

	bool setNextMissionItem() override;
	void setActiveMissionItems() override;

	void setWaypointMissionItem(mission_item_s &mission_item, const RtlRoutePlanner::Position &position,
				    bool autocontinue, bool vtol_back_transition_required = false) const;
	void setLandMissionItem(mission_item_s &mission_item) const;
	void normalizeRouteMissionItem(mission_item_s &mission_item) const;
	bool loadAdjacentRouteItem(mission_item_s &mission_item, int32_t *adjacent_index = nullptr);
	bool loadPreviousRoutePositionItemNoJump(int32_t start_index, int32_t &previous_index);
	bool loadMissionItemAtIndex(int32_t index, mission_item_s &mission_item);
	bool findAttachedRoutePositionIndex(int32_t start_index, int32_t &attached_index);
	bool findNextRoutePositionIndex(int32_t start_index, int32_t &next_index);
	bool currentTargetIsBranchOff() const;
	bool joinProjectionNearBranchOff() const;
	void updateLastFlownLoopSegmentFromPlan();
	void updateLastFlownLoopSegmentForNominalAdvance();
	uint8_t getVtolStateAtAnchor(uint16_t anchor_index);
	bool missedBacktransitionBetweenIndices(int32_t target_index, bool reversed);
	RtlRoutePlanner::TransitionAction transitionActionForTargetIndex(int32_t target_index,
			bool direction_reversed);
	void publishRouteItems(position_setpoint_triplet_s *pos_sp_triplet,
			       const position_setpoint_s &current_setpoint_copy,
			       const mission_item_s &current_mission_item,
			       const mission_item_s *next_mission_item);

	RtlRoutePlanner::Plan _plan{};
	Stage _stage{Stage::Idle};
	int32_t _branch_off_index{-1};
	bool _should_go_straight_to_goal{false};
	RtlRoutePlanner::Segment _last_flown_loop_segment{};
};
