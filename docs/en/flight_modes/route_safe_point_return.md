# Route Safe Point Return

_Route Safe Point Return_ is a mission-aware [Return Mode](./return.md) that uses the uploaded mission geometry as the return corridor.
PX4 projects the vehicle onto the route, projects every uploaded safe point onto the same route, chooses the best route-following return, rejoins the route, and only then leaves it to land.

Set [RTL_TYPE=6](../advanced_config/parameter_reference.md#RTL_TYPE) to enable it.

This mode is intended for operations where the mission itself is the safest known path through terrain, obstacles, or airspace constraints.
Unlike direct RTL variants, it does not assume that the safest way home is a straight line.

## 1. Key Concepts

- **Vehicle projection**: orthogonal projection of the current vehicle position onto the mission route, clamped to segment endpoints when needed.
- **Safe-point projection**: the same projection process applied to each uploaded safe point.
- **Local-minimum candidate**: a segment projection is only kept if it is locally minimal on the route. This avoids keeping every nearby segment corner.
- **Loop edge**: a geometric segment created from a `DO_JUMP`. It is used for projection and path solving, but not replayed as mission control flow during Route Safe Point Return.
- **Branch-off point**: the projected point where PX4 leaves the mission route and flies directly to the selected safe point.
- **Path cost**: along-route distance between the vehicle projection and the branch-off projection, plus a fixed-wing U-turn penalty when reversing would require an immediate turn-around.
- **Join context**: the virtual waypoint used to rejoin the route, plus any VTOL transition action required before the route can be followed safely.
- **Direct-to-safe-point shortcut**: multicopters, and VTOLs currently flying in MC mode, may skip route following if the selected safe point is already within the direct acceptance radius.

## 2. Mode Entry and Selection Flow

When `RTL_TYPE=6` is evaluated, PX4 performs these steps:

1. Project the vehicle onto the mission route.
2. Project all safe points onto the same route.
3. Score the reachable safe-point projections.
4. If no safe point is usable, fall back to the closer mission endpoint.
5. Build a route-join, route-follow, and branch-off plan from that result.

### 2.1 Vehicle projection

The vehicle is projected onto the full route geometry, not just the nearest waypoint.

- Consecutive position-bearing mission items create route segments.
- `DO_JUMP` items create loop edges by connecting the jump source to the jump target's next valid position item.
- Zero-length interior segments are ignored.
- Up to three local-minimum candidates are kept, sorted by cross-track distance.
- The search window shrinks to `min_xtrack + MIS_MC_SEG_DIST` or `min_xtrack + MIS_FW_SEG_DIST` as soon as a closer candidate is found.

Candidate selection then favors route continuity:

- if one candidate contains the current mission leg, that candidate wins,
- otherwise the candidate with the shortest `xtrack + distance_to_last_flown_anchor` wins,
- if the stored loop anchor matches a loop candidate, that loop candidate is preferred to avoid jumping off an active loop edge.

That last rule mirrors the legacy `_last_loop_jump_flown` behavior and matters on missions that contain `DO_JUMP` loops.

### 2.2 Safe-point scan and scoring

Safe points are loaded once and evaluated in one batched route scan.

- valid safe points are read from `dataman`,
- invalid coordinates or unsupported frames are skipped,
- oversized safe-point sets are clamped to the planner batch limit,
- every valid safe point gets up to three local-minimum route projections from the same mission scan.

This batched scan preserves the legacy behavior and avoids rereading the full mission once per safe point.

For each safe point, PX4 evaluates every valid branch-off candidate:

- the path solver computes the shortest route distance from the vehicle projection to that branch-off,
- loop candidates are only considered when the vehicle itself is projected onto a loop edge,
- fixed-wing and VTOL-in-FW paths add a U-turn penalty if reversing would require an immediate turn-around,
- the best path for that safe point becomes its candidate score.

The winning safe point is the one with the lowest valid path cost.
Cross-track distance is used to find projection candidates, but the final ranking is based on route cost, not on raw cross-track distance.

### 2.3 Direct-to-safe-point and cached branch-off reuse

Two shortcuts are inherited from the legacy SRP behavior:

- **Direct-to-safe-point**: if a multicopter-safe-point pair is already within `NAV_ACC_RAD`, PX4 can skip route following and land directly at the chosen safe point.
- **Cached branch-off reuse**: if Route Safe Point Return is selected again while the vehicle is still near the stored branch-off leg, PX4 can keep the previous safe-point choice and continue flying straight to that goal instead of forcing a route rejoin.

The cached branch-off reuse is what prevents repeated mode toggles from pulling the vehicle back to the route after it has already branched off.

### 2.4 Mission-endpoint fallback

If no safe point can be selected, Route Safe Point Return stays mission-aware.
It falls back to the better of:

- the mission landing endpoint, flown in nominal direction, or
- the mission takeoff endpoint, flown in reverse if that is shorter.

Only if the mission itself cannot be projected or solved does PX4 fall back to the generic RTL destination logic outside type 6.

## 3. Execution Behavior

The active type-6 executor runs in stages:

1. join the route,
2. apply any required post-join VTOL transition,
3. follow the route in nominal or reverse direction,
4. replace the mission target with the virtual branch-off waypoint when branch-off becomes active,
5. land at the safe point or at the selected mission endpoint fallback.

### 3.1 Join route and post-join transition

The join point is a virtual `NAV_CMD_WAYPOINT` placed at the vehicle projection.

- If the target route segment requires MC flight while the vehicle is currently fixed-wing or transitioning to FW, the join context requests a VTOL back-transition after the join waypoint is reached.
- If the selected goal is already within the acceptance radius of a landing endpoint, the join altitude requirement is skipped so landing can start immediately.
- If the join projection is already within acceptance radius of the safe-point branch-off projection, PX4 goes straight to the final landing stage instead of trying to follow a zero-length route segment.

The back-transition requirement is driven by the segment state that must be flown after joining.
It is not simply "because the join landed on a fixed-wing segment".

### 3.2 Route following and reverse traversal

During route following, PX4 treats the mission as geometry rather than as a full mission replay:

- nominal direction walks forward through position items,
- reverse direction walks backward through position items,
- non-position mission commands are skipped instead of being replayed,
- `DO_JUMP` is not re-executed as mission control flow.

VTOL transition handling is still preserved:

- after joining, PX4 can issue a required back-transition before continuing,
- while following the route, PX4 uses the same segment-end anchor rules as the planner, so reverse traversal still sees the transition items attached to the segment being entered,
- if type 6 is re-evaluated while the vehicle is already front-transitioning and the new plan flips route direction, PX4 immediately commands a back-transition before continuing.

### 3.3 Branch-off activation

The branch-off point is not an actual mission item.
PX4 injects it as a virtual waypoint when needed.

- As soon as the route target becomes the selected branch-off index, the executor replaces the current mission target with the virtual branch-off waypoint.
- If the branch-off is the _next_ route target, PX4 also previews the virtual branch-off waypoint in the mission setpoint triplet so the controller sees the correct upcoming target.
- Once the branch-off waypoint is active, the next setpoint becomes the final landing item.

This is a subtle but important part of the legacy behavior: the vehicle branches off at the projected point on the segment, not after flying all the way to the real mission endpoint.

### 3.4 Landing behavior

All final SRP landings now run through the same `MissionBase::handleLanding()` pipeline used by the other mission-based RTL modes.
That preserves the legacy landing subtleties instead of publishing a bare landing item directly.

For a selected safe point, PX4 still injects a synthetic final landing item with:

- `NAV_CMD_LAND` or `NAV_CMD_VTOL_LAND`,
- `land_precision = 2`,
- `autocontinue = false`.

That landing then inherits the normal mission landing helpers:

- rotary-wing landings insert a move-to-land waypoint first when the vehicle is still laterally displaced,
- VTOL landings can insert a fixed-wing move-to-land leg, command the VTOL back-transition, and only then descend in MC,
- mission-land fallback preserves the mission landing item itself, including any mission precision-landing setting,
- mission-takeoff fallback still lands at the takeoff location using ground-level altitude, not the takeoff waypoint altitude, when the route is flown in reverse.

## 4. Failure Handling and Current Limitations

### 4.1 Failure handling

Route Safe Point Return is evaluated defensively:

- invalid global position rejects the route plan,
- invalid or empty mission geometry rejects the route plan,
- invalid safe points are skipped,
- no usable safe point causes mission-endpoint fallback,
- no valid mission-endpoint fallback causes type-6 planning to fail.

The planner and executor now emit `PX4_DEBUG` and `PX4_INFO` logs with the `RTL SRP` prefix so projection choice, safe-point scoring, loop handling, branch-off reuse, and stage transitions can be traced during testing.

### 4.2 Current limitations

The current upstream port still leaves these items for future work:

- geofence-aware pruning for vehicle projection and safe-point projection,
- dedicated reverse-turn execution module.

## 5. Setup and Configuration

Route Safe Point Return requires:

- a valid mission with at least two position items,
- one or more uploaded safe points,
- [RTL_TYPE](../advanced_config/parameter_reference.md#RTL_TYPE) set to `6`.

Useful tuning parameters:

- [MIS_MC_SEG_DIST](../advanced_config/parameter_reference.md#MIS_MC_SEG_DIST): extra cross-track search window for multicopter or VTOL-in-MC vehicle projection.
- [MIS_FW_SEG_DIST](../advanced_config/parameter_reference.md#MIS_FW_SEG_DIST): extra cross-track search window for fixed-wing or VTOL-in-FW vehicle projection.
- [MIS_RP_SEG_DIST](../advanced_config/parameter_reference.md#MIS_RP_SEG_DIST): extra cross-track search window for safe-point projection.
- [NAV_ACC_RAD](../advanced_config/parameter_reference.md#NAV_ACC_RAD): affects join acceptance, branch-off acceptance, and the direct-to-safe-point shortcut.

Larger search windows expose more candidate segments but also admit more distant branch-off options.
Smaller windows keep the behavior closer to the nominal route.

## 6. Developer Notes

The core type-6 logic lives in:

- `src/modules/navigator/rtl_route_planner.cpp`: pure route projection, safe-point scoring, loop handling, and join/fallback planning.
- `src/modules/navigator/rtl.cpp`: plan construction, cached branch-off reuse, and mode selection.
- `src/modules/navigator/rtl_mission_safe_point_follow.cpp`: staged route execution, branch-off injection, and landing injection.

The main planner structures are:

- `ProjectionContext`: vehicle projection, mission index, loop context, and route distance state.
- `Selection`: winning safe point or fallback endpoint, branch-off geometry, and chosen path.
- `JoinContext`: join projection plus transition and altitude handling.
- `Plan`: projection, join, and goal selection combined.

Unit tests in `src/modules/navigator/RtlRoutePlannerTest.cpp` cover:

- current-leg projection preference,
- batched safe-point projection,
- safe-point selection,
- mission-endpoint fallback,
- direct-to-safe-point behavior,
- branch-off reuse helper logic,
- loop-anchor preference,
- reverse-direction VTOL transition-anchor behavior.

## Related Topics

- [Return Mode](./return.md)
- [Safety Points (Rally Points)](../flying/plan_safety_points.md)
