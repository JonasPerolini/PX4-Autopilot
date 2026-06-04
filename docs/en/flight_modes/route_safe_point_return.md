# Route Safe Point Return

_Route Safe Point Return_ is a mission-aware [Return Mode](./return.md) that uses the uploaded mission geometry as the return corridor. PX4 [projects the vehicle](../concept/mission_route_planning.md#vehicle-projection) and every uploaded safe point onto the mission route, chooses the safe point with the lowest total return cost using [safe-point scoring](../concept/mission_route_planning.md#safe-point-scoring), follows the route in nominal or reverse direction, and leaves the route only at the selected branch-off point.

Set [RTL_TYPE=7](../advanced_config/parameter_reference.md#RTL_TYPE) to enable it.

## Requirements

Route Safe Point Return requires:

- A valid mission with at least two position items.
- [RTL_TYPE](../advanced_config/parameter_reference.md#RTL_TYPE) set to `7`.
- A mission small enough to fit `CONFIG_RTL_MISSION_CACHE_SIZE`.

If route planning cannot run, PX4 falls back to the same direct destination selection used by `RTL_TYPE=3`: home, the closest eligible safe point, or the mission landing point.

## Behavior

When `RTL_TYPE=7` is evaluated, PX4:

1. [Projects the vehicle](../concept/mission_route_planning.md#vehicle-projection) onto the uploaded mission route.
2. Projects safe points onto the route using the same [projection candidate search](../concept/mission_route_planning.md#projection-candidate-search).
3. [Scores reachable safe-point projections](../concept/mission_route_planning.md#safe-point-scoring) by route-following distance, final branch-off distance, and fixed-wing U-turn penalty.
4. Selects the best safe point and builds a route-join, route-follow, branch-off, and landing plan.
5. If no safe point is usable but route planning succeeds, falls back to the closer mission endpoint: mission land in nominal direction or mission takeoff in reverse.

During execution, route following treats `DO_JUMP` items as geometry only. The RTL executor skips jump commands as mission commands and chooses the shorter continue-vs-rewind path through a loop exit.

For VTOL vehicles flying in fixed-wing mode, `RTL_APPR_FORCE=1` restricts safe-point selection to safe points with a valid VTOL approach. If the selected safe point has `NAV_CMD_LOITER_TO_ALT` approach items, PX4 flies the wind-selected approach before the final landing sequence.

## Parameters

| Parameter | Description |
| --- | --- |
| [MIS_MC_SEG_DIST](../advanced_config/parameter_reference.md#MIS_MC_SEG_DIST) | Extra cross-track search window for multicopter or VTOL-in-MC vehicle projection. |
| [MIS_FW_SEG_DIST](../advanced_config/parameter_reference.md#MIS_FW_SEG_DIST) | Extra cross-track search window for fixed-wing or VTOL-in-FW vehicle projection. |
| [RTL_RP_SEG_DIST](../advanced_config/parameter_reference.md#RTL_RP_SEG_DIST) | Extra cross-track search window for safe-point projection. Increase if safe points are placed far from the mission route. |
| [RTL_FW_UTURN_PEN](../advanced_config/parameter_reference.md#RTL_FW_UTURN_PEN) | U-turn distance penalty for fixed-wing and VTOL-in-FW safe-point scoring. Set to `0` to disable. |
| [RTL_APPR_FORCE](../advanced_config/parameter_reference.md#RTL_APPR_FORCE) | For VTOL in FW mode, only safe points with a valid VTOL approach are considered. |
| [RTL_PLD_MD](../advanced_config/parameter_reference.md#RTL_PLD_MD) | Precision landing mode used for the synthetic safe-point landing item and reverse-takeoff landing fallback. |

## Limits

- The route mission cache is bounded by `CONFIG_RTL_MISSION_CACHE_SIZE`; larger missions fall back to direct-path RTL.
- The route scorer evaluates at most 64 eligible rally points per planning pass.
- VTOL safe-point approaches are associated by upload order: a rally point owns following `NAV_CMD_LOITER_TO_ALT` items up to the next rally point.
- Geofence-aware pruning for vehicle and safe-point projections is not implemented.

## Implementation

The main implementation files are:

| File | Role |
| --- | --- |
| `rtl.cpp` / `rtl.h` | Selects `RTL_TYPE=7`, builds planner config, applies fallback policy, and hands the selected plan to the executor. |
| `rtl_mission_safe_point_follow.cpp` / `rtl_mission_safe_point_follow.h` | Executes route join, route follow, branch-off, optional approach, and final landing. |
| `mission_route_planner.cpp` / `mission_route_planner.h` | Projects vehicle and safe points, scores route paths, and builds the selected plan. |
| `mission_route_cache.cpp` / `mission_route_cache.h` | Provides non-blocking cached mission geometry, safe points, and mission-land data to the planner. |
| `mission_base.cpp` / `mission_base.h` | Provides shared route-join, traversal, VTOL transition, and loop-anchor helpers used by the route RTL executor. |
