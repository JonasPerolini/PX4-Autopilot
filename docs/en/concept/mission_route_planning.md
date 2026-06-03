# Mission Route Planning Infrastructure

PX4 includes mission-route planning infrastructure in Navigator for code paths that need to reason about an uploaded mission as route geometry.
It provides a non-blocking full-route cache and a stateless planner that can project vehicle and safe-point positions onto the mission path.

::: info
This page documents developer infrastructure used by Mission smart route rejoin.
The route-aware Return mode uses the same planner concepts but is documented separately when enabled.
:::

## Purpose

Normal mission execution only needs a small window of mission items in RAM.
Navigator can load the next few mission items asynchronously from dataman and keep RAM usage low.

Route-based planning has different access patterns.
To project a position onto the uploaded route or compare safe points against route segments, the planner must scan mission geometry in arbitrary order without triggering blocking dataman reads.
The route-planning infrastructure separates that work into:

- `MissionRouteCache`: a Navigator-owned, dataman-backed provider for mission geometry, the mission landing item, and safe points.
- `MissionRoutePlanner`: a stateless geometry and scoring engine that reads data through a provider interface.
- Mission-mode route-join execution: a temporary branch-in waypoint and optional post-join VTOL transition before normal mission execution resumes.
- `DatamanCache::updateCachedItem()`: a helper that mirrors an already-written authoritative mission item into another cache instance without issuing a second dataman request.

This split lets future Navigator modes share the same route geometry without duplicating dataman state machines or making route scans depend on SD-card latency.

## Components

| Component | Role |
| --- | --- |
| `src/modules/navigator/mission_route_planner.h/cpp` | Projects the vehicle and safe points onto mission segments, evaluates route paths, and returns `JoinPlan` or `Plan` data to a caller. |
| `src/modules/navigator/mission_route_cache.h/cpp` | Maintains full-mission, mission-land, and safe-point caches and implements `MissionRoutePlanner::Provider`. |
| `src/modules/navigator/mission.h/cpp` | Gates Mission smart rejoin with `MIS_ROUTE_JOIN`, builds `JoinPlan`, and arms the branch-in pipeline on activation. |
| `src/modules/navigator/mission_base.h/cpp` | Executes shared branch-in work items and optional post-join VTOL transitions. |
| `src/modules/navigator/navigator.h` | Owns the shared `MissionRouteCache` instance for Navigator. |
| `src/lib/dataman_client/DatamanClient.hpp/cpp` | Adds `DatamanCache::updateCachedItem()` so one cache can be kept coherent after another path has already written the value. |
| `src/modules/navigator/Kconfig` | Defines `CONFIG_RTL_MISSION_CACHE_SIZE`, the board-level RAM reservation for the full mission-route cache. |

## Provider Interface

`MissionRoutePlanner` does not read dataman directly.
It accesses mission and safe-point data through `MissionRoutePlanner::Provider`.

Production code uses `MissionRouteCache` as the provider.
Tests can use a vector-backed provider in `src/modules/navigator/test/test_RTL_helpers.h`, which keeps planner tests independent from dataman, uORB, and SD-card behavior.

The provider exposes:

- Mission item count and indexed mission-item reads.
- Safe-point item count and indexed safe-point reads.
- Mission landing and takeoff lookup helpers.
- VTOL landing-approach lookup helpers for safe-point blocks.

This keeps the planner stateless: any continuity hint that a caller needs, such as a previously active `DO_JUMP` loop segment, is supplied through `MissionRoutePlanner::Config`.

## Route Planner

`MissionRoutePlanner` treats the mission as route geometry.
It builds route segments from position-bearing mission items, projects points onto those segments, and keeps only locally useful projection candidates.
The planner also understands synthetic `DO_JUMP` loop edges through `MissionRoutePlanner::Segment`, `LoopContext`, and the caller-supplied `last_flown_loop_segment`.

The current infrastructure exposes these planning products:

- `ProjectionContext`: the vehicle position projected onto a route segment.
- `JoinContext`: the geometric branch-in point and traversal direction that a caller can use to rejoin the route.
- `JoinPlan`: projection, path, and join data for mission-resume style callers.
- `Selection`: route path, selected goal, branch-off projection, and fallback goal information.
- `Plan`: projection, join, and goal-selection data for route-to-goal callers.

The planner computes this data only.
It does not publish setpoints, change mission indices, or activate flight-mode behavior.

## Projection Candidate Search

Vehicle projection and safe-point scoring use the same route scan.
The planner walks the cached mission once, builds each segment from consecutive position-bearing mission items, and evaluates that segment against a batch of reference positions.
For each reference position it:

- Computes the perpendicular projection onto the segment, clamped to the segment endpoints.
- Keeps interior projections and route-corner projections that are local minima.
- Tracks the closest crosstrack distance and accepts other local minima only within the caller-supplied extra search margin.
- Stores the best candidates in a fixed-size, crosstrack-sorted buffer of `MissionRoutePlanner::kMaxSegmentCandidates` entries.

The scan records along-route distance as it goes, so later scoring can compare a vehicle projection, safe-point projection, mission takeoff, mission land, or `DO_JUMP` loop edge in the same distance frame.

## Vehicle Projection

`collectVehicleProjection()` projects the current vehicle position onto the mission route and chooses the branch-in point that best preserves mission continuity.
Callers pass the active mission index, flight direction, velocity, optional active loop segment, and `vehicle_projection_search_dist` through `MissionRoutePlanner::Config`.

The vehicle projection has three steps:

1. Candidate collection: find up to `kMaxSegmentCandidates` local-minimum projections on the route.
   A candidate stays eligible when its crosstrack distance is no more than the closest candidate plus `vehicle_projection_search_dist`.
2. Branch-in selection: prefer a candidate on the segment that contains the active mission index.
   If the caller supplied an active `DO_JUMP` loop segment, that exact loop edge has priority.
   If there is no priority match, choose the candidate with the lowest `crosstrack distance + distance back to the last-flown segment` score.
3. Altitude assignment: interpolate altitude between the segment endpoints.
   A land segment uses the previous route altitude, because the land item altitude is not a good branch-in target.
   A zero-length segment uses the endpoint altitude.

The result is a `ProjectionContext` and `JoinContext`.
Mission mode uses this data when `MIS_ROUTE_JOIN=1`: it updates the active mission index to the selected route target, publishes a virtual branch-in waypoint, and then resumes the real mission item stream.
If the branch-in segment requires a VTOL state change, `MissionBase` publishes a post-join transition work item before returning to normal mission execution.

## Safe-Point Scoring

`selectSafePoint()` uses the same projection scan with each safe point as a reference position.
The caller supplies `safe_point_projection_search_dist`, `u_turn_penalty_m`, vehicle type flags, and whether VTOL approach eligibility is required.

Safe points are filtered before scoring:

- The provider must expose a valid `NAV_CMD_RALLY_POINT` item.
- The frame must be global or global-relative, with a valid home altitude when relative altitude is used.
- Invalid coordinates are skipped.
- If `require_vtol_approach` is set, the safe point must own at least one valid `NAV_CMD_LOITER_TO_ALT` item in the contiguous block after the rally point and before the next rally point.

Each remaining safe point gets up to `kMaxSegmentCandidates` branch-off candidates.
For every candidate, the planner computes a full route path from the vehicle projection to the branch-off projection and adds the final straight branch-off leg from the route to the safe point.
Fixed-wing and VTOL-in-fixed-wing callers can add `u_turn_penalty_m` when the selected route direction requires an immediate U-turn.
The safe point with the lowest total cost wins.

Route-to-goal callers can still apply execution shortcuts after scoring, such as going direct when the vehicle is already close to the selected safe point or close to the selected branch-off leg.
Those shortcuts do not change which safe point wins the route-based cost comparison.

## Safe-Point Batch Limit

The planner evaluates a bounded safe-point batch in one pass.
`MissionRoutePlanner::kMaxSafePointBatch` is the maximum number of eligible safe points considered in a single planning pass; extra eligible entries are skipped after that limit.
The current limit is also kept within the `uint8_t` range used by `RtlStatus.safe_point_index`.

## Route Cache

`MissionRouteCache` is the production provider for the planner.
It is maintained from Navigator's work loop and uses `DatamanCache` instances underneath:

```text
MissionRouteCache
|-- full mission route cache      [0 ... CONFIG_RTL_MISSION_CACHE_SIZE - 1]
|-- safe-point cache              [all uploaded safe-point dataman items]
|-- mission-land item cache       [published mission land index]
`-- safe-point stats reader       [DM_KEY_SAFE_POINTS_STATE async state machine]
```

Planner/provider reads use a zero wait timeout.
A cache miss therefore returns failure instead of blocking Navigator while dataman or the SD card catches up.

The cache tracks mission identity, dataman storage id, mission count, and readiness.
If the mission changes, the full-route cache is reloaded.
If validation finds that an item is missing, the cache schedules a retry using bounded exponential backoff.

Safe points have their own asynchronous state machine because the safe-point set is tracked through `DM_KEY_SAFE_POINTS_STATE`.
The cache reloads safe points when the safe-point source identity changes and prevents stale safe-point data from being exposed while a reload is in progress.

## Cache Size

`CONFIG_RTL_MISSION_CACHE_SIZE` controls the maximum number of mission items reserved for full-route planning helpers.

The default is:

- `100` for `BOARD_TESTING` builds.
- `0` for normal builds unless a board opts in.

Setting the value to `0` disables the full mission-route cache and reserves no mission-route entries.
Boards that enable route-planning consumers must size this value for their RAM budget.
Each reserved cached item uses approximately 76 bytes of heap for the lifetime of Navigator.

Example board configuration:

```ini
CONFIG_RTL_MISSION_CACHE_SIZE=300
```

## Cache Coherency

Navigator can have more than one cache view over the same dataman-backed mission item.
`DatamanCache::writeWait()` writes an authoritative value and updates the cache that performed the write.
`DatamanCache::updateCachedItem()` lets a caller mirror that same value into another cache instance when the item is already present and stable.

`MissionRouteCache::syncMissionItem()` uses this helper to keep the planner-facing mission and mission-land caches coherent after a mission item update has already been written through another path.
The helper does not issue a new dataman request, and it does not patch entries that are still waiting for an asynchronous read response.

## Tests

The infrastructure and Mission smart-rejoin behavior have focused coverage in Navigator tests:

- `functional-test_mission_route_cache` covers mission-cache loading, too-large mission rejection, mission-land caching, safe-point retry behavior, safe-point identity changes, and stale-data protection during reload.
- `functional-test_mission_route_planner_candidates` covers route projection candidate handling, corner behavior, candidate ordering, and invalid input rejection close to the planner.
- `functional-test_mission_route_planner_join` covers Mission resume join plans, including `DO_JUMP` loop exits and near-landing altitude shortcuts.
- `functional-test_mission_base` remains the shared baseline for mission traversal behavior and route-cache DO_JUMP coherency.
- `functional-test_navigator_smart_mission_join` covers Mission activation wiring, branch-in arming, and post-join VTOL transition handoff.

Future Return-mode behavior that consumes this infrastructure should add its own execution and integration coverage.

## Deferred Behavior

This Mission smart-rejoin change intentionally does not document or enable:

- A new Return mode type that follows the mission path.
- Route-following, branch-off, final approach, or landing executor stages.
- User setup instructions for route-aware RTL behavior.

Those topics belong with the later Return-mode behavior changes.
