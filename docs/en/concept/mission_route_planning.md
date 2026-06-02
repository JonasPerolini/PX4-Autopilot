# Mission Route Planning Infrastructure

PX4 includes mission-route planning infrastructure in Navigator for code paths that need to reason about an uploaded mission as route geometry.
It provides a non-blocking full-route cache and a stateless planner that can project vehicle and safe-point positions onto the mission path.

::: info
This page documents developer infrastructure.
The infrastructure does not, by itself, enable a user-selectable mission smart-rejoin behavior or a mission-route-aware Return mode.
Those behaviors are expected to be documented separately by Navigator changes that consume this planner and cache.
:::

## Purpose

Normal mission execution only needs a small window of mission items in RAM.
Navigator can load the next few mission items asynchronously from dataman and keep RAM usage low.

Route-based planning has different access patterns.
To project a position onto the uploaded route or compare safe points against route segments, the planner must scan mission geometry in arbitrary order without triggering blocking dataman reads.
The route-planning infrastructure separates that work into:

- `MissionRouteCache`: a Navigator-owned, dataman-backed provider for mission geometry, the mission landing item, and safe points.
- `MissionRoutePlanner`: a stateless geometry and scoring engine that reads data through a provider interface.
- `DatamanCache::updateCachedItem()`: a helper that mirrors an already-written authoritative mission item into another cache instance without issuing a second dataman request.

This split lets future Navigator modes share the same route geometry without duplicating dataman state machines or making route scans depend on SD-card latency.

## Components

| Component | Role |
| --- | --- |
| `src/modules/navigator/mission_route_planner.h/cpp` | Projects the vehicle and safe points onto mission segments, evaluates route paths, and returns `JoinPlan` or `Plan` data to a caller. |
| `src/modules/navigator/mission_route_cache.h/cpp` | Maintains full-mission, mission-land, and safe-point caches and implements `MissionRoutePlanner::Provider`. |
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

The infrastructure has focused coverage in Navigator tests:

- `functional-test_mission_route_cache` covers mission-cache loading, too-large mission rejection, mission-land caching, safe-point retry behavior, safe-point identity changes, and stale-data protection during reload.
- `functional-test_mission_route_planner_candidates` covers route projection candidate handling, corner behavior, candidate ordering, and invalid input rejection close to the planner.
- `functional-test_mission_base` remains the shared baseline for existing mission behavior while this infrastructure is present but not yet used by Mission or RTL execution paths.

Future behavior changes that consume this infrastructure should add their own execution and integration coverage for the specific Mission or RTL modes they enable.

## Deferred Behavior

This infrastructure intentionally does not document or enable:

- A user-facing smart mission rejoin parameter.
- A new Return mode type that follows the mission path.
- Route-following, branch-off, final approach, or landing executor stages.
- User setup instructions for route-aware RTL behavior.

Those topics belong with the behavior changes that wire the planner into Mission and RTL execution.
