# Locked algorithm spec — drive-unit routing (team: vibe coders)

Candidate: `variants/v10_noretreat.py`. Deliberately NOT written into
`ar_hackathon/api/routing.py` — a parallel session was editing that file at the
time of writing. Copy it over once the team agrees.

## Scores (measured, not estimated)

| suite | this | best other candidate | naive baseline |
|---|---|---|---|
| 6 repo cases | **505.2** | 505.2 (v8/v9) | 351.9 |
| 8 stress cases | **615.0** | 537.7 | — |
| combined | **1120.2** | 1042.9 | — |

Reproduce: `python3 bench.py` (repo cases), stress harness in the scratchpad.

## Algorithm

1. **Static all-pairs Dijkstra, cached.** Edge weights never change in the 2026
   engine, so shortest paths are computed once per distinct graph and reused.
   Edge cost is `ceil(weight)`: the engine sets `transit_remaining_time =
   weight` and decrements by 1, so a weight of 1.5 truly costs 2 steps.
2. **Deterministic global assignment, recomputed every call.** Each call gets an
   identical view of committed state, so every unit derives the *same* greedy
   unit→pod matching and two units never chase the same pod. No cross-call
   memory is needed, which removes a whole class of stale-state bugs.
   Cost = `dist(unit→pod) + dist(pod→station) − AGE_BONUS × pod_age`.
3. **Carrying units deliver first**, diverting to batch an extra pod only if the
   detour is within `DETOUR_SLACK`.
4. **Never retreat.** A hop is only legal if it strictly reduces distance to
   goal; otherwise the unit waits. This is the single highest-value rule (below).
5. **Idle units vacate capacity-limited nodes and drift toward storage**, so a
   parked unit never blocks a single-dock station.

## The finding that matters

All 11 candidates benchmarked — including two advertising a "deadlock breaker" —
scored **0.00** on a head-on capacity-1 corridor: zero pods delivered.

Cause: when the forward hop is blocked, falling through to "any valid neighbour"
sends the unit *backwards*. Two units do this to each other across the aisle and
ping-pong forever. Fix: never accept a hop that increases distance-to-goal; wait
instead (waiting is explicitly legal). That case went 0.00 → 77.3 with no
regression anywhere else.

Level 2 is explicitly "narrow (capacity-1) aisles… expect traffic jams", so a
hidden case of this shape is likely. A zero there is catastrophic: undelivered
pods score 0 *and* stay in the denominator.

## Assumptions, stated not proven

- `DETOUR_SLACK=5.0` / `AGE_BONUS=0.5` are **inert on all 6 repo cases** — the
  score is flat across the whole sweep grid. Kept at mid-plateau values because
  `AGE_BONUS` is the anti-starvation mechanism and starvation is a hidden-case
  risk the repo cases never exercise. Unvalidated.
- Idle units drift to `storage` nodes. If a hidden case spawns pods at travel
  nodes this prior is wrong but not harmful (stress case s5 scores 90.5).

## Hidden-case coverage

Schema-legal features **no repo case exercises**, all covered by the stress
suite in `test_cases_stress/`: fractional weights, `bidirectional: false`,
node capacity on travel nodes, unit capacity ≥ 3, pods sourced at non-storage
nodes, disconnected graphs, head-on capacity-1 corridors, starvation.

Still unprobed: graphs much larger than 9 nodes; >3 units; `source_node ==
destination_station`.
