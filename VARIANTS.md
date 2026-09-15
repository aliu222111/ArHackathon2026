# All driver versions — scores and provenance

Only ONE submission slot exists: `submit.py` uploads to the fixed S3 key
`2026/vibe_coders_routing.py`, so each submit overwrites the last. Every
version below is preserved here; to make any one of them the final answer,
copy it to `ar_hackathon/api/routing.py` and run `python3 submit.py`.

**Currently live: v13_poison_merge.py**

Suites: OFFICIAL = the 6 graded practice cases. 15-case = OFFICIAL + 9
adversarial. CORPUS = 31 predicted hidden cases in the graders' didactic style.

| version | OFFICIAL | 15-case | CORPUS | imperfect | notes |
|---|---|---|---|---|---|
| **v13_poison_merge** (LIVE) | 505.20 | 1210.71 | **2119.4** | 2 | v12 + poison-pod avoidance |
| v12_swapyield / routing_M2 | 505.20 | 1210.71 | 2064.2 | 2 | capability-based swap yielding |
| vC_rollout | 505.20 | **1212.54** | 2056.3 | 4 | rollout search; 2 zero-classes |
| routing_MINE / v10 | 505.20 | 1120.15 | — | — | no-retreat rule only |
| a1_assign | 505.20 | 1123.76 | — | — | agent, killed mid-run |
| a3_idle | 505.20 | 1121.38 | — | — | agent, killed mid-run |
| routing_M3 | 505.20 | 1212.54 | — | — | scratch copy, superseded |
| routing_M4 | 505.20 | 1210.71 | 2119.4 | 2 | == v13 (submitted copy) |
| **a2_reserve** | **0.00** | **0.00** | — | — | BROKEN, killed mid-edit. Do not submit. |
| basic driver (baseline) | 351.95 | — | — | — | engine's naive example |

## Why v13 is live rather than vC_rollout

vC wins the 15-case suite by 1.83 and loses the 31-case corpus by 63.1,
carrying two near-total failures v13 does not have:
`l2_06_idle_units_clog_storage` 0/4 and `l3_04_vacate_the_dock` 1/8.
The corpus is the better hidden-set proxy, so the trade favours v13.

## Known-open defects in the live driver

- `l3_12_horizon_and_late_spawns` 7/9 — never diagnosed.
- `l3_10_poison_pod_detour` 4/6 — improved from 0/6; 6/6 may not be reachable.
- `_graph_sig` sorts tuples mixing `None` and `int` capacities. Two parallel
  edges differing only in capacity raise TypeError on every call; the bare
  `except` swallows it and the case silently scores 0. **Cheapest high-value
  fix remaining** — use a None-safe sort key.
- Dead release-when-full branch in the claim logic.
- Idle unit can ping-pong at a capacity-limited storage node.

## Test suites in this repo

- `test_cases/` — the 6 official practice cases
- `test_cases/extra/` — 9 adversarial (incl. s6_headon, s9_bay, siding)
- `test_cases/corpus_a4/`, `test_cases/corpus/` — 31 predicted hidden cases
- Runners: `bench.py`, `gauntlet15.py`, `corpus.py`
