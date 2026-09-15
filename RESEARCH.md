# Research: Amazon Robotics Hackathon 2026 — Drive-Unit Routing

Deep-dive on the problem: verified engine mechanics (from this repo's source, which is
the ground truth), the academic problem class it belongs to, applicable techniques, and
ranked candidate architectures for the submission.

Citation convention: repo mechanics cite `file.py:line`. Literature cites the primary
paper (venue + arXiv). Everything in §1–§3 was verified by reading the engine code in
this repo at commit `aac0d7f`; anything from memory rather than a fetched source is
marked as such.

---

## 1. Verified game mechanics (code, not README)

### 1.1 Engine step order

Each `GameEngine.step()` (game_engine.py:61-97) runs, in order:

1. **Spawn** pods whose `entry_time == current_time_step` (game_engine.py:120-126).
2. **Deliver + pick up** for every unit standing at a node (game_engine.py:128-163).
3. **Route**: poll `drive_unit_next_move` for each idle unit in **ascending unit-ID
   order**; each valid move is committed to the real state immediately
   (game_engine.py:174-185).
4. **Advance** in-transit units by one step; arrivals resolve (game_engine.py:203-218).
5. **Deliver + pick up again** for units that just arrived (game_engine.py:85).
6. `current_time_step += 1` (game_engine.py:88).
7. Game over check (game_engine.py:111-118): all pods spawned **and** delivered
   (early exit), or `current_time_step >= max_time_steps`.

### 1.2 Timing arithmetic (worked out from the code)

- Committing a move sets `transit_remaining_time = edge.weight`
  (game_engine.py:197-199). The advance phase decrements by 1 and the unit arrives
  when it hits `<= 0` (game_engine.py:203-218).
- **A move committed at step `t` on an edge of weight `w` arrives during step
  `t + w − 1`, and the unit can commit its next move at step `t + w`.** So each edge
  costs exactly `w` steps of wall time between consecutive commits. Weight-1 edges:
  commit and arrive in the same step.
- Since weights are JSON `number` with min 1 (schema.json), a fractional weight
  (e.g. 2.5) effectively costs `ceil(w)` steps. All sample cases use integers.
- Delivery happens in phase 5 of the arrival step; `delivery_time` = that step's
  index (game_engine.py:149). `delivery_duration = delivery_time − entry_time`
  (game_engine.py:240).
- **Zero-duration deliveries are possible**: a unit already standing on a pod's
  source node when it spawns picks it up in phase 2, moves along a weight-1 edge in
  phase 3, arrives in phase 4, delivers in phase 5 — duration 0, full 100 points.
  Pre-positioning at storage nodes before spawns is directly rewarded.
- Worked example, test_case_1: path 0→1→2→3 (weights 2+2+2): commits at t=0,2,4,
  delivery at t=5, duration 5 → score 100·e^(−5/50) = **90.48**. The naive baseline
  takes the direct weight-10 aisle: duration 9 → 83.53 (reproduced empirically, §3).

### 1.3 Move validity (`is_valid_move`, routing_utils.py:12-56)

A returned move is silently discarded (unit waits) unless **all** hold:

- `int` and not `bool` (routing_utils.py:26) — `numpy` ints would fail too, but
  stdlib-only anyway.
- Unit not in transit (:30). Returning the current node = wait (:34).
- An edge exists `current → next`; bidirectional edges match either direction
  (:38-42, graph_state.py:35-44). Note `get_edge` returns the *first* matching edge,
  so parallel edges would alias — none exist in the samples.
- **Edge capacity**: units currently traversing the edge, both directions combined
  (:45-47, graph_state.py:76-89) must be `< capacity`. `None` = unlimited.
- **Node capacity**: units standing at the destination **plus units in transit
  toward it** must be `< capacity` (:50-53, graph_state.py:91-102). Inbound units
  reserve their slot for their *entire* traversal — a unit on a long edge into a
  capacity-1 station holds the dock the whole way in.

Consequences: on a capacity-1 edge no head-on swap and no tailgating in the same
direction; a unit *parked* at a capacity-1 station blocks it indefinitely (clear the
dock immediately after delivering); there are **no collision failures at all** —
invalid moves just waste a step.

### 1.4 Pickups and deliveries (game_engine.py:128-163)

- Per unit, **ascending ID**: deliveries first (frees capacity), then pickups.
- Pickups are **automatic and cannot be declined**: a unit with free capacity
  standing at a node with waiting pods grabs them greedily, sorted by
  `(entry_time, id)` (:157), until full. **Passing through a storage node
  auto-picks-up whatever is waiting there** — plan around unintended pickups.
- Lowest unit ID at a shared node picks up first (:138).
- With multi-pod capacity, each carried pod is dropped individually whenever the
  unit stands at that pod's destination (:143-151).

### 1.5 Player-call contract

- Each idle unit gets a **fresh deep copy** of the full state per call
  (game_engine.py:179). Crucially, the copy is taken **after** lower-ID units'
  moves this step were committed — **higher-ID units can observe what lower-ID
  units just did within the same time step** (their `in_transit`,
  `transit_destination`, `transit_remaining_time = w` not yet decremented).
- An in-transit unit observed at routing time of step `t` with remaining time `r`
  arrives during step `t + r − 1`.
- 1-second timeout per call, enforced via a thread pool (game_engine.py:176-182,
  273-297). **Exceptions and timeouts are swallowed** → unit waits. Two sharp edges:
  (a) a timed-out call's thread is never killed (Python can't), so it keeps burning
  CPU under the GIL and slows every later call — stay well under 1 s; (b) a crash is
  silent at the grader, so wrap everything in `try/except` with a safe fallback.
- **The future is hidden**: `GraphState` contains only already-spawned pods
  (`active_pods`) plus `delivered_pods`; there is no reference to the test case's
  `pods_by_time`. Future arrivals are genuinely unknown to the player. This is an
  *online* problem.
- Module-level globals in `routing.py` persist across calls — global planning is
  legal and intended (README "A note on coordination"). They may also persist
  **across test cases** if the grader runs several in one process: detect a reset
  (`current_time_step` decreased, or graph fingerprint changed) and clear state.
- `state.get_pod` searches delivered pods too (graph_state.py:53-60), so you can
  track your own running score.

### 1.6 Scoring (game_engine.py:220-269)

- Per delivered pod: `100 · e^(−duration/50)`. Sum, then normalize by
  `100 · total_pods` → **final score = mean over all pods of `e^(−d/50)`, ×100**,
  with undelivered pods contributing 0.
- Leaderboard score = **sum across all evaluation test cases** (README), so every
  case matters equally in absolute points.

### 1.7 README vs code discrepancies

The code confirms essentially every README claim. Nuances the README under-states:

| Topic | Code truth |
|---|---|
| "picks it up… passes through" (routing.py docstring) | Pickups only occur when *not in transit*, i.e. at arrival/standing — but arrival at an intermediate node does trigger pickup (phase 5), so pass-through pickup is real, with zero extra dwell. |
| Pickup is optional | It is **not** — auto-pickup is forced whenever capacity is free (game_engine.py:154-163). |
| max_time_steps | Defaults to 100 if metadata omits it (test_case.py:34); all sample cases set it. |
| 2-minute per-test-case limit | Grading-side only; not in this codebase. |
| Directed edges | `bidirectional: false` is legal in the schema; all 6 samples are bidirectional. Handle directed edges anyway. |

### 1.8 Exploitable mechanics (all verified above)

1. **Low ID = absolute priority** in both contention and shared-node pickups →
   give the most contended/urgent assignments to low-ID units.
2. **Same-step observability**: higher-ID units see lower-ID commitments → a
   central plan executed in ID order is naturally consistent; reactive dodging works.
3. **Pre-position idle units** at storage nodes (they're capacity-unlimited in all
   samples) to catch future spawns at duration ≈ travel time, sometimes 0.
4. **Dock discipline**: never park at finite-capacity nodes; time arrivals so the
   inbound reservation window (the whole edge traversal) doesn't starve others.
5. **Deterministic engine + full state** → you can clone the engine's transition
   rules inside `routing.py` and forward-simulate candidate plans exactly.
6. **Early termination** on last delivery — no incentive to hold anything back.

---

## 2. Test-case atlas

| Case | Nodes | Edges | Units (cap) | Pods | Entry times | max_t | Key constraint |
|---|---|---|---|---|---|---|---|
| L1 tc1 | 4 | 4 | 1 (1) | 1 | 0 | 60 | weighted SP beats greedy (10 vs 2+2+2) |
| L1 tc2 | 8 | 10 | 1 (1) | 3 | 0,20,40 | 150 | return trips, 2 stations |
| L2 tc3 | 6 | 6 | 2 (1) | 2 | 0,0 | 80 | capacity-1 bridge (w=6) vs detour (4+4) |
| L2 tc4 | 9 | 12 | 3 (1) | 6 | 0×3,4,8×2 | 150 | 3×3 grid, **every** edge capacity 1 |
| L3 tc5 | 6 | 5 | 3 (1) | 4 | 0,0,2,4 | 120 | single cap-1 station dock, tree graph |
| L3 tc6 | 8 | 8 | 2 (**2**) | 8 | 0×4,10×2,20×2 | 200 | station caps 1 and 2, batching pays |

All sample edges bidirectional; only tc4 has edge capacities; only tc5/tc6 have node
capacities (stations); only tc6 has multi-pod units. Hidden cases are "of similar size
and difficulty" (README), so graphs stay small — **think ≤ ~15 nodes, ≤ ~5 units**.

Baseline (`basic_driver`, greedy cheapest-edge, no coordination) scores, reproduced
2026-09-15: tc1 **83.5**, tc2 **27.3**, tc3 **74.1**, tc4 **70.3**, tc5 **54.1**,
tc6 **42.6** — sum **352.0**, with 10 of 24 pods stranded. Its failure modes
(examples/basic_driver.py): no shortest path (visited-set wander), all units chase
the *same* first pod in list order, no dock/edge awareness.

---

## 3. Scoring math and what it implies for strategy

`v(d) = e^(−d/50)`: half-life ≈ 34.7 steps; each step of delay costs ~2 % of the
pod's *current* value.

| d | 0 | 5 | 10 | 20 | 35 | 50 | 100 | 150 | stranded |
|---|---|---|---|---|---|---|---|---|---|
| points | 100 | 90.5 | 81.9 | 67.0 | 49.7 | 36.8 | 13.5 | 5.0 | 0 |

Decision-relevant consequences:

1. **Delivery percentage dominates.** A pod delivered at d=150 still beats a
   stranded pod. Never abandon reachable pods (see baseline's 10 stranded pods).
2. **Exponential decay inverts FIFO.** Because marginal loss is proportional to
   remaining value, an already-old pod loses little by waiting more, a fresh pod
   loses a lot. With a service bottleneck, delivering one pod at d=0 and one at
   d=100 (100+13.5=113.5) beats both at d=50 (73.6). **Under contention, serve the
   freshest-catchable pod fast rather than equalizing lateness** — while still
   eventually clearing everything (point 1).
3. **On 4–9-node graphs, dispatch beats path-finesse.** Shortest paths differ by a
   few steps (~a few %); a wrong assignment or a stranded pod costs tens of points.
   Assignment and sequencing are where the score is.
4. Convexity also means a *near-optimal* schedule loses very little — robust
   heuristics are fine; brittleness (crashes, timeouts, deadlocks) is what kills.

---

## 4. Problem class: where this sits in the literature

This is **lifelong / online Multi-Agent Pickup and Delivery (MAPD)** on a small
weighted graph with edge and node capacities — the direct academic descendant of the
Kiva/Amazon-Robotics warehouse problem this hackathon simulates
([Wurman, D'Andrea & Mountz, "Coordinating Hundreds of Cooperative, Autonomous
Vehicles in Warehouses," AI Magazine 29(1), 2008](https://ojs.aaai.org/aimagazine/index.php/aimagazine/article/view/2082);
MAPD formalized by [Ma, Li, Kumar & Koenig 2017, arXiv:1705.10868](https://arxiv.org/abs/1705.10868);
MAPF taxonomy in [Stern et al., "MAPF: Definitions, Variants, and Benchmarks,"
SoCS 2019, arXiv:1906.08291](https://arxiv.org/abs/1906.08291)).

**Where this problem deviates from standard MAPF/MAPD assumptions** (each deviation
weakens the case for off-the-shelf MAPF machinery):

| Standard MAPF/MAPD assumption | This game |
|---|---|
| Unit-time edges, 4-connected grid | Weighted edges (1–10 in samples), arbitrary small graphs |
| Vertex/swap conflicts are hard failures | No collisions exist; capacities gate entry; blocked = wasted step |
| One agent per vertex/edge | Capacities default **unlimited**; only tagged edges/nodes constrain |
| Joint plan executed open-loop | Engine polls per agent, ascending ID, commits immediately = **prioritized planning with a fixed priority order baked into the referee** |
| Tasks assigned by planner; pickup is an action | Pickup is automatic and **compulsory** |
| Task stream visible or announced | Future arrivals invisible until spawn |
| Hundreds of agents, big maps (scalability is the research problem) | ≤ 3 units, ≤ 9 nodes in samples — **scalability is a non-issue** |

That last row is the punchline: most MAPD literature fights scale. Here the instance
is tiny and the per-step compute budget (1 s × ≤ 5 units, 2 min/case) is enormous
relative to it — near-exhaustive methods are on the table.

### 4.1 Techniques and their fit

**Space-time A* / reservation tables + windowed replanning (WHCA\*)** —
[Silver, "Cooperative Pathfinding," AIIDE 2005](https://ojs.aaai.org/index.php/AIIDE/article/view/18726).
Plan each agent in (node, time) space against a table of reservations left by
previously-planned agents; replan on a rolling window. **Fit: excellent.** The
engine's ID-order commitment *is* prioritized planning, and the reservation table
generalizes cleanly to capacity-k edges/nodes (store counts, not booleans). Search
space ≈ nodes × horizon ≈ 10 × 200 = 2 000 states — microseconds with stdlib
`heapq`. This is the natural pathfinding core. Known failure mode of prioritized
planning — a low-priority agent gets walled in — is real but mild here (capacities
are mostly unlimited; waiting is always legal and never fatal).

**Conflict-Based Search (CBS)** —
[Sharon, Stern, Felner & Sturtevant, AIJ 219:40-66, 2015](https://dl.acm.org/doi/10.1016/j.artint.2014.11.006).
Optimal two-level search branching on conflicts. **Fit: poor effort/reward.**
Optimal *collision-free* paths aren't the objective here (there are no collisions),
and CBS says nothing about task assignment, which is where the points are.
Adapting its conflict semantics to capacity-k counting is real work for ~no score.

**Token Passing / TPTS / CENTRAL (MAPD)** —
[Ma et al. 2017](https://arxiv.org/abs/1705.10868). TP: agents take the token,
greedily grab the highest-priority unassigned task, and path-plan against the
token's reservations; complete on "well-formed" instances; scales to hundreds of
agents. **Fit: conceptually the right frame** (lifelong tasks, decoupled agents,
reservations) but its machinery targets unit-cost grids with designated endpoints
and declinable tasks. With ≤ 5 agents, TP's greedy task choice is strictly worse
than just enumerating assignments. Steal the *shape* (persistent reservation store +
greedy replan on new tasks), not the algorithm.

**Rolling-Horizon Collision Resolution (RHCR)** —
[Li, Tinka, Kiesel, Durham, Kumar & Koenig, AAAI 2021, arXiv:2005.07371](https://arxiv.org/abs/2005.07371).
Windowed replanning every h steps, resolving conflicts only w steps ahead; 1 000+
agents. **Fit: the *replan-every-step* discipline is the right takeaway**; the
scalability machinery is unnecessary at this size.

**Task assignment: Hungarian / min-cost matching, auctions, regret insertion** —
Offline MAPD with assignment treated jointly in
[Liu, Ma, Li & Koenig, AAMAS 2019](https://www.ifaamas.org/Proceedings/aamas2019/pdfs/p1152.pdf)
(TA-Hybrid: assignment as TSP-ish sequencing, then path planning). **Fit: high, in
miniature.** With u ≤ 5 units and p ≤ ~8 live pods, brute-force over assignments
(u! ≤ 120) or Hungarian O(n³) via a ~60-line stdlib implementation is trivial. The
cost matrix should be *time-to-deliver* estimates from space-time Dijkstra, and the
objective should be summed **score gain** `e^(−(age+ttd)/50)`, not raw distance —
this bakes §3's freshest-first insight in for free.

**Vehicle-routing lineage (PDP / dial-a-ride)** — the OR ancestor of all of this
(standard reference: Savelsbergh & Sol, "The General Pickup and Delivery Problem,"
Transportation Science 29(1), 1995 — *citation from memory, not fetched this
session*). Relevant only as framing: multi-pod units (tc6) make per-unit tours a
tiny PDP; with ≤ 4 stops, enumerate orderings exactly.

**Continuous/non-unit-time MAPF (CCBS et al.)** exists for the weighted-edge gap
(Andreychuk et al., Continuous-Time CBS — *from memory, not fetched*), but
space-time Dijkstra with integer step costs already handles this game's weights
exactly; no need.

**Learning-based / potential-field / guidance-graph methods** (e.g.
[arXiv:2404.16162](https://arxiv.org/pdf/2404.16162) surveys the frontier):
irrelevant under stdlib-only, tiny instances, and a one-day timeline.

---

## 5. Candidate architectures, ranked

Common chassis for all of them (mandated by §1): module-global planner keyed by a
state fingerprint (reset detection); every entry point wrapped in `try/except`
returning `None`; plan stored as per-unit next-hop; replan when reality diverges
from plan or a new pod spawns; all compute well under 1 s (expect < 10 ms).

**A. Central assignment + space-time Dijkstra with capacity reservations —
RECOMMENDED TARGET.**
On spawn/divergence: enumerate assignments of idle+en-route units to live pods
(brute force ≤ 120; Hungarian if bigger), cost = score gain from estimated delivery
time; then plan paths in ID order through a (thing, timestep)→count reservation
table honoring §1.3 semantics exactly (inbound node reservations for the full edge
duration; both-direction edge counts). Emit next hops; higher-ID units replan around
lower-ID commitments observed in-state (§1.5). Handles all 6 samples' mechanics;
degrades gracefully on hidden cases. Effort: several hours. Risk: low. Expected
outcome: near-ceiling on L1/L2, strong L3.

**B. Forward-simulation search (engine clone + rollout evaluation).**
Reimplement §1.1's deterministic transition inside `routing.py` (it is ~150 lines),
then evaluate candidate high-level plans (assignment + path + wait insertions) by
*exact* simulated score; hill-climb or beam-search over plans each replan. Only
approach that natively prices forced auto-pickups, dock-arrival timing, and the
freshest-first tradeoff — it optimizes the actual objective. Effort: +2–4 h over A;
best final score; the natural "afternoon upgrade" with A as its move generator.

**C. Greedy Dijkstra + wait-on-conflict (fallback, build FIRST).**
Plain weighted Dijkstra to nearest unclaimed pod (claim registry in a global),
carried pods to destination; before emitting a hop, check §1.3 validity and dock
occupancy, else wait or take second-best neighbor. ~1 h, beats the baseline
massively (fixes stranded pods and duplicate chasing), and is the safety net to
submit early and keep on the leaderboard while A/B mature.

**D. Token Passing adaptation.** Correct lineage, wrong scale — its greedy
task-grab is dominated by A's tiny exhaustive assignment. Skip.

**E. Per-case offline optimal schedules.** Hidden cases and the online pod stream
make memorized schedules brittle; B already captures the benefit safely. Skip.

**Recommended path: C → A → B**, submitting after each stage (latest submission
wins; the leaderboard confirms hidden-case behavior early).

---

## 6. Open questions / residual risks

1. **Grader process model** — one process per test case or shared? Costless to
   defend with fingerprint-keyed reset (§1.5); do it unconditionally.
2. **Hidden-case shapes** — directed edges (`bidirectional: false`) and fractional
   weights are schema-legal but absent from samples; support both (cheap).
3. **Grading-machine speed** — 2 min/case is ~10⁴× headroom for A; only B's rollout
   breadth needs a step-budget guard (cap rollouts per call, keep a greedy answer
   ready).
4. **Pathological hidden cases** — e.g. more pods than deliverable in max_t: the
   freshest-first rule (§3.2) is exactly the right triage and falls out of A's
   score-gain objective automatically.
