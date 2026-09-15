"""Session A3 candidate: base router + learned idle staging / pre-positioning.
# team: vibe coders / alexliu22111@gmail.com

Everything except ``_idle_move`` is inherited unchanged from the shared
baseline (all-pairs Dijkstra + deterministic global greedy assignment).
The contribution here is what a unit does when it has *nothing* to do:
instead of drifting to the nearest node typed "storage", it learns where
pods have actually been spawning and parks the fleet so that the expected
travel time to the next spawn is as small as possible.
"""

import heapq
import math
import os
from typing import Dict, List, Optional, Tuple

from ar_hackathon.models.graph_state import GraphState

INF = float("inf")
BIG = 1.0e6

DETOUR_SLACK = 5.0
AGE_BONUS = 0.5


def _p(name: str, default: float) -> float:
    try:
        return float(os.environ[name])
    except Exception:
        return default


# --- idle-staging tuning -------------------------------------------------
# Pseudo-count handed to each "storage" node before any pod has been seen.
# The schema allows a pod to spawn at *any* node id, so this is only a prior:
# observed spawns outvote it as the sample grows. The prior must be heavy
# enough that one early pod does not convince the whole fleet that one source
# is the only source, and must fade fast enough that a case whose pods all
# come from a non-storage node is learned quickly.
PRIOR_STORAGE = _p("A3_PRIOR", 0.6)
PRIOR_FADE = _p("A3_FADE", 0.0)   # 0 disables fading; else prior *= K/(K+n)
SEED_BUSY = _p("A3_SEEDBUSY", 0.0)  # >0: busy units' goals count as coverage
# Weight of the 2nd / 3rd / ... nearest idle unit in the coverage objective.
# 0 would be pure "one unit covers a source"; 1 would ignore spreading.
RANK_DECAY = _p("A3_DECAY", 0.4)
RANK_DEPTH = int(_p("A3_DEPTH", 3))
# Tie-break weight pulling a unit toward staging spots it is already near.
MOVE_PENALTY = _p("A3_MOVE", 0.05)
# Relative hysteresis: keep the previously chosen staging node unless some
# other node is better by more than this fraction.
HYSTERESIS = _p("A3_HYST", 0.03)
REFINE_PASSES = int(_p("A3_REFINE", 2))
MAX_CAND = 96
MAX_SRC = 48

# --- per-graph caches ----------------------------------------------------
_SIG = None
_DIST: Dict[int, Dict[int, float]] = {}
_ADJ: Dict[int, List[Tuple[int, float]]] = {}
_STORAGE: List[int] = []
_FREE: List[int] = []
_ALL: List[int] = []

# --- per-run learned state (must reset when a new game starts) -----------
_LAST_T = -1
_SEEN: set = set()
_SPAWN: Dict[int, float] = {}
_STAGE_MEMO: Dict[int, int] = {}


def _signature(state: GraphState):
    return tuple(sorted(
        (e.from_node, e.to_node, e.weight, e.bidirectional) for e in state.edges
    ))


def _reset_run() -> None:
    global _SEEN, _SPAWN, _STAGE_MEMO, _LAST_T
    _SEEN = set()
    _SPAWN = {}
    _STAGE_MEMO = {}
    _LAST_T = -1


def _build(state: GraphState) -> None:
    global _SIG, _DIST, _ADJ, _STORAGE, _FREE, _ALL
    sig = _signature(state)
    if sig == _SIG:
        return
    adj: Dict[int, List[Tuple[int, float]]] = {n.id: [] for n in state.nodes}
    for e in state.edges:
        cost = math.ceil(e.weight)
        adj.setdefault(e.from_node, []).append((e.to_node, cost))
        if e.bidirectional:
            adj.setdefault(e.to_node, []).append((e.from_node, cost))
    dist: Dict[int, Dict[int, float]] = {}
    for src in adj:
        d = {src: 0.0}
        pq = [(0.0, src)]
        seen = set()
        while pq:
            cd, u = heapq.heappop(pq)
            if u in seen:
                continue
            seen.add(u)
            for v, w in adj.get(u, []):
                nd = cd + w
                if nd < d.get(v, INF):
                    d[v] = nd
                    heapq.heappush(pq, (nd, v))
        dist[src] = d
    _SIG, _DIST, _ADJ = sig, dist, adj
    _STORAGE = [n.id for n in state.nodes if n.node_type == "storage"]
    _ALL = [n.id for n in state.nodes]
    _FREE = [n.id for n in state.nodes if n.capacity is None]
    # A brand new graph always means a brand new game.
    _reset_run()


def _d(a: int, b: int) -> float:
    return _DIST.get(a, {}).get(b, INF)


def _position(unit) -> int:
    return unit.transit_destination if unit.in_transit else unit.current_node


# --- spawn observation ---------------------------------------------------

def _observe(state: GraphState) -> None:
    """Accumulate the empirical distribution of pod source nodes.

    A pod is credited to the node it was first seen sitting on. If it was
    already picked up by the time we look (a unit parked on the source grabs
    it during the same step it spawns, before routing runs), the carrier's
    ``current_node`` is still the pickup node -- in transit that field holds
    the edge's origin -- so it is a faithful source.
    """
    for pod in state.active_pods:
        if pod.id in _SEEN:
            continue
        src = pod.current_node
        if src is None and pod.carried_by is not None:
            carrier = state.get_drive_unit(pod.carried_by)
            if carrier is not None:
                src = carrier.current_node
        if src is None:
            continue
        _SEEN.add(pod.id)
        _SPAWN[src] = _SPAWN.get(src, 0.0) + 1.0
    for pod in state.delivered_pods:
        # Seen too late to know where it came from: never credit a source,
        # but remember it so it cannot be miscounted later.
        _SEEN.add(pod.id)


def _source_weights() -> List[Tuple[int, float]]:
    w: Dict[int, float] = dict(_SPAWN)
    prior = PRIOR_STORAGE
    if PRIOR_FADE > 0.0:
        n = sum(_SPAWN.values())
        prior *= PRIOR_FADE / (PRIOR_FADE + n)
    if prior > 0.0:
        for s in _STORAGE:
            w[s] = w.get(s, 0.0) + prior
    items = [(n, x) for n, x in w.items() if x > 0.0 and n in _DIST]
    if not items:
        return []
    items.sort(key=lambda t: (-t[1], t[0]))
    return items[:MAX_SRC]


# --- staging placement ---------------------------------------------------

def _coverage(placed: List[int], srcs: List[Tuple[int, float]]) -> float:
    """Weighted expected response time of a fleet parked at ``placed``.

    The nearest parked unit dominates, but further units still count (with a
    geometric discount) because several pods can queue at one hot source and
    a unit that is busy hauling one of them cannot take the next.
    """
    total = 0.0
    for node, w in srcs:
        ds = []
        for p in placed:
            v = _d(p, node)
            ds.append(BIG if v == INF else v)
        ds.sort()
        acc = 0.0
        f = 1.0
        for v in ds[:RANK_DEPTH]:
            acc += f * v
            f *= RANK_DECAY
        total += w * acc
    return total


def _candidates(srcs: List[Tuple[int, float]]) -> List[int]:
    """Nodes an idle unit may park on.

    Squatting on a finite-capacity node steals a dock or an aisle slot from a
    later delivery, so those are excluded outright. Only if the whole graph is
    capacity-bound do we fall back to using every node.
    """
    pool = _FREE if _FREE else _ALL
    if len(pool) <= MAX_CAND:
        return sorted(pool)
    scored = sorted(pool, key=lambda c: (_coverage([c], srcs), c))
    return sorted(scored[:MAX_CAND])


def _pick(pos: int, others: List[int], cands: List[int],
          srcs: List[Tuple[int, float]], totw: float,
          memo: Optional[int]) -> Optional[int]:
    best_node = None
    best_val = INF
    memo_val = INF
    for c in cands:
        reach = _d(pos, c)
        if reach == INF:
            continue
        val = _coverage(others + [c], srcs) + MOVE_PENALTY * totw * reach
        if c == memo:
            memo_val = val
        if val < best_val - 1e-12 or (val < best_val + 1e-12 and
                                      (best_node is None or c < best_node)):
            if val < best_val:
                best_val = val
            best_node = c
    if best_node is None:
        return None
    if memo is not None and memo_val < INF:
        if memo_val <= best_val * (1.0 + HYSTERESIS) + 1e-9:
            return memo
    return best_node


def _stage_plan(state: GraphState, goals: Dict[int, int]) -> Dict[int, int]:
    """Deterministic global parking plan for every unit that has no task."""
    srcs = _source_weights()
    if not srcs:
        return {}
    totw = sum(w for _n, w in srcs)
    cands = _candidates(srcs)
    if not cands:
        return {}

    free: List[Tuple[int, int]] = []
    fixed: List[int] = []
    for u in sorted(state.drive_units, key=lambda x: x.id):
        if u.id in goals or u.carrying:
            if SEED_BUSY > 0.0:
                # A unit already tasked toward a node is covering that part of
                # the floor for now; idle units should fan out from it rather
                # than pile onto the same source.
                g = goals.get(u.id)
                if g is not None:
                    fixed.append(g)
            continue
        if u.in_transit:
            # Already repositioning: it will land on whatever it was last told
            # to park at, so treat that as occupied rather than re-planning it.
            tgt = _STAGE_MEMO.get(u.id)
            if tgt is None or _d(u.transit_destination, tgt) == INF:
                tgt = u.transit_destination
            fixed.append(tgt)
        else:
            free.append((u.id, u.current_node))
    if not free:
        return {}

    chosen: Dict[int, int] = {}
    placed = list(fixed)
    for uid, pos in free:
        pick = _pick(pos, placed, cands, srcs, totw, _STAGE_MEMO.get(uid))
        if pick is None:
            continue
        chosen[uid] = pick
        placed.append(pick)

    for _ in range(REFINE_PASSES):
        changed = False
        for uid, pos in free:
            if uid not in chosen:
                continue
            others = list(fixed) + [v for k, v in chosen.items() if k != uid]
            pick = _pick(pos, others, cands, srcs, totw, chosen[uid])
            if pick is not None and pick != chosen[uid]:
                chosen[uid] = pick
                changed = True
        if not changed:
            break
    return chosen


# --- task assignment (unchanged baseline) --------------------------------

def _goals(state: GraphState) -> Dict[int, int]:
    goals: Dict[int, int] = {}
    waiting = [p for p in state.active_pods
               if p.carried_by is None and p.current_node is not None]
    claimable = []
    for u in state.drive_units:
        carried = [state.get_pod(pid) for pid in u.carrying]
        carried = [p for p in carried if p is not None]
        if carried:
            pos = _position(u)
            nearest = min(carried, key=lambda p: _d(pos, p.destination_station))
            goals[u.id] = nearest.destination_station
        if u.has_capacity:
            claimable.append(u)
    pairs = []
    for u in claimable:
        pos = _position(u)
        for p in waiting:
            travel = _d(pos, p.current_node)
            haul = _d(p.current_node, p.destination_station)
            if travel == INF or haul == INF:
                continue
            age = state.current_time_step - p.entry_time
            pairs.append((travel + haul - AGE_BONUS * age, u.id, p.id, p.current_node))
    pairs.sort()
    used_u, used_p = set(), set()
    for _cost, uid, pid, pnode in pairs:
        if uid in used_u or pid in used_p:
            continue
        if uid in goals:
            unit = state.get_drive_unit(uid)
            pos = _position(unit)
            dest = goals[uid]
            if _d(pos, pnode) + _d(pnode, dest) - _d(pos, dest) > DETOUR_SLACK:
                continue
        goals[uid] = pnode
        used_u.add(uid)
        used_p.add(pid)
    return goals


def _can_enter(state: GraphState, unit, nxt: int) -> bool:
    edge = state.get_edge(unit.current_node, nxt)
    if edge is None:
        return False
    if edge.capacity is not None:
        if state.edge_occupancy(unit.current_node, nxt) >= edge.capacity:
            return False
    node = state.get_node(nxt)
    if node is not None and node.capacity is not None:
        if state.node_occupancy(nxt) >= node.capacity:
            return False
    return True


def _advance(state: GraphState, unit, goal: int) -> Optional[int]:
    """
    Best legal hop toward goal.

    A blocked first choice must never be answered by retreating: a hop that
    leaves the unit further from its goal than it already is undoes real
    progress, and when two units do it to each other across a capacity-1
    aisle they ping-pong forever and deliver nothing. Waiting is a legal
    move and is the correct one here -- hold position until the aisle frees.
    """
    here = unit.current_node
    here_rest = _d(here, goal)
    cands = []
    for nxt, w in _ADJ.get(here, []):
        rest = _d(nxt, goal)
        if rest == INF or rest >= here_rest:
            continue
        cands.append((w + rest, nxt))
    cands.sort()
    for _, nxt in cands:
        if _can_enter(state, unit, nxt):
            return nxt
    return None


def _idle_move(state: GraphState, unit, goals: Dict[int, int]) -> Optional[int]:
    here = unit.current_node
    node = state.get_node(here)
    blocking = node is not None and node.capacity is not None

    target = _stage_plan(state, goals).get(unit.id)
    if target is not None:
        _STAGE_MEMO[unit.id] = target
        if target != here:
            step = _advance(state, unit, target)
            if step is not None:
                return step
        elif not blocking:
            return None

    if blocking:
        # Never squat on a node somebody may need to deliver into.
        for _w, nxt in sorted((w, n) for n, w in _ADJ.get(here, [])):
            if _can_enter(state, unit, nxt):
                return nxt
    return None


def _decide(drive_unit_id: int, state: GraphState) -> Optional[int]:
    global _LAST_T
    _build(state)
    t = state.current_time_step
    if t < _LAST_T:
        # Time ran backwards: a fresh game on an identical graph.
        _reset_run()
    _LAST_T = t
    _observe(state)

    unit = state.get_drive_unit(drive_unit_id)
    if unit is None or unit.in_transit:
        return None
    goals = _goals(state)
    goal = goals.get(drive_unit_id)
    if goal is None or goal == unit.current_node:
        return _idle_move(state, unit, goals)
    return _advance(state, unit, goal)


def drive_unit_next_move(drive_unit_id: int, state: GraphState) -> Optional[int]:
    try:
        return _decide(drive_unit_id, state)
    except Exception:
        return None
