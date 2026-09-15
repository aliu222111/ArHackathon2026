"""
Amazon Robotics Hackathon - Routing API

*****IMPORTANT*****
Team name: vibe coders
Email address: alexliu22111@gmail.com
*******************

Centralized greedy planner: cached all-pairs Dijkstra over ceil(weight),
deterministic global unit->pod assignment recomputed every call, and a
strict no-retreat movement rule (a hop must reduce distance-to-goal, else
the unit waits) which prevents the head-on capacity-1 ping-pong livelock.
"""

import heapq
import math
from typing import Dict, List, Optional, Tuple

from ar_hackathon.models.graph_state import GraphState

INF = float("inf")
DETOUR_SLACK = 5.0
AGE_BONUS = 0.5

_SIG = None
_DIST: Dict[int, Dict[int, float]] = {}
_PARENT: Dict[int, Dict[int, int]] = {}
_ADJ: Dict[int, List[Tuple[int, float]]] = {}
_STORAGE: List[int] = []
_AVOID_CACHE: Dict = {}
_RADJ: Dict[int, List[Tuple[int, float]]] = {}


def _signature(state: GraphState):
    return tuple(sorted(
        (e.from_node, e.to_node, e.weight, e.bidirectional) for e in state.edges
    ))


def _build(state: GraphState) -> None:
    global _SIG, _DIST, _PARENT, _ADJ, _STORAGE
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
    parent: Dict[int, Dict[int, int]] = {}
    for src in adj:
        d = {src: 0.0}
        par: Dict[int, int] = {}
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
                    par[v] = u
                    heapq.heappush(pq, (nd, v))
        dist[src] = d
        parent[src] = par
    radj: Dict[int, List[Tuple[int, float]]] = {n.id: [] for n in state.nodes}
    for u, lst in adj.items():
        for v, w in lst:
            radj.setdefault(v, []).append((u, w))
    _SIG, _DIST, _PARENT, _ADJ = sig, dist, parent, adj
    _RADJ.clear(); _RADJ.update(radj)
    _AVOID_CACHE.clear()
    _STORAGE = [n.id for n in state.nodes if n.node_type == "storage"]


def _d(a: int, b: int) -> float:
    return _DIST.get(a, {}).get(b, INF)


def _position(unit) -> int:
    return unit.transit_destination if unit.in_transit else unit.current_node


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
    poison = _poison_nodes(state) if unit.has_capacity else set()
    poison.discard(goal)
    if poison:
        dmap = _dist_avoiding(goal, poison)
        get = lambda n: dmap.get(n, INF)
    else:
        get = lambda n: _d(n, goal)      # distance TO goal; correct when directed
    here_rest = get(here)
    cands = []
    for nxt, w in _ADJ.get(here, []):
        if nxt in poison:
            continue
        rest = get(nxt)
        if rest == INF or rest >= here_rest:
            continue
        cands.append((w + rest, nxt))
    cands.sort()
    for _, nxt in cands:
        if _can_enter(state, unit, nxt):
            return nxt
    return None


def _poison_nodes(state: GraphState):
    """
    Nodes holding a pod that can never be delivered.

    Pickups are compulsory: a unit with free capacity that merely STOPS at
    such a node is loaded with dead weight forever. A unit that can still
    carry something must therefore never stand on one -- not even in passing
    on the way somewhere else.
    """
    bad = set()
    for p in state.active_pods:
        if p.carried_by is None and p.current_node is not None:
            if _d(p.current_node, p.destination_station) == INF:
                bad.add(p.current_node)
    return bad


def _dist_avoiding(goal: int, blocked) -> dict:
    """Distance to goal over a graph with `blocked` nodes removed."""
    key = (goal, frozenset(blocked))
    hit = _AVOID_CACHE.get(key)
    if hit is not None:
        return hit
    d = {goal: 0.0}
    pq = [(0.0, goal)]
    seen = set()
    while pq:
        cd, u = heapq.heappop(pq)
        if u in seen:
            continue
        seen.add(u)
        for v, w in _RADJ.get(u, []):     # walk INCOMING edges: correct on one-way aisles
            if v in blocked and v != goal:
                continue
            nd = cd + w
            if nd < d.get(v, INF):
                d[v] = nd
                heapq.heappush(pq, (nd, v))
    _AVOID_CACHE[key] = d
    return d


def _forward_hops(state: GraphState, unit, goal: int):
    """Legal hops that strictly reduce distance-to-goal, best first."""
    here = unit.current_node
    here_rest = _d(here, goal)
    out = []
    for nxt, w in _ADJ.get(here, []):
        rest = _d(nxt, goal)
        if rest == INF or rest >= here_rest:
            continue
        out.append((w + rest, nxt))
    out.sort()
    return [n for _, n in out if _can_enter(state, unit, n)]


def _wanted_hop(unit, goal):
    """Where the unit WANTS to go, ignoring occupancy."""
    here = unit.current_node
    here_rest = _d(here, goal)
    best = None
    for nxt, w in _ADJ.get(here, []):
        rest = _d(nxt, goal)
        if rest == INF or rest >= here_rest:
            continue
        if best is None or (w + rest) < best[0]:
            best = (w + rest, nxt)
    return best[1] if best else None


def _escapes(state: GraphState, unit, avoid: int):
    """Valid neighbours this unit could move to, excluding the contested node."""
    return [n for n, _w in _ADJ.get(unit.current_node, [])
            if n != avoid and _can_enter(state, unit, n)]


def _holder_of(state: GraphState, node: int, me_id: int):
    for other in state.drive_units:
        if other.id != me_id and not other.in_transit and other.current_node == node:
            return other
    return None


def _wants_my_node(state: GraphState, me, goals) -> bool:
    """Is some other stationary unit trying to move onto the node I occupy?"""
    for other in state.drive_units:
        if other.id == me.id or other.in_transit:
            continue
        g = goals.get(other.id)
        if g is None or g == other.current_node:
            continue
        if _wanted_hop(other, g) == me.current_node:
            return True
    return False


def _yield_move(state: GraphState, unit, goal: int) -> Optional[int]:
    """
    Break a swap deadlock by stepping aside -- the one place a move away
    from goal is correct. Prefer the sidestep that keeps us closest to goal
    and off the contested node.
    """
    here = unit.current_node
    opts = []
    for nxt, w in _ADJ.get(here, []):
        if not _can_enter(state, unit, nxt):
            continue
        opts.append((_d(nxt, goal), w, nxt))
    opts.sort()
    return opts[0][2] if opts else None


def _idle_move(state: GraphState, unit) -> Optional[int]:
    here = unit.current_node
    node = state.get_node(here)
    blocking = node is not None and node.capacity is not None
    poison = _poison_nodes(state) if unit.has_capacity else set()
    if _STORAGE:
        safe = [s for s in _STORAGE if s not in poison and _d(here, s) < INF]
        if not safe:
            return None
        target = min(safe, key=lambda s: _d(here, s))
        if target != here and _d(here, target) < INF:
            step = _advance(state, unit, target)
            if step is not None:
                return step
    if blocking:
        for _w, nxt in sorted((w, n) for n, w in _ADJ.get(here, [])):
            if _can_enter(state, unit, nxt):
                return nxt
    return None


def _idle_clearing_move(state: GraphState, unit, goals) -> Optional[int]:
    """An idle unit parked in a working unit's path must get out of the way."""
    if not _wants_my_node(state, unit, goals):
        return None
    for _w, nxt in sorted((w, n) for n, w in _ADJ.get(unit.current_node, [])):
        if _can_enter(state, unit, nxt):
            return nxt
    return None


def _decide(drive_unit_id: int, state: GraphState) -> Optional[int]:
    _build(state)
    unit = state.get_drive_unit(drive_unit_id)
    if unit is None or unit.in_transit:
        return None
    goals = _goals(state)
    goal = goals.get(drive_unit_id)
    if goal is None or goal == unit.current_node:
        clear = _idle_clearing_move(state, unit, goals)
        if clear is not None:
            return clear
        return _idle_move(state, unit)

    step = _advance(state, unit, goal)
    if step is not None:
        return step

    # Blocked. Waiting is usually right -- a unit queueing for a busy dock
    # must not wander off. But when the node I want is held by a stationary
    # unit that cannot clear except through me, somebody has to step aside
    # or neither of us ever moves again.
    want = _wanted_hop(unit, goal)
    if want is None:
        return None
    holder = _holder_of(state, want, unit.id)
    if holder is None:
        return None            # transient block (edge full / inbound reservation): wait

    hgoal = goals.get(holder.id)
    holder_leaving = hgoal is not None and hgoal != holder.current_node \
        and _forward_hops(state, holder, hgoal)
    if holder_leaving:
        return None            # it is moving under its own power: wait, do not thrash

    # Genuine standoff. The unit that yields is the one that CAN, not the one
    # with the higher id: at a dead end there may be only one candidate. Only
    # when both can move does id break the tie, so we never both step aside
    # and re-create the ping-pong no-retreat exists to prevent.
    mine = _escapes(state, unit, want)
    theirs = _escapes(state, holder, unit.current_node)
    if not mine:
        return None
    if theirs and unit.id < holder.id:
        return None
    return _yield_move(state, unit, goal)


def drive_unit_next_move(drive_unit_id: int, state: GraphState) -> Optional[int]:
    try:
        return _decide(drive_unit_id, state)
    except Exception:
        return None
