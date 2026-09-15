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
    _SIG, _DIST, _PARENT, _ADJ = sig, dist, parent, adj
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


def _idle_move(state: GraphState, unit) -> Optional[int]:
    here = unit.current_node
    node = state.get_node(here)
    blocking = node is not None and node.capacity is not None
    if _STORAGE:
        target = min(_STORAGE, key=lambda s: _d(here, s))
        if target != here and _d(here, target) < INF:
            step = _advance(state, unit, target)
            if step is not None:
                return step
    if blocking:
        for _w, nxt in sorted((w, n) for n, w in _ADJ.get(here, [])):
            if _can_enter(state, unit, nxt):
                return nxt
    return None


def _decide(drive_unit_id: int, state: GraphState) -> Optional[int]:
    _build(state)
    unit = state.get_drive_unit(drive_unit_id)
    if unit is None or unit.in_transit:
        return None
    goal = _goals(state).get(drive_unit_id)
    if goal is None or goal == unit.current_node:
        return _idle_move(state, unit)
    return _advance(state, unit, goal)


def drive_unit_next_move(drive_unit_id: int, state: GraphState) -> Optional[int]:
    try:
        return _decide(drive_unit_id, state)
    except Exception:
        return None
