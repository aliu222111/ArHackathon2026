"""
Reference driver used ONLY to prove that the cases in this corpus are winnable.

It is deliberately not a submission candidate: it is routing_MINE plus exactly
three extra rules, so that a score above zero here isolates the missing rule
rather than "a different algorithm happens to do better".

  R1  YIELD: a unit blocked for several consecutive steps may take a hop that
      increases its distance to goal (retreat / pull into a passing bay).
      Thresholds are staggered by unit id so only one unit yields at a time.
  R2  TAIL LOOKAHEAD: never enter a finite-capacity node unless the next node
      on the way out of it is also enterable. Stops the one-in-one-out dock
      gridlock (unit on the dock cannot leave, unit on the approach cannot advance).
  R3  POISON AVOIDANCE: nodes holding a pod whose destination is unreachable
      from it are excluded from path search while the unit has free capacity,
      because pickups are compulsory and such a pod kills the unit.
"""

import heapq
import math
from typing import Dict, List, Optional, Tuple

INF = float("inf")
DETOUR_SLACK = 5.0
AGE_BONUS = 0.5

_SIG = None
_DIST: Dict[int, Dict[int, float]] = {}
_ADJ: Dict[int, List[Tuple[int, float]]] = {}
_STORAGE: List[int] = []
_BLOCKED: Dict[int, int] = {}
_LAST_T = -1


def _signature(state):
    return tuple(sorted((e.from_node, e.to_node, e.weight, e.bidirectional)
                        for e in state.edges))


def _build(state):
    global _SIG, _DIST, _ADJ, _STORAGE
    sig = _signature(state)
    if sig == _SIG:
        return
    adj = {n.id: [] for n in state.nodes}
    for e in state.edges:
        c = math.ceil(e.weight)
        adj.setdefault(e.from_node, []).append((e.to_node, c))
        if e.bidirectional:
            adj.setdefault(e.to_node, []).append((e.from_node, c))
    dist = {}
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


def _d(a, b, avoid=frozenset()):
    if not avoid:
        return _DIST.get(a, {}).get(b, INF)
    d = {a: 0.0}
    pq = [(0.0, a)]
    seen = set()
    while pq:
        cd, u = heapq.heappop(pq)
        if u in seen:
            continue
        seen.add(u)
        if u == b:
            return cd
        for v, w in _ADJ.get(u, []):
            if v in avoid and v != b:
                continue
            nd = cd + w
            if nd < d.get(v, INF):
                d[v] = nd
                heapq.heappush(pq, (nd, v))
    return INF


def _position(unit):
    return unit.transit_destination if unit.in_transit else unit.current_node


def _poison(state):
    """R3: nodes holding a pod that can never reach its destination."""
    bad = set()
    for p in state.active_pods:
        if p.carried_by is None and p.current_node is not None:
            if _d(p.current_node, p.destination_station) == INF:
                bad.add(p.current_node)
    return bad


def _goals(state, poison):
    goals = {}
    waiting = [p for p in state.active_pods
               if p.carried_by is None and p.current_node is not None
               and _d(p.current_node, p.destination_station) < INF]
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
    for _c, uid, pid, pnode in pairs:
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


def _can_enter(state, unit, nxt):
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


def _tail_ok(state, nxt, goal, avoid):
    """R2: do not enter a finite-capacity node whose exit toward goal is full."""
    node = state.get_node(nxt)
    if node is None or node.capacity is None or nxt == goal:
        return True
    here_rest = _d(nxt, goal, avoid)
    if here_rest == INF:
        return True
    for w, _c in ((v, c) for v, c in _ADJ.get(nxt, [])):
        rest = _d(w, goal, avoid)
        if rest == INF or rest >= here_rest:
            continue
        nd = state.get_node(w)
        if nd is None or nd.capacity is None:
            return True
        if state.node_occupancy(w) < nd.capacity:
            return True
    return False


def _advance(state, unit, goal, avoid):
    here = unit.current_node
    here_rest = _d(here, goal, avoid)
    cands = []
    for nxt, w in _ADJ.get(here, []):
        if nxt in avoid and nxt != goal:
            continue
        rest = _d(nxt, goal, avoid)
        if rest == INF or rest >= here_rest:
            continue
        cands.append((w + rest, nxt))
    cands.sort()
    for _s, nxt in cands:
        if _can_enter(state, unit, nxt) and _tail_ok(state, nxt, goal, avoid):
            return nxt
    return None


def _yield_move(state, unit, goal, avoid):
    """R1: any enterable hop, preferring the one that clears the aisle fastest."""
    here = unit.current_node
    opts = []
    for nxt, w in _ADJ.get(here, []):
        if not _can_enter(state, unit, nxt):
            continue
        if nxt in avoid:
            continue
        opts.append((w, _d(nxt, goal, avoid), nxt))
    if not opts:
        return None
    opts.sort()
    return opts[0][2]


def _idle_move(state, unit, avoid):
    here = unit.current_node
    node = state.get_node(here)
    blocking = node is not None and node.capacity is not None
    targets = [s for s in _STORAGE if s not in avoid]
    if targets:
        target = min(targets, key=lambda s: _d(here, s, avoid))
        if target != here and _d(here, target, avoid) < INF:
            step = _advance(state, unit, target, avoid)
            if step is not None:
                return step
    if blocking:
        for _w, nxt in sorted((w, n) for n, w in _ADJ.get(here, [])):
            if nxt not in avoid and _can_enter(state, unit, nxt):
                return nxt
    return None


def _decide(drive_unit_id, state):
    global _LAST_T
    _build(state)
    if state.current_time_step < _LAST_T:
        _BLOCKED.clear()
    _LAST_T = state.current_time_step

    unit = state.get_drive_unit(drive_unit_id)
    if unit is None or unit.in_transit:
        return None
    poison = _poison(state) if unit.has_capacity else frozenset()
    goal = _goals(state, poison).get(drive_unit_id)
    if goal is None or goal == unit.current_node:
        _BLOCKED[drive_unit_id] = 0
        return _idle_move(state, unit, poison)

    step = _advance(state, unit, goal, poison)
    if step is not None:
        _BLOCKED[drive_unit_id] = 0
        return step

    n = _BLOCKED.get(drive_unit_id, 0) + 1
    _BLOCKED[drive_unit_id] = n
    order = sorted(u.id for u in state.drive_units)
    rank = order.index(drive_unit_id)
    if n >= 4 + 3 * rank:
        _BLOCKED[drive_unit_id] = 0
        return _yield_move(state, unit, goal, poison)
    return None


def drive_unit_next_move(drive_unit_id: int, state) -> Optional[int]:
    try:
        return _decide(drive_unit_id, state)
    except Exception:
        return None
