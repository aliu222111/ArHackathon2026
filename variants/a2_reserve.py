"""Congestion-aware drive-unit routing.

Same skeleton as the greedy baseline (lazy reverse-Dijkstra distances +
deterministic global assignment) but the per-step move is chosen by
*prioritised space-time planning*: units are planned one at a time, in a
fixed priority order, each one writing node and edge reservations into a
time-expanded table that every later unit must respect.  Cooperative A*
in the style of WHCA*, with a sliding window.

Two invariants are kept from the greedy version because they are what
stops livelock on capacity-1 corridors:

  * a hop must strictly reduce distance-to-goal; when the forward hop is
    unavailable the unit waits (never retreats);
  * idle units never squat on a finite-capacity node.

The reservation table adds three things the greedy version could not do:
choose *which* of several equally short forward hops avoids the traffic
another unit already claimed, work out that waiting now beats moving into
a slot that is about to be taken, and keep idle units from wandering into
a chokepoint a loaded unit is about to need.
"""

import heapq
import math
import time
from typing import Dict, List, Optional, Set, Tuple

from ar_hackathon.models.graph_state import GraphState

INF = float("inf")

# --- tunables -------------------------------------------------------------
WINDOW = 24          # reservation horizon, in time steps
DETOUR_SLACK = 5.0   # extra travel a carrying unit will accept to grab a pod
AGE_BONUS = 0.5      # per-step urgency credit for a waiting pod
PLAN_SECONDS = 0.30  # wall-clock budget for one full replan
EXPANSIONS = 40000   # space-time states expanded per unit, at most

# --- graph cache (rebuilt when the graph changes) -------------------------
_SIG = None
_ADJ: Dict[int, List[Tuple[int, int]]] = {}
_RADJ: Dict[int, List[Tuple[int, int]]] = {}
_EDGE_IDX: Dict[Tuple[int, int], int] = {}
_EDGE_CAP: Dict[int, Optional[int]] = {}
_NODE_CAP: Dict[int, Optional[int]] = {}
_DTO: Dict[int, Dict[int, float]] = {}
_SOURCES: Set[int] = set()
_FREE_PARK: List[int] = []

# --- per-step plan cache --------------------------------------------------
_PLAN_KEY = None
_PLAN_FIRST: Dict[int, Optional[int]] = {}


# ==========================================================================
# graph preprocessing
# ==========================================================================

def _signature(state: GraphState):
    nodes = tuple(sorted(
        (n.id, -1 if n.capacity is None else n.capacity) for n in state.nodes))
    edges = tuple(sorted(
        (e.from_node, e.to_node, float(e.weight),
         1 if e.bidirectional else 0,
         -1 if e.capacity is None else e.capacity) for e in state.edges))
    return nodes, edges


def _build(state: GraphState) -> None:
    global _SIG, _ADJ, _RADJ, _EDGE_IDX, _EDGE_CAP, _NODE_CAP
    global _DTO, _SOURCES, _FREE_PARK, _PLAN_KEY, _PLAN_FIRST
    sig = _signature(state)
    if sig == _SIG:
        return

    node_cap: Dict[int, Optional[int]] = {}
    for n in state.nodes:
        node_cap[n.id] = n.capacity

    edge_idx: Dict[Tuple[int, int], int] = {}
    edge_cap: Dict[int, Optional[int]] = {}
    for i, e in enumerate(state.edges):
        edge_cap[i] = e.capacity
        pairs = [(e.from_node, e.to_node)]
        if e.bidirectional:
            pairs.append((e.to_node, e.from_node))
        for pair in pairs:
            # get_edge() returns the first edge that connects; mirror that.
            if pair not in edge_idx:
                edge_idx[pair] = i

    adj: Dict[int, List[Tuple[int, int]]] = {n.id: [] for n in state.nodes}
    radj: Dict[int, List[Tuple[int, int]]] = {n.id: [] for n in state.nodes}
    for (u, v), i in edge_idx.items():
        cost = int(math.ceil(state.edges[i].weight))
        if cost < 1:
            cost = 1
        adj.setdefault(u, []).append((v, cost))
        radj.setdefault(v, []).append((u, cost))
    for lst in adj.values():
        lst.sort()
    for lst in radj.values():
        lst.sort()

    _SIG = sig
    _ADJ, _RADJ = adj, radj
    _EDGE_IDX, _EDGE_CAP, _NODE_CAP = edge_idx, edge_cap, node_cap
    _DTO = {}
    _SOURCES = set(n.id for n in state.nodes if n.node_type == "storage")
    _FREE_PARK = sorted(n.id for n in state.nodes if n.capacity is None)
    _PLAN_KEY = None
    _PLAN_FIRST = {}


def _dto(goal: int) -> Dict[int, float]:
    """Distance from every node TO `goal` (reverse Dijkstra, cached)."""
    cached = _DTO.get(goal)
    if cached is not None:
        return cached
    dist = {goal: 0.0}
    pq = [(0.0, goal)]
    seen = set()
    while pq:
        d, u = heapq.heappop(pq)
        if u in seen:
            continue
        seen.add(u)
        for v, w in _RADJ.get(u, ()):
            nd = d + w
            if nd < dist.get(v, INF):
                dist[v] = nd
                heapq.heappush(pq, (nd, v))
    _DTO[goal] = dist
    return dist


def _d(a: int, b: int) -> float:
    return _dto(b).get(a, INF)


def _pos_eta(unit) -> Tuple[int, int]:
    """Where the unit will next be able to make a decision, and when."""
    if unit.in_transit:
        return unit.transit_destination, max(1, int(math.ceil(unit.transit_remaining_time)))
    return unit.current_node, 0


# ==========================================================================
# task assignment  ->  per-unit waypoint list
# ==========================================================================

def _dedupe(seq: List[int]) -> List[int]:
    out: List[int] = []
    for x in seq:
        if not out or out[-1] != x:
            out.append(x)
    return out


def _carry_route(pos: int, carried) -> List[int]:
    """Greedy nearest-first ordering of the destinations already on board."""
    route: List[int] = []
    cur = pos
    rest = list(carried)
    while rest:
        best = None
        for p in rest:
            d = _d(cur, p.destination_station)
            if d == INF:
                continue
            key = (d, str(p.id))
            if best is None or key < best[0]:
                best = (key, p)
        if best is None:
            break
        p = best[1]
        route.append(p.destination_station)
        cur = p.destination_station
        rest.remove(p)
    return _dedupe(route)


def _assign(state: GraphState) -> Dict[int, List[int]]:
    """unit id -> ordered waypoints.  Missing key means 'nothing to do'."""
    global _SOURCES
    now = state.current_time_step
    units = sorted(state.drive_units, key=lambda u: u.id)

    waiting = [p for p in state.active_pods
               if p.carried_by is None and p.current_node is not None]
    for p in waiting:
        _SOURCES.add(p.current_node)

    goals: Dict[int, List[int]] = {}
    claimable = []
    for u in units:
        pos, _eta = _pos_eta(u)
        carried = [state.get_pod(pid) for pid in u.carrying]
        carried = [p for p in carried if p is not None]
        if carried:
            route = _carry_route(pos, carried)
            if route:
                goals[u.id] = route
        if u.has_capacity:
            claimable.append(u)

    pairs = []
    for u in claimable:
        pos, eta = _pos_eta(u)
        for p in waiting:
            travel = _d(pos, p.current_node)
            haul = _d(p.current_node, p.destination_station)
            if travel == INF or haul == INF:
                continue
            age = now - p.entry_time
            cost = eta + travel + haul - AGE_BONUS * age
            pairs.append((cost, u.id, str(p.id), p.current_node))
    pairs.sort()

    used_u: Set[int] = set()
    used_p: Set[str] = set()
    claimed_nodes: Set[int] = set()
    for _cost, uid, pid, pnode in pairs:
        if uid in used_u or pid in used_p:
            continue
        existing = goals.get(uid)
        if existing:
            unit = state.get_drive_unit(uid)
            pos, _eta = _pos_eta(unit)
            dest = existing[0]
            detour = _d(pos, pnode) + _d(pnode, dest) - _d(pos, dest)
            if detour > DETOUR_SLACK:
                continue
            goals[uid] = _dedupe([pnode] + existing)
        else:
            goals[uid] = [pnode]
        used_u.add(uid)
        used_p.add(pid)
        claimed_nodes.add(pnode)

    # Idle units: spread out over the nodes pods are known to come from, at
    # most one unit per source, and never toward a source another unit is
    # already on its way to.  Crowding a pickup point is how a fleet jams a
    # chokepoint; a unit with nowhere useful to be is better off standing
    # still than driving through traffic it does not need.
    idle = [u for u in units if u.id not in goals]
    if idle:
        avail = sorted(s for s in _SOURCES if s not in claimed_nodes)
        taken: Set[int] = set()
        still_idle = []
        for u in idle:
            pos, _eta = _pos_eta(u)
            if pos in avail and pos not in taken:
                taken.add(pos)          # already parked on a useful spot
            else:
                still_idle.append(u)
        cands = []
        for u in still_idle:
            pos, eta = _pos_eta(u)
            for s in avail:
                d = _d(pos, s)
                if d == INF:
                    continue
                cands.append((eta + d, u.id, s))
        cands.sort()
        placed: Set[int] = set()
        for _c, uid, s in cands:
            if uid in placed or s in taken:
                continue
            goals[uid] = [s]
            placed.add(uid)
            taken.add(s)

    # Anything still without a goal stays put -- unless it is sitting on a
    # node with a finite capacity, where it would block later arrivals.
    for u in units:
        if u.id in goals:
            continue
        pos, _eta = _pos_eta(u)
        if _NODE_CAP.get(pos) is None:
            continue
        best = None
        for n in _FREE_PARK:
            d = _d(pos, n)
            if d == INF or d == 0:
                continue
            if best is None or (d, n) < best:
                best = (d, n)
        if best is not None:
            goals[u.id] = [best[1]]
    return goals


# ==========================================================================
# reservation table
# ==========================================================================

def _node_free(res_node, own, n: int, tau: int) -> bool:
    cap = _NODE_CAP.get(n)
    if cap is None or tau > WINDOW:
        return True
    key = (n, tau)
    return res_node.get(key, 0) - own.get(key, 0) < cap


def _leg_free(res_node, res_edge, own_n, own_e, u: int, v: int,
              tau: int, cost: int) -> bool:
    """Can this unit depart u at `tau` and hold the edge until it lands?"""
    ei = _EDGE_IDX.get((u, v))
    if ei is None:
        return False
    cap = _EDGE_CAP.get(ei)
    if cap is not None:
        last = min(tau + cost - 1, WINDOW)
        for t in range(tau, last + 1):
            key = (ei, t)
            if res_edge.get(key, 0) - own_e.get(key, 0) >= cap:
                return False
    ncap = _NODE_CAP.get(v)
    if ncap is not None:
        last = min(tau + cost, WINDOW)
        for t in range(tau, last + 1):
            key = (v, t)
            if res_node.get(key, 0) - own_n.get(key, 0) >= ncap:
                return False
    return True


def _add(table, key, delta) -> None:
    table[key] = table.get(key, 0) + delta


def _reserve(res_node, res_edge, segs, transit_from, sink_n=None, sink_e=None):
    """Write a plan's node/edge claims into the table.

    segs is [(node, arrival_tau, departure_tau or None)].  A unit holds a
    node from the moment it sets off toward it (inbound reserves a slot, as
    node_occupancy() does) until the step it leaves, exclusive; it holds an
    edge from departure until the step before it lands.
    """
    for i, (n, arrive, depart) in enumerate(segs):
        start = 0 if i == 0 else segs[i - 1][2]
        end = WINDOW if depart is None else depart - 1
        if _NODE_CAP.get(n) is not None:
            for t in range(max(0, start), min(end, WINDOW) + 1):
                _add(res_node, (n, t), 1)
                if sink_n is not None:
                    _add(sink_n, (n, t), 1)
        if i > 0:
            u = segs[i - 1][0]
            ei = _EDGE_IDX.get((u, n))
            if ei is not None and _EDGE_CAP.get(ei) is not None:
                for t in range(segs[i - 1][2], min(arrive - 1, WINDOW) + 1):
                    _add(res_edge, (ei, t), 1)
                    if sink_e is not None:
                        _add(sink_e, (ei, t), 1)
    if transit_from is not None:
        ei = _EDGE_IDX.get((transit_from, segs[0][0]))
        if ei is not None and _EDGE_CAP.get(ei) is not None:
            for t in range(0, min(segs[0][1] - 1, WINDOW) + 1):
                _add(res_edge, (ei, t), 1)
                if sink_e is not None:
                    _add(sink_e, (ei, t), 1)


# ==========================================================================
# space-time A*
# ==========================================================================

def _astar(start: int, start_tau: int, wps: List[int],
           res_node, res_edge, own_n, own_e,
           earliest_depart: int, deadline: float):
    """Monotone cooperative A*.  Returns segs, or None if nothing beats waiting."""
    k = 0
    while k < len(wps) and wps[k] == start:
        k += 1
    if k >= len(wps):
        return None

    suffix = [0.0] * (len(wps) + 1)
    for i in range(len(wps) - 2, -1, -1):
        step = _d(wps[i], wps[i + 1])
        if step == INF:
            wps = wps[:i + 1]
            suffix = suffix[:i + 2]
            break
        suffix[i] = step + suffix[i + 1]
    if k >= len(wps):
        return None

    h0 = _d(start, wps[k])
    if h0 == INF:
        return None

    root = (start, start_tau, k)
    best = {root: start_tau}
    parent: Dict[Tuple[int, int, int], Tuple[Tuple[int, int, int], bool]] = {}
    openq = [(start_tau + h0 + suffix[k], start_tau, start, k)]
    goal_state = None
    leaf_state = None
    leaf_f = INF
    expanded = 0

    while openq:
        f, tau, n, wp = heapq.heappop(openq)
        if best.get((n, tau, wp), INF) < tau:
            continue
        if wp >= len(wps):
            goal_state = (n, tau, wp)
            break
        expanded += 1
        if expanded > EXPANSIONS:
            break
        if (expanded & 255) == 0 and time.monotonic() > deadline:
            break
        if tau >= WINDOW:
            if f < leaf_f:
                leaf_f, leaf_state = f, (n, tau, wp)
            continue

        goal_n = wps[wp]
        dn = _dto(goal_n)
        here = dn.get(n, INF)

        # wait in place
        if _node_free(res_node, own_n, n, tau + 1):
            ns = (n, tau + 1, wp)
            if tau + 1 < best.get(ns, INF):
                best[ns] = tau + 1
                parent[ns] = ((n, tau, wp), False)
                heapq.heappush(openq, (tau + 1 + here + suffix[wp], tau + 1, n, wp))

        if tau < earliest_depart:
            continue

        for v, cost in _ADJ.get(n, ()):
            rest = dn.get(v, INF)
            if rest == INF or rest >= here:
                continue        # never accept a hop that loses ground
            if not _leg_free(res_node, res_edge, own_n, own_e, n, v, tau, cost):
                continue
            tau2 = tau + cost
            wp2 = wp
            while wp2 < len(wps) and wps[wp2] == v:
                wp2 += 1
            ns = (v, tau2, wp2)
            if tau2 >= best.get(ns, INF):
                continue
            if wp2 >= len(wps):
                hh = 0.0
            else:
                hh = _d(v, wps[wp2])
                if hh == INF:
                    continue
                hh += suffix[wp2]
            best[ns] = tau2
            parent[ns] = ((n, tau, wp), True)
            heapq.heappush(openq, (tau2 + hh, tau2, v, wp2))

    end = goal_state if goal_state is not None else leaf_state
    if end is None:
        return None

    # walk back, keeping only the moves
    chain = []
    cur = end
    while cur != root:
        prev, moved = parent[cur]
        if moved:
            chain.append((prev[0], prev[1], cur[0], cur[1]))
        cur = prev
    if not chain:
        return None
    chain.reverse()

    segs: List[List] = [[start, start_tau, None]]
    for frm, depart, to, arrive in chain:
        segs[-1][2] = depart
        segs.append([to, arrive, None])
    return [tuple(s) for s in segs]


# ==========================================================================
# per-step plan
# ==========================================================================

def _build_plan(state: GraphState) -> Dict[int, Optional[int]]:
    units = sorted(state.drive_units, key=lambda u: u.id)
    goals = _assign(state)

    res_node: Dict[Tuple[int, int], int] = {}
    res_edge: Dict[Tuple[int, int], int] = {}
    own_n: Dict[int, Dict[Tuple[int, int], int]] = {}
    own_e: Dict[int, Dict[Tuple[int, int], int]] = {}

    # Seed with what is physically true right now, so a unit that is planned
    # early cannot plan straight through a unit that is planned late.
    for u in units:
        mine_n: Dict[Tuple[int, int], int] = {}
        mine_e: Dict[Tuple[int, int], int] = {}
        if u.in_transit:
            dest = u.transit_destination
            arrive = max(1, int(math.ceil(u.transit_remaining_time)))
            if _NODE_CAP.get(dest) is not None:
                for t in range(0, min(arrive, WINDOW) + 1):
                    _add(res_node, (dest, t), 1)
                    _add(mine_n, (dest, t), 1)
            ei = _EDGE_IDX.get((u.current_node, dest))
            if ei is not None and _EDGE_CAP.get(ei) is not None:
                for t in range(0, min(arrive - 1, WINDOW) + 1):
                    _add(res_edge, (ei, t), 1)
                    _add(mine_e, (ei, t), 1)
        else:
            here = u.current_node
            if _NODE_CAP.get(here) is not None:
                span = 0 if u.id in goals else WINDOW
                for t in range(0, span + 1):
                    _add(res_node, (here, t), 1)
                    _add(mine_n, (here, t), 1)
        own_n[u.id] = mine_n
        own_e[u.id] = mine_e

    # Priority: a unit with a pod on board books its lane first, then units
    # on their way to collect one, and last the units with nothing to do.
    # Ordering by raw id instead lets an empty unit reserve a corridor out
    # from under a loaded one, which costs a delivery to save a deadhead.
    order = sorted(units, key=lambda u: (_priority(u, goals), u.id))
    deadline = time.monotonic() + PLAN_SECONDS

    first: Dict[int, Optional[int]] = {}
    for u in order:
        wps = goals.get(u.id)
        if u.in_transit:
            start = u.transit_destination
            start_tau = max(1, int(math.ceil(u.transit_remaining_time)))
            transit_from = u.current_node
        else:
            start = u.current_node
            start_tau = 0
            transit_from = None

        segs = None
        if wps and time.monotonic() < deadline:
            segs = _astar(start, start_tau, list(wps), res_node, res_edge,
                          own_n[u.id], own_e[u.id], 0, deadline)
        if segs is None:
            segs = [(start, start_tau, None)]

        # swap this unit's physical seed for its full plan
        for key, cnt in own_n[u.id].items():
            res_node[key] = res_node.get(key, 0) - cnt
        for key, cnt in own_e[u.id].items():
            res_edge[key] = res_edge.get(key, 0) - cnt
        own_n[u.id] = {}
        own_e[u.id] = {}
        _reserve(res_node, res_edge, segs, transit_from)

        if (not u.in_transit) and len(segs) > 1 and segs[0][2] == 0:
            first[u.id] = segs[1][0]
        else:
            first[u.id] = None
    return first


def _plan_signature(state: GraphState):
    return (_SIG,
            state.current_time_step,
            len(state.active_pods),
            len(state.delivered_pods),
            tuple(sorted((u.id, u.current_node, len(u.carrying))
                         for u in state.drive_units)))


def _ensure_plan(state: GraphState) -> None:
    global _PLAN_KEY, _PLAN_FIRST
    key = _plan_signature(state)
    if key == _PLAN_KEY:
        return
    _PLAN_FIRST = _build_plan(state)
    _PLAN_KEY = key


# ==========================================================================
# legality against the live board
# ==========================================================================

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


def _decide(drive_unit_id: int, state: GraphState) -> Optional[int]:
    _build(state)
    unit = state.get_drive_unit(drive_unit_id)
    if unit is None or unit.in_transit:
        return None
    _ensure_plan(state)
    nxt = _PLAN_FIRST.get(drive_unit_id)
    if nxt is None:
        return None
    if nxt == unit.current_node:
        return None
    if not _can_enter(state, unit, nxt):
        return None
    return nxt


def drive_unit_next_move(drive_unit_id: int, state: GraphState) -> Optional[int]:
    try:
        return _decide(drive_unit_id, state)
    except Exception:
        return None
