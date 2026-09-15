"""
Amazon Robotics Hackathon - Routing API

*****IMPORTANT*****
Team name: vibe coders
Email address: alexliu22111@gmail.com
*******************

Strategy: centralized greedy planner, replanned every call.
- All-pairs Dijkstra (cost = ceil(edge.weight) = true traversal steps), cached per graph.
- Sticky pod assignments in module state (persists between calls).
- Carrying units head to nearest carried-pod destination; batching for multi-capacity units.
- Occupancy-aware: validates moves like the referee; detours around blocked edges/nodes.
- Deadlock breaker: after repeated blocks, sidestep to any valid neighbor.
"""

import heapq
import math
from typing import Optional
from ar_hackathon.models.graph_state import GraphState

# ---------------- module-level persistent state ----------------
_cache = {
    "sig": None,        # graph signature
    "adj": None,        # node -> list[(nbr, cost)]
    "dist": None,       # (src -> {node: dist}) lazy dijkstra results
    "assign": {},       # unit_id -> pod_id claims
    "blocked": {},      # unit_id -> (node, target, count) consecutive-block tracker
    "last_t": None,
}


def _graph_sig(state):
    edges = tuple(sorted((e.from_node, e.to_node, e.weight, e.capacity, e.bidirectional)
                         for e in state.edges))
    nodes = tuple(sorted((n.id, n.capacity, n.node_type) for n in state.nodes))
    return hash((edges, nodes))


def _build_adj(state):
    adj = {n.id: [] for n in state.nodes}
    for e in state.edges:
        cost = max(1, math.ceil(e.weight))
        adj.setdefault(e.from_node, []).append((e.to_node, cost))
        if e.bidirectional:
            adj.setdefault(e.to_node, []).append((e.from_node, cost))
    return adj


def _dijkstra(adj, src, skip_edge=None, skip_node=None):
    """dist map from src. skip_edge=(a,b) unordered pair, skip_node blocks entry (not src)."""
    dist = {src: 0}
    prev = {}
    pq = [(0, src)]
    while pq:
        d, u = heapq.heappop(pq)
        if d > dist.get(u, float("inf")):
            continue
        for v, c in adj.get(u, ()):
            if skip_node is not None and v == skip_node:
                continue
            if skip_edge is not None and {u, v} == set(skip_edge):
                continue
            nd = d + c
            if nd < dist.get(v, float("inf")):
                dist[v] = nd
                prev[v] = u
                heapq.heappush(pq, (nd, v))
    return dist, prev


def _dist_from(state, src):
    """Cached full dijkstra from src on the clean graph."""
    if src not in _cache["dist"]:
        d, p = _dijkstra(_cache["adj"], src)
        _cache["dist"][src] = (d, p)
    return _cache["dist"][src]


def _path_next_hop(state, src, dst, skip_edge=None, skip_node=None):
    """First hop of shortest src->dst path, or None if unreachable."""
    if src == dst:
        return None
    if skip_edge is None and skip_node is None:
        d, prev = _dist_from(state, src)
    else:
        d, prev = _dijkstra(_cache["adj"], src, skip_edge, skip_node)
    if dst not in d:
        return None
    node = dst
    while prev.get(node) != src:
        node = prev.get(node)
        if node is None:
            return None
    return node


def _dd(state, a, b):
    """Shortest distance a->b (clean graph), inf if unreachable."""
    d, _ = _dist_from(state, a)
    return d.get(b, float("inf"))


def _move_ok(state, unit, nxt):
    """Mirror the referee's is_valid_move using current occupancy."""
    edge = state.get_edge(unit.current_node, nxt)
    if edge is None:
        return False
    if edge.capacity is not None and state.edge_occupancy(unit.current_node, nxt) >= edge.capacity:
        return False
    node = state.get_node(nxt)
    if node is not None and node.capacity is not None and state.node_occupancy(nxt) >= node.capacity:
        return False
    return True


def _reset_if_new_game(state):
    sig = _graph_sig(state)
    t = state.current_time_step
    if sig != _cache["sig"] or (_cache["last_t"] is not None and t < _cache["last_t"]):
        _cache["sig"] = sig
        _cache["adj"] = _build_adj(state)
        _cache["dist"] = {}
        _cache["assign"] = {}
        _cache["blocked"] = {}
    _cache["last_t"] = t


def _claimed_pods(state, me_id):
    """Pod ids claimed by other units (assigned or being carried)."""
    claimed = set()
    for u in state.drive_units:
        for pid in u.carrying:
            claimed.add(pid)
    for uid, pid in _cache["assign"].items():
        if uid != me_id:
            claimed.add(pid)
    return claimed


def _validate_assignment(state, unit):
    """Drop my claim if the pod is gone or grabbed by someone else."""
    pid = _cache["assign"].get(unit.id)
    if pid is None:
        return None
    pod = None
    for p in state.active_pods:
        if p.id == pid:
            pod = p
            break
    if pod is None or pod.carried_by is not None or pod.current_node is None:
        _cache["assign"].pop(unit.id, None)
        return None
    return pod


def _pick_new_pod(state, unit):
    """Claim nearest unclaimed waiting pod; ties -> older entry_time, lower id."""
    claimed = _claimed_pods(state, unit.id)
    best = None
    best_key = None
    for p in state.active_pods:
        if p.carried_by is not None or p.current_node is None or p.id in claimed:
            continue
        d = _dd(state, unit.current_node, p.current_node)
        d2 = _dd(state, p.current_node, p.destination_station)
        if d == float("inf") or d2 == float("inf"):
            continue
        key = (d + d2, p.entry_time, p.id)
        if best_key is None or key < best_key:
            best_key = key
            best = p
    if best is not None:
        _cache["assign"][unit.id] = best.id
    return best


def _choose_target(state, unit):
    """Target node: nearest among carried-pod destinations and (if capacity) assigned pickup."""
    candidates = []
    for pid in unit.carrying:
        pod = state.get_pod(pid)
        if pod is not None:
            candidates.append(pod.destination_station)
    if unit.has_capacity:
        pod = _validate_assignment(state, unit)
        if pod is None and not unit.carrying:
            pod = _pick_new_pod(state, unit)
        elif pod is None and unit.carrying:
            # batching: only chase an extra pod if one is claimable
            pod = _pick_new_pod(state, unit)
        if pod is not None:
            candidates.append(pod.current_node)
    if not candidates:
        return None
    return min(candidates, key=lambda n: _dd(state, unit.current_node, n))


def _idle_step_off_station(state, unit):
    """Idle units should not squat on capacity-limited stations/storage."""
    node = state.get_node(unit.current_node)
    if node is None or node.capacity is None or node.node_type == "travel":
        return None
    for nbr, _c in sorted(_cache["adj"].get(unit.current_node, ()), key=lambda x: x[1]):
        nb = state.get_node(nbr)
        if nb is not None and nb.node_type == "travel" and _move_ok(state, unit, nbr):
            return nbr
    return None


def drive_unit_next_move(drive_unit_id: int, state: GraphState) -> Optional[int]:
    try:
        return _next_move(drive_unit_id, state)
    except Exception:
        return None


def _next_move(drive_unit_id, state):
    _reset_if_new_game(state)
    unit = state.get_drive_unit(drive_unit_id)
    if unit is None or unit.in_transit:
        return None

    target = _choose_target(state, unit)
    if target is None or target == unit.current_node:
        return _idle_step_off_station(state, unit)

    hop = _path_next_hop(state, unit.current_node, target)
    if hop is not None and _move_ok(state, unit, hop):
        _cache["blocked"].pop(drive_unit_id, None)
        return hop

    # Blocked: try detour around the offending edge/node
    if hop is not None:
        detour = _path_next_hop(state, unit.current_node, target,
                                skip_edge=(unit.current_node, hop))
        if detour is not None and _move_ok(state, unit, detour):
            _cache["blocked"].pop(drive_unit_id, None)
            return detour
        detour = _path_next_hop(state, unit.current_node, target, skip_node=hop)
        if detour is not None and _move_ok(state, unit, detour):
            _cache["blocked"].pop(drive_unit_id, None)
            return detour

    # Track consecutive blocks; sidestep to break symmetric deadlocks
    node_now = unit.current_node
    prev = _cache["blocked"].get(drive_unit_id)
    count = prev[2] + 1 if prev and prev[0] == node_now and prev[1] == target else 1
    _cache["blocked"][drive_unit_id] = (node_now, target, count)
    if count >= 3:
        for nbr, _c in _cache["adj"].get(node_now, ()):
            if _move_ok(state, unit, nbr):
                _cache["blocked"].pop(drive_unit_id, None)
                return nbr
    return None
