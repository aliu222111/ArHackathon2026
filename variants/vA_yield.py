"""
Amazon Robotics Hackathon - Routing API  (variant vA_yield)

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
- NEW (vA_yield): head-on mutual-block RECOVERY via passing bays. When two units
  face each other across a capacity-1 pinch (each unit's next hop is the other's
  node and both hops are invalid, with no improving detour for either), exactly
  one of the pair - higher ID if it has an escape, else the lower - pulls into a
  side bay (a distance-INCREASING move, normally forbidden by the no-retreat
  rule) and HOLDS there until the opposing unit has strictly passed the shared
  section, then re-enters. Railway passing-loop protocol + static-priority
  symmetry breaking.
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
    "yield": {},        # unit_id -> {"bay","home","blocker","t"} passing-bay holds
    "last_t": None,
}

_WAIT = object()  # sentinel: head-on recognized, counterpart will yield


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
        _cache["yield"] = {}
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
    if not unit.has_capacity:
        # full unit can't pick this pod up - release the claim so others can
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
        if d == float("inf"):
            continue
        key = (d, p.entry_time, p.id)
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
            pod = None  # no batching: deliver first, then claim
        if pod is not None:
            candidates.append(pod.current_node)
    if not candidates:
        return None
    return min(candidates, key=lambda n: _dd(state, unit.current_node, n))


def _peek_target(state, u):
    """Read-only view of a unit's current target (no claiming side effects)."""
    cands = []
    for pid in u.carrying:
        pod = state.get_pod(pid)
        if pod is not None:
            cands.append(pod.destination_station)
    if not cands and u.has_capacity:
        pid = _cache["assign"].get(u.id)
        if pid is not None:
            for p in state.active_pods:
                if p.id == pid and p.carried_by is None and p.current_node is not None:
                    cands.append(p.current_node)
                    break
    if not cands:
        return None
    return min(cands, key=lambda n: _dd(state, u.current_node, n))


def _escape_bay(state, mover, other, mover_target):
    """Best passing-bay neighbor for mover: valid, not other's node, never a
    capacity-limited station, and if on other's route it must keep spare room.
    Prefer off-route bays, then closest to mover's own target."""
    otgt = _peek_target(state, other)
    opath = set()
    if otgt is not None and otgt != other.current_node:
        _d, prev = _dist_from(state, other.current_node)
        n = otgt if otgt in _d else None
        while n is not None and n != other.current_node:
            opath.add(n)
            n = prev.get(n)
    best = None
    best_key = None
    for nbr, _c in _cache["adj"].get(mover.current_node, ()):
        if nbr == other.current_node:
            continue
        if not _move_ok(state, mover, nbr):
            continue
        nb = state.get_node(nbr)
        if nb is not None and nb.node_type == "station" and nb.capacity is not None:
            continue
        on_path = nbr in opath
        if on_path:
            cap = nb.capacity if nb is not None else None
            if cap is not None and state.node_occupancy(nbr) + 1 >= cap:
                continue  # entering would plug the other's route
        back = _dd(state, nbr, mover_target) if mover_target is not None else 0
        key = (1 if on_path else 0, back, nbr)
        if best_key is None or key < best_key:
            best_key = key
            best = nbr
    return best


def _resolve_headon(state, me, occ, my_target):
    """occ stands at my next hop. If we mutually block each other with no
    improving detour for occ, exactly one of us yields into a bay.
    Returns: bay node to move to, _WAIT (counterpart will yield), or None."""
    otgt = _peek_target(state, occ)
    if otgt is None or otgt == occ.current_node:
        return None
    ohop = _path_next_hop(state, occ.current_node, otgt)
    if ohop != me.current_node:
        return None            # not opposing traffic
    if _move_ok(state, occ, ohop):
        return None            # it can actually enter my node: no deadlock
    od = _dd(state, occ.current_node, otgt)
    for nbr, _c in _cache["adj"].get(occ.current_node, ()):
        if nbr != me.current_node and _move_ok(state, occ, nbr) \
                and _dd(state, nbr, otgt) < od:
            return None        # occ has its own improving detour; let it take it
    my_bay = _escape_bay(state, me, occ, my_target)
    occ_bay = _escape_bay(state, occ, me, otgt)
    if my_bay is not None and (occ_bay is None or me.id > occ.id):
        _cache["yield"][me.id] = {"bay": my_bay, "home": me.current_node,
                                  "blocker": occ.id, "t": state.current_time_step}
        return my_bay
    if occ_bay is not None:
        return _WAIT
    return None


def _bay_hold(state, unit, y):
    """Parked in a bay: hold until the blocker strictly passes the shared
    section, then re-enter. Returns _WAIT, a move, or None (released)."""
    held = state.current_time_step - y["t"]
    tgt = _peek_target(state, unit)
    if tgt is None or tgt == unit.current_node:
        _cache["yield"].pop(unit.id, None)
        return None
    hop = _path_next_hop(state, unit.current_node, tgt)
    if hop is None:
        _cache["yield"].pop(unit.id, None)
        return None
    blocker = state.get_drive_unit(y["blocker"])
    if blocker is None:
        passed, bpos = True, None
    else:
        bpos = blocker.transit_destination if blocker.in_transit else blocker.current_node
        btgt = _peek_target(state, blocker)
        if btgt is None:
            passed = (bpos != hop and bpos != y["home"])
        else:
            passed = _dd(state, bpos, btgt) < _dd(state, y["home"], btgt)
    if passed and bpos != hop and _move_ok(state, unit, hop):
        _cache["yield"].pop(unit.id, None)
        return hop
    if held > 30:
        _cache["yield"].pop(unit.id, None)
        return None
    return _WAIT


def _idle_step_off_station(state, unit):
    """Idle: drift toward nearest storage node (pods spawn there), else clear stations."""
    storages = [n.id for n in state.nodes if n.node_type == "storage"]
    if storages and unit.current_node not in storages:
        tgt = min(storages, key=lambda s: _dd(state, unit.current_node, s))
        if _dd(state, unit.current_node, tgt) != float("inf"):
            hop = _path_next_hop(state, unit.current_node, tgt)
            if hop is not None and _move_ok(state, unit, hop):
                nb = state.get_node(hop)
                # don't enter a capacity-limited station while idle
                if nb is None or nb.node_type != "station":
                    return hop
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

    # Parked in a passing bay? Hold until the blocker passes, then re-enter.
    y = _cache["yield"].get(drive_unit_id)
    if y is not None:
        if unit.current_node != y["bay"]:
            _cache["yield"].pop(drive_unit_id, None)  # yield move never landed
        else:
            mv = _bay_hold(state, unit, y)
            if mv is _WAIT:
                return None
            if mv is not None:
                return mv
            # released with no move: fall through to normal routing

    target = _choose_target(state, unit)
    if target is None or target == unit.current_node:
        return _idle_step_off_station(state, unit)

    hop = _path_next_hop(state, unit.current_node, target)
    if hop is not None and _move_ok(state, unit, hop):
        _cache["blocked"].pop(drive_unit_id, None)
        return hop

    # Blocked: try detour around the offending edge/node.
    # No-retreat: a detour first-hop that increases distance-to-goal invites
    # mirrored flip-flopping across capacity-1 aisles; wait instead.
    if hop is not None:
        here_d = _dd(state, unit.current_node, target)
        detour = _path_next_hop(state, unit.current_node, target,
                                skip_edge=(unit.current_node, hop))
        if (detour is not None and _move_ok(state, unit, detour)
                and _dd(state, detour, target) < here_d):
            _cache["blocked"].pop(drive_unit_id, None)
            return detour
        detour = _path_next_hop(state, unit.current_node, target, skip_node=hop)
        if (detour is not None and _move_ok(state, unit, detour)
                and _dd(state, detour, target) < here_d):
            _cache["blocked"].pop(drive_unit_id, None)
            return detour

    # Track consecutive blocks; sidestep to break symmetric deadlocks
    node_now = unit.current_node
    prev = _cache["blocked"].get(drive_unit_id)
    count = prev[2] + 1 if prev and prev[0] == node_now and prev[1] == target else 1
    _cache["blocked"][drive_unit_id] = (node_now, target, count)

    # Head-on mutual-block RECOVERY: the one exception to no-retreat.
    if hop is not None:
        occ = None
        for u2 in state.drive_units:
            if u2.id != drive_unit_id and not u2.in_transit and u2.current_node == hop:
                occ = u2
                break
        if occ is not None:
            mv = _resolve_headon(state, unit, occ, target)
            if mv is _WAIT:
                return None
            if mv is not None:
                _cache["blocked"].pop(drive_unit_id, None)
                return mv

    # Queueing for a dock (blocked hop IS the target) is correct waiting - never sidestep.
    if hop == target:
        return None
    if count >= 3:
        # sidestep, but never into a capacity-limited station (don't block docks)
        options = []
        here_d = _dd(state, node_now, target)
        for nbr, _c in _cache["adj"].get(node_now, ()):
            if not _move_ok(state, unit, nbr):
                continue
            # no-retreat: a sidestep that doesn't strictly reduce distance-to-goal
            # invites mirrored ping-pong across capacity-1 aisles; wait instead
            if _dd(state, nbr, target) >= here_d:
                continue
            nb = state.get_node(nbr)
            if nb is not None and nb.node_type == "station" and nb.capacity is not None:
                continue
            options.append(nbr)
        if options:
            best = min(options, key=lambda n: _dd(state, n, target))
            _cache["blocked"].pop(drive_unit_id, None)
            return best
    return None
