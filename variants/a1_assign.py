"""
Amazon Robotics Hackathon - Routing API

*****IMPORTANT*****
Team name: vibe coders
Email address: alexliu22111@gmail.com
*******************

Centralized planner focused on the unit->pod ASSIGNMENT policy.

  * lazy reverse-Dijkstra distance oracle over ceil(weight) edge costs
  * assignment scored directly in the grader's currency -- the marginal
    exp(-flow/50) value a unit adds by taking a pod -- instead of a raw
    distance proxy with an ad-hoc age bonus
  * optimal (brute-forced) stop ordering for the pods a unit already
    carries plus the pod it is being offered, so multi-capacity units
    sequence their dock visits instead of always running to the nearest
  * optimal min-cost matching (hand-rolled Hungarian) over those values
  * idle units are forbidden from consuming scarce node capacity, so
    empty units can never wall off a loaded one
  * strict no-retreat movement rule (a hop must reduce distance-to-goal,
    else the unit waits) which prevents head-on capacity-1 ping-pong
"""

import heapq
import math
from itertools import permutations
from typing import Dict, List, Optional, Tuple

from ar_hackathon.models.graph_state import GraphState

INF = float("inf")
DECAY = 1.0 / 50.0          # matches the grader's exp(-flow/50)
STICKY = 1.03               # bonus for keeping last step's assignment
MAX_EVENTS = 6              # brute-force stop orderings up to this many stops
MAX_PAIRS = 4000            # above this, drop to the cheap greedy matcher

# ---------------------------------------------------------------- graph cache
_SIG = None
_ADJ: Dict[int, List[Tuple[int, int]]] = {}
_RADJ: Dict[int, List[Tuple[int, int]]] = {}
_RDIST: Dict[int, Dict[int, float]] = {}
_STORAGE: List[int] = []
_DESTS = set()
_SAFE = None
_SAFE_KEY = None

# ------------------------------------------------------------------ run state
_PLAN_T = -1
_PLAN: Dict[int, int] = {}
_PARK: Dict[int, int] = {}
_ASSIGN: Dict[int, str] = {}
_SPAWN: Dict[int, int] = {}
_SEEN = set()
_LAST_T = -1


def _reset_run() -> None:
    global _PLAN_T, _PLAN, _PARK, _ASSIGN, _SPAWN, _SEEN, _LAST_T
    global _DESTS, _SAFE, _SAFE_KEY
    _DESTS = set()
    _SAFE = None
    _SAFE_KEY = None
    _PLAN_T = -1
    _PLAN = {}
    _PARK = {}
    _ASSIGN = {}
    _SPAWN = {}
    _SEEN = set()
    _LAST_T = -1


def _signature(state: GraphState):
    return (
        tuple(sorted((n.id, n.node_type, n.capacity) for n in state.nodes)),
        tuple(sorted((e.from_node, e.to_node, e.weight, e.capacity, e.bidirectional)
                     for e in state.edges)),
    )


def _build(state: GraphState) -> None:
    """Rebuild the static graph caches when a new test case shows up."""
    global _SIG, _ADJ, _RADJ, _RDIST, _STORAGE, _LAST_T
    sig = _signature(state)
    if sig != _SIG:
        adj: Dict[int, List[Tuple[int, int]]] = {n.id: [] for n in state.nodes}
        radj: Dict[int, List[Tuple[int, int]]] = {n.id: [] for n in state.nodes}
        for e in state.edges:
            cost = int(math.ceil(e.weight))
            if cost < 1:
                cost = 1
            adj.setdefault(e.from_node, []).append((e.to_node, cost))
            radj.setdefault(e.to_node, []).append((e.from_node, cost))
            if e.bidirectional:
                adj.setdefault(e.to_node, []).append((e.from_node, cost))
                radj.setdefault(e.from_node, []).append((e.to_node, cost))
        for k in adj:
            adj[k].sort()
        _SIG, _ADJ, _RADJ = sig, adj, radj
        _RDIST = {}
        _STORAGE = sorted(n.id for n in state.nodes if n.node_type == "storage")
        _reset_run()
        for n in state.nodes:
            if n.node_type == "station":
                _DESTS.add(n.id)
    elif state.current_time_step < _LAST_T:
        _reset_run()
    _LAST_T = state.current_time_step


def _dto(target: int) -> Dict[int, float]:
    """Distance from every node TO target (one Dijkstra on the reverse graph)."""
    cached = _RDIST.get(target)
    if cached is not None:
        return cached
    dist = {target: 0.0}
    pq = [(0.0, target)]
    seen = set()
    while pq:
        cd, u = heapq.heappop(pq)
        if u in seen:
            continue
        seen.add(u)
        for v, w in _RADJ.get(u, ()):
            nd = cd + w
            if nd < dist.get(v, INF):
                dist[v] = nd
                heapq.heappush(pq, (nd, v))
    _RDIST[target] = dist
    return dist


def _safe_nodes():
    """
    Nodes from which some delivery destination is still reachable.

    One-way aisles can make a node a roach motel: a unit that wanders in
    never gets out, and any pod it auto-picks up there is lost for good.
    Never walk into one voluntarily.
    """
    global _SAFE, _SAFE_KEY
    key = tuple(sorted(_DESTS))
    if _SAFE_KEY == key and _SAFE is not None:
        return _SAFE
    out = set()
    for d in _DESTS:
        out.update(_dto(d).keys())
    _SAFE, _SAFE_KEY = out, key
    return out


def _d(a: int, b: int) -> float:
    if a == b:
        return 0.0
    return _dto(b).get(a, INF)


# --------------------------------------------------------------- route scoring
def _route(pos: int, t0: float, carried, pickups, cap: int):
    """
    Best achievable value for one unit's remaining work.

    carried: [(destination, entry_time)] pods already aboard
    pickups: [(node, destination, entry_time)] pods we are considering taking
    Returns (value, first_stop) where value is the sum of exp(-flow/50) the
    unit would bank, and first_stop the node it must head for next.

    A leg of length w started at step s lands at step s+w-1 (the engine
    decrements transit time before it resolves deliveries), hence the -1.
    """
    if not carried and not pickups:
        return 0.0, None

    pick_at: Dict[int, List[Tuple[int, int]]] = {}
    for node, dest, entry in pickups:
        pick_at.setdefault(node, []).append((dest, entry))

    stops = set(pick_at)
    for dest, _e in carried:
        stops.add(dest)
    for _n, dest, _e in pickups:
        stops.add(dest)
    events = sorted(stops)

    if len(events) > MAX_EVENTS:
        orders = [_greedy_order(pos, events, pick_at, carried)]
    else:
        orders = permutations(events)

    best_v = None
    best_first = None
    for perm in orders:
        if perm is None:
            continue
        t = t0
        cur = pos
        val = 0.0
        hold = list(carried)
        pending = {k: list(v) for k, v in pick_at.items()}
        first = None
        ok = True
        for node in perm:
            step = _d(cur, node)
            if step == INF:
                ok = False
                break
            t += step
            arrive = t - 1.0
            if first is None and node != pos:
                first = node
            keep = []
            for dest, entry in hold:
                if dest == node:
                    val += math.exp(-DECAY * (arrive - entry))
                else:
                    keep.append((dest, entry))
            hold = keep
            taking = pending.pop(node, None)
            if taking:
                if len(hold) + len(taking) > cap:
                    ok = False
                    break
                hold.extend(taking)
            cur = node
        if not ok or hold:
            continue
        if best_v is None or val > best_v:
            best_v = val
            best_first = first
    if best_v is None:
        return None, None
    return best_v, best_first


def _greedy_order(pos, events, pick_at, carried):
    """Nearest-stop fallback ordering, pickups before their deliveries."""
    remaining = list(events)
    order = []
    cur = pos
    blocked = set()
    for node, items in pick_at.items():
        for dest, _e in items:
            if dest != node:
                blocked.add(dest)
    held = set(dest for dest, _e in carried)
    while remaining:
        best = None
        for node in remaining:
            if node in blocked and node not in held:
                continue
            dd = _d(cur, node)
            if dd == INF:
                continue
            if best is None or dd < best[0]:
                best = (dd, node)
        if best is None:
            return None
        node = best[1]
        remaining.remove(node)
        order.append(node)
        cur = node
        if node in pick_at:
            for dest, _e in pick_at[node]:
                blocked.discard(dest)
                held.add(dest)
    return tuple(order)


# -------------------------------------------------------------- assignment
def _hungarian(cost: List[List[float]]) -> List[int]:
    """Min-cost perfect matching on rows (len(rows) <= len(cols)). O(n^2 m)."""
    n = len(cost)
    m = len(cost[0]) if n else 0
    u = [0.0] * (n + 1)
    v = [0.0] * (m + 1)
    p = [0] * (m + 1)
    way = [0] * (m + 1)
    for i in range(1, n + 1):
        p[0] = i
        j0 = 0
        minv = [INF] * (m + 1)
        used = [False] * (m + 1)
        while True:
            used[j0] = True
            i0 = p[j0]
            delta = INF
            j1 = -1
            row = cost[i0 - 1]
            ui = u[i0]
            for j in range(1, m + 1):
                if not used[j]:
                    cur = row[j - 1] - ui - v[j]
                    if cur < minv[j]:
                        minv[j] = cur
                        way[j] = j0
                    if minv[j] < delta:
                        delta = minv[j]
                        j1 = j
            if j1 < 0:
                break
            for j in range(m + 1):
                if used[j]:
                    u[p[j]] += delta
                    v[j] -= delta
                else:
                    minv[j] -= delta
            j0 = j1
            if p[j0] == 0:
                break
        while j0:
            j1 = way[j0]
            p[j0] = p[j1]
            j0 = j1
    out = [-1] * n
    for j in range(1, m + 1):
        if p[j] > 0:
            out[p[j] - 1] = j - 1
    return out


def _match(uids: List[int], pids: List[str], gain: Dict[Tuple[int, str], float]):
    """Max-total-value matching between units and waiting pods."""
    if not uids or not pids:
        return {}
    pairs = len(uids) * len(pids)
    if pairs > MAX_PAIRS:
        ranked = sorted(((-g, uid, pid) for (uid, pid), g in gain.items()))
        taken_u, taken_p, out = set(), set(), {}
        for _neg, uid, pid in ranked:
            if uid in taken_u or pid in taken_p:
                continue
            out[uid] = pid
            taken_u.add(uid)
            taken_p.add(pid)
        return out

    flip = len(uids) > len(pids)
    rows, cols = (pids, uids) if flip else (uids, pids)
    cost = []
    for a in rows:
        line = []
        for b in cols:
            key = (b, a) if flip else (a, b)
            g = gain.get(key)
            line.append(1e6 if g is None else -g)
        cost.append(line)
    res = _hungarian(cost)
    out = {}
    for i, j in enumerate(res):
        if j < 0:
            continue
        uid = cols[j] if flip else rows[i]
        pid = rows[i] if flip else cols[j]
        if gain.get((uid, pid)) is None:
            continue
        out[uid] = pid
    return out


def _make_plan(state: GraphState) -> None:
    global _PLAN, _PARK, _ASSIGN
    now = float(state.current_time_step)
    pods = {p.id: p for p in state.active_pods}

    here_of = {}
    for u in state.drive_units:
        if not u.in_transit:
            here_of[u.id] = u.current_node
    for p in state.active_pods:
        if p.id in _SEEN:
            continue
        _SEEN.add(p.id)
        src = p.current_node
        if src is None:
            # Picked up the instant it spawned: its source is wherever the
            # unit now holding it is standing. Still an observation, not a
            # peek at the future.
            src = here_of.get(p.carried_by)
        if src is not None:
            _SPAWN[src] = _SPAWN.get(src, 0) + 1
        _DESTS.add(p.destination_station)

    units = sorted(state.drive_units, key=lambda x: x.id)
    info = {}
    for u in units:
        if u.in_transit:
            pos = u.transit_destination
            t0 = now + max(1, int(math.ceil(u.transit_remaining_time)))
        else:
            pos = u.current_node
            t0 = now
        carried = []
        for pid in u.carrying:
            p = pods.get(pid)
            if p is None:
                continue
            if _d(pos, p.destination_station) == INF:
                continue          # stuck with it; do not let it steer the unit
            carried.append((p.destination_station, p.entry_time))
        base_v, base_first = _route(pos, t0, carried, [], u.capacity)
        if base_v is None:
            base_v, base_first = 0.0, None
        info[u.id] = (pos, t0, carried, base_v, base_first)

    waiting = []
    for p in state.active_pods:
        if p.carried_by is not None or p.current_node is None:
            continue
        if _d(p.current_node, p.destination_station) == INF:
            continue              # nobody can ever score it
        waiting.append(p)

    free = [u for u in units if u.has_capacity]
    gain: Dict[Tuple[int, str], float] = {}
    first_hop: Dict[Tuple[int, str], Optional[int]] = {}
    for u in free:
        pos, t0, carried, base_v, _bf = info[u.id]
        for p in waiting:
            if _d(pos, p.current_node) == INF:
                continue
            v, first = _route(pos, t0, carried,
                              [(p.current_node, p.destination_station, p.entry_time)],
                              u.capacity)
            if v is None:
                continue
            g = v - base_v
            if g <= 0.0:
                g = 1e-12
            if _ASSIGN.get(u.id) == p.id:
                g *= STICKY
            gain[(u.id, p.id)] = g
            first_hop[(u.id, p.id)] = first

    chosen = _match([u.id for u in free], [p.id for p in waiting], gain)

    plan: Dict[int, int] = {}
    idle: List[int] = []
    for u in units:
        pid = chosen.get(u.id)
        target = None
        if pid is not None:
            target = first_hop.get((u.id, pid))
        if target is None:
            target = info[u.id][4]
        if target is None:
            idle.append(u.id)
        else:
            plan[u.id] = target

    park: Dict[int, int] = {}
    spots = set(_SPAWN)
    spots.update(_STORAGE)
    safe = _safe_nodes()
    if safe:
        spots &= safe
    if spots and idle:
        ranked = sorted(spots)
        for uid in idle:
            pos = info[uid][0]
            best = None
            for s in ranked:
                dd = _d(pos, s)
                if dd == INF:
                    continue
                # How much a future pod appearing at s is worth if we sit
                # here: how often pods have shown up there, discounted by
                # how far away we would be when one does.
                w = _SPAWN.get(s, 0) + (0.5 if s in _STORAGE else 0.0)
                score = w * math.exp(-DECAY * dd)
                if score <= 0.0:
                    continue
                if best is None or score > best[0]:
                    best = (score, -dd, s)
            if best is not None and best[2] != pos:
                park[uid] = best[2]

    _PLAN = plan
    _PARK = park
    _ASSIGN = dict(chosen)


# ------------------------------------------------------------------- movement
def _enterable(state: GraphState, unit, nxt: int, idle: bool) -> bool:
    edge = state.get_edge(unit.current_node, nxt)
    if edge is None:
        return False
    if edge.capacity is not None:
        if state.edge_occupancy(unit.current_node, nxt) >= edge.capacity:
            return False
    node = state.get_node(nxt)
    if node is not None and node.capacity is not None:
        limit = node.capacity - 1 if idle else node.capacity
        if state.node_occupancy(nxt) >= limit:
            return False
    return True


def _advance(state: GraphState, unit, goal: int, idle: bool = False) -> Optional[int]:
    """
    Best legal hop toward goal.

    A blocked first choice must never be answered by retreating: a hop that
    leaves the unit further from its goal than it already is undoes real
    progress, and when two units do it to each other across a capacity-1
    aisle they ping-pong forever and deliver nothing. Waiting is a legal
    move and is the correct one here -- hold position until the aisle frees.
    """
    here = unit.current_node
    dg = _dto(goal)
    here_rest = dg.get(here, INF)
    if here_rest == INF:
        return None
    safe = _safe_nodes()
    guard = bool(safe) and here in safe
    cands = []
    for nxt, w in _ADJ.get(here, ()):
        if guard and nxt not in safe:
            continue
        rest = dg.get(nxt, INF)
        if rest == INF or rest >= here_rest:
            continue
        cands.append((w + rest, nxt))
    cands.sort()
    for _score, nxt in cands:
        if _enterable(state, unit, nxt, idle):
            return nxt
    return None


def _park_move(state: GraphState, unit) -> Optional[int]:
    """
    Idle units reposition toward where pods keep appearing, but they are
    never allowed to spend the last slot of a capacity-limited node: an
    empty unit parked in a choke point starves the loaded unit behind it.
    """
    here = unit.current_node
    target = _PARK.get(unit.id)
    if target is not None and target != here:
        step = _advance(state, unit, target, idle=True)
        if step is not None:
            return step
    node = state.get_node(here)
    if node is not None and node.capacity is not None:
        safe = _safe_nodes()
        guard = bool(safe) and here in safe
        opts = []
        for nxt, w in _ADJ.get(here, ()):
            if guard and nxt not in safe:
                continue
            nd = state.get_node(nxt)
            pen = 0 if (nd is None or nd.capacity is None) else 1
            opts.append((pen, w, nxt))
        opts.sort()
        for _pen, _w, nxt in opts:
            if _enterable(state, unit, nxt, False):
                return nxt
    return None


def _decide(drive_unit_id: int, state: GraphState) -> Optional[int]:
    global _PLAN_T
    _build(state)
    unit = state.get_drive_unit(drive_unit_id)
    if unit is None or unit.in_transit:
        return None
    if _PLAN_T != state.current_time_step:
        _PLAN_T = state.current_time_step
        _make_plan(state)
    goal = _PLAN.get(drive_unit_id)
    if goal is None or goal == unit.current_node:
        return _park_move(state, unit)
    return _advance(state, unit, goal)


def drive_unit_next_move(drive_unit_id: int, state: GraphState) -> Optional[int]:
    try:
        return _decide(drive_unit_id, state)
    except Exception:
        return None
