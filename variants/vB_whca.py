"""
Variant B - true space-time cooperative A* (WHCA*, Silver 2005).

Team name: vibe coders
Email address: alexliu22111@gmail.com

On the first poll of each time step, plan ALL units centrally in ascending
unit-ID order (matching the engine's poll/commit order) with space-time A*
(wait actions included) against a reservation table of (node, t) and
(edge, t) occupancy COUNTS seeded with in-transit units' committed
trajectories.  Each planned unit's full trajectory (pickup leg + delivery
leg + step-off-the-dock hop) is written into the table so lower-priority
units route around it in time, waiting in bays when a corridor is claimed.
Full replan every step (RHCR discipline); cached first moves served to
later polls in the same step, re-validated against the live state.

Reservation semantics mirror the referee exactly (routing_utils.py):
- a move committed at t on an edge of w steps occupies the edge [t, t+w-1]
  and counts toward the destination node's occupancy [t, t+w-1];
- the unit then stands on the node [t+w, depart-1]: a later-planned
  (higher-ID, polled-after) unit may enter at exactly its departure step;
- a unit standing at n at routing time t blocks a lower-ID arrival
  committing at t, so A* forbids occupying a cell any earlier-planned
  unit has reserved (capacity counted, not binary).
"""
import heapq
import itertools
import math

WINDOW = 120        # planning horizon (steps)
MAXEXP = 40000      # A* expansion guard

_S = {
    'sig': None,
    'adj': None,      # node -> [(nbr, w_steps, edge_idx, edge_cap)]
    'dist': None,     # src -> {dst: steps} free-flow all-pairs
    'ncap': None,     # node -> capacity (None = unlimited)
    'ntype': None,    # node -> node_type
    'plan': {},       # unit_id -> next node (or None) for plan_time
    'plan_time': None,
    'assign': {},     # unit_id -> pod_id sticky claims
}


def _steps(w):
    return max(1, int(math.ceil(w)))


def _graph_sig(state):
    nodes = tuple(sorted((n.id, repr(n.capacity)) for n in state.nodes))
    edges = tuple(sorted(
        (e.from_node, e.to_node, repr(e.weight), repr(e.capacity), e.bidirectional)
        for e in state.edges))
    units = tuple(sorted(u.id for u in state.drive_units))
    return (nodes, edges, units)


def _build_graph(state):
    adj = {n.id: [] for n in state.nodes}
    for idx, e in enumerate(state.edges):
        w = _steps(e.weight)
        if e.from_node in adj:
            adj[e.from_node].append((e.to_node, w, idx, e.capacity))
        if e.bidirectional and e.to_node in adj:
            adj[e.to_node].append((e.from_node, w, idx, e.capacity))
    dist = {}
    for src in adj:
        d = {src: 0}
        pq = [(0, src)]
        while pq:
            dd, node = heapq.heappop(pq)
            if dd > d.get(node, float('inf')):
                continue
            for nbr, w, _i, _c in adj[node]:
                nd = dd + w
                if nd < d.get(nbr, float('inf')):
                    d[nbr] = nd
                    heapq.heappush(pq, (nd, nbr))
        dist[src] = d
    _S['adj'] = adj
    _S['dist'] = dist
    _S['ncap'] = {n.id: n.capacity for n in state.nodes}
    _S['ntype'] = {n.id: n.node_type for n in state.nodes}


# ---------------- reservation table ----------------

class _Res:
    def __init__(self):
        self.n = {}   # (node, t) -> count
        self.e = {}   # (edge_idx, t) -> count

    def rn(self, node, a, b):
        if _S['ncap'].get(node) is None:
            return
        for s in range(max(0, a), min(b, WINDOW) + 1):
            self.n[(node, s)] = self.n.get((node, s), 0) + 1

    def un(self, node, t):
        k = (node, t)
        if k in self.n:
            self.n[k] -= 1
            if self.n[k] <= 0:
                del self.n[k]

    def re(self, idx, cap, a, b):
        if cap is None:
            return
        for s in range(max(0, a), min(b, WINDOW) + 1):
            self.e[(idx, s)] = self.e.get((idx, s), 0) + 1

    def cn(self, node, s):
        return self.n.get((node, s), 0)

    def ce(self, idx, s):
        return self.e.get((idx, s), 0)


def _stand_ok(res, node, t):
    cap = _S['ncap'].get(node)
    return cap is None or res.cn(node, t) < cap


def _astar(res, src, t0, goal):
    """Space-time A* with waits. Returns (moves, arrival_t) or None.
    moves = list of (commit_t, from, to, w, edge_idx, edge_cap)."""
    dist = _S['dist']
    hmap = {n: dist[n].get(goal) for n in _S['adj']}
    if hmap.get(src) is None:
        return None
    if src == goal:
        return ([], t0)
    horizon = t0 + WINDOW
    came = {(src, t0): None}
    pq = [(t0 + hmap[src], t0, src)]
    exp = 0
    while pq and exp < MAXEXP:
        f, t, n = heapq.heappop(pq)
        if n == goal:
            moves = []
            cur = (n, t)
            while came[cur] is not None:
                prev, mv = came[cur]
                if mv is not None:
                    moves.append(mv)
                cur = prev
            moves.reverse()
            return (moves, t)
        if t >= horizon:
            continue
        exp += 1
        # wait
        ns = (n, t + 1)
        if ns not in came and _stand_ok(res, n, t + 1):
            came[ns] = ((n, t), None)
            heapq.heappush(pq, (t + 1 + hmap[n], t + 1, n))
        # moves
        for nbr, w, idx, ecap in _S['adj'].get(n, ()):
            if hmap.get(nbr) is None:
                continue
            ta = t + w
            ns = (nbr, ta)
            if ns in came:
                continue
            ok = True
            if ecap is not None:
                for s in range(t, ta):
                    if res.ce(idx, s) >= ecap:
                        ok = False
                        break
            ncap = _S['ncap'].get(nbr)
            if ok and ncap is not None:
                for s in range(t, ta + 1):   # transit-toward [t,ta-1] + arrival stand [ta]
                    if res.cn(nbr, s) >= ncap:
                        ok = False
                        break
            if ok:
                came[ns] = ((n, t), (t, n, nbr, w, idx, ecap))
                heapq.heappush(pq, (ta + hmap[nbr], ta, nbr))
    return None


def _record(res, n0, t0, moves):
    """Write a trajectory into the reservation table."""
    cur, ct = n0, t0
    for (tc, a, b, w, idx, ecap) in moves:
        if tc > ct:
            res.rn(cur, ct, tc - 1)          # standing before departure
        res.re(idx, ecap, tc, tc + w - 1)    # on the edge
        res.rn(b, tc, tc + w - 1)            # in transit toward b
        cur, ct = b, tc + w
    res.rn(cur, ct, WINDOW)                  # final stand


# ---------------- per-step central planning ----------------

def _eff(u):
    if u.in_transit:
        return u.transit_destination, max(1, u.transit_remaining_time)
    return u.current_node, 0


def _match(state, units):
    """Exhaustive score-gain assignment of empty units to free pods.

    Objective: maximize sum of e^(-(age + eta_pickup + haul)/50) over the
    assignment (the actual scoring curve), with a small stickiness bonus.
    Instances are tiny (<= ~5 units, <= ~8 pods) so enumeration is trivial.
    """
    dist = _S['dist']
    t_now = state.current_time_step
    old = dict(_S['assign'])
    free_pods = [p for p in state.active_pods
                 if p.carried_by is None and p.current_node is not None]
    cand = [u for u in units if not u.carrying]
    if not cand or not free_pods:
        _S['assign'] = {}
        return {}
    # value[u][p]
    val = {}
    for u in cand:
        pos, lag = _eff(u)
        row = {}
        for p in free_pods:
            d1 = dist.get(pos, {}).get(p.current_node)
            d2 = dist.get(p.current_node, {}).get(p.destination_station)
            if d1 is None or d2 is None:
                continue
            age = t_now - p.entry_time
            v = math.exp(-(age + lag + d1 + d2) / 50.0)
            if old.get(u.id) == p.id:
                v += 0.001
            row[p.id] = v
        val[u.id] = row
    k = min(len(cand), len(free_pods))
    best_v, best_asgn = -1.0, {}
    pod_ids = [p.id for p in free_pods]
    n_combos = (math.perm(len(pod_ids), k) * math.comb(len(cand), k)
                if hasattr(math, 'perm') else 10 ** 9)
    if n_combos <= 200000:
        for combo in itertools.permutations(pod_ids, k):
            for usub in itertools.combinations(cand, k):
                tot, asgn, ok = 0.0, {}, True
                for u, pid in zip(usub, combo):
                    v = val[u.id].get(pid)
                    if v is None:
                        ok = False
                        break
                    tot += v
                    asgn[u.id] = pid
                if ok and tot > best_v:
                    best_v, best_asgn = tot, asgn
    if not best_asgn:
        # fallback: greedy
        pairs = []
        for u in cand:
            for pid, v in val.get(u.id, {}).items():
                pairs.append((-v, u.id, pid))
        pairs.sort()
        used_u, used_p = set(), set()
        for _v, uid, pid in pairs:
            if uid in used_u or pid in used_p:
                continue
            used_u.add(uid)
            used_p.add(pid)
            best_asgn[uid] = pid
    _S['assign'] = best_asgn
    return best_asgn


def _goals(state, u, assign, pods, pos):
    """Ordered list of leg goals for this unit."""
    dist = _S['dist']
    if u.carrying:
        dests = []
        for pid in u.carrying:
            p = state.get_pod(pid)
            if p is not None:
                dests.append(p.destination_station)
        if dests:
            g = min(dests, key=lambda d: dist.get(pos, {}).get(d, float('inf')))
            if dist.get(pos, {}).get(g) is not None:
                return [g]
        return []
    pid = assign.get(u.id)
    if pid is not None and pid in pods:
        p = pods[pid]
        return [p.current_node, p.destination_station]
    # idle: clear finite-capacity nodes, drift toward a storage node
    here_cap = _S['ncap'].get(pos)
    storages = [n for n, ty in _S['ntype'].items() if ty == 'storage'
                and _S['ncap'].get(n) is None]
    reach = [s for s in storages if dist.get(pos, {}).get(s) is not None]
    if reach:
        g = min(reach, key=lambda s: dist[pos][s])
        if g != pos:
            return [g]
        return []
    if here_cap is not None:
        opts = [n for n in _S['adj'] if _S['ncap'].get(n) is None
                and dist.get(pos, {}).get(n) is not None]
        if opts:
            return [min(opts, key=lambda n: dist[pos][n])]
    return []


def _plan_all(state):
    units = sorted(state.drive_units, key=lambda u: u.id)
    pods = {p.id: p for p in state.active_pods}
    assign = _match(state, units)
    res = _Res()

    # seed: idle units occupy their node at t=0 until planned
    for u in units:
        if not u.in_transit:
            res.rn(u.current_node, 0, 0)
    # seed: committed in-transit trajectories
    for u in units:
        if u.in_transit:
            r = max(1, u.transit_remaining_time)
            for nbr, w, idx, ecap in _S['adj'].get(u.current_node, ()):
                if nbr == u.transit_destination:
                    res.re(idx, ecap, 0, r - 1)
                    break
            res.rn(u.transit_destination, 0, r - 1)

    plan = {}
    for u in units:
        pos, t0 = _eff(u)
        if not u.in_transit and _S['ncap'].get(pos) is not None:
            res.un(pos, 0)   # replace the seed with the real trajectory
        goals = _goals(state, u, assign, pods, pos)
        itinerary = []
        cur, ct = pos, t0
        ok = True
        for g in goals:
            if g == cur:
                continue
            got = _astar(res, cur, ct, g)
            if got is None:
                ok = False
                break
            moves, ta = got
            itinerary.extend(moves)
            cur, ct = g, ta
        # step off a finite-capacity dock after the last arrival
        if ok and itinerary and _S['ncap'].get(cur) is not None:
            best = None
            for dwell in range(0, 4):
                td = ct + dwell
                for nbr, w, idx, ecap in _S['adj'].get(cur, ()):
                    ncap = _S['ncap'].get(nbr)
                    bad = False
                    if ecap is not None:
                        for s in range(td, td + w):
                            if res.ce(idx, s) >= ecap:
                                bad = True
                                break
                    if not bad and ncap is not None:
                        for s in range(td, td + w + 1):
                            if res.cn(nbr, s) >= ncap:
                                bad = True
                                break
                    if not bad:
                        pref = (0 if ncap is None else 1,
                                0 if _S['ntype'].get(nbr) != 'station' else 1)
                        if best is None or pref < best[0]:
                            best = (pref, (td, cur, nbr, w, idx, ecap))
                if best is not None:
                    break
            if best is not None:
                itinerary.append(best[1])
        if not ok:
            itinerary = []
        _record(res, pos, t0, itinerary)
        if not u.in_transit:
            mv = None
            if itinerary and itinerary[0][0] == 0:
                mv = itinerary[0][2]
            plan[u.id] = mv
    _S['plan'] = plan
    _S['plan_time'] = state.current_time_step


# ---------------- entry point ----------------

def drive_unit_next_move(drive_unit_id, state):
    try:
        sig = _graph_sig(state)
        if sig != _S['sig'] or (_S['plan_time'] is not None
                                and state.current_time_step < _S['plan_time']):
            _S.update({'sig': sig, 'plan': {}, 'plan_time': None, 'assign': {}})
            _build_graph(state)
        if _S['plan_time'] != state.current_time_step:
            _plan_all(state)
        move = _S['plan'].get(drive_unit_id)
        if move is None:
            return None
        u = state.get_drive_unit(drive_unit_id)
        if u is None or u.in_transit:
            return None
        e = state.get_edge(u.current_node, move)
        if e is None:
            return None
        if e.capacity is not None and \
                state.edge_occupancy(u.current_node, move) >= e.capacity:
            return None
        n = state.get_node(move)
        if n is not None and n.capacity is not None and \
                state.node_occupancy(move) >= n.capacity:
            return None
        return int(move)
    except Exception:
        return None
