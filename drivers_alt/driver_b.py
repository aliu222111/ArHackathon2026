"""
Driver B - cooperative full-replan planner with conflict-aware first hops.

Team name: vibe coders
Email address: alexliu22111@gmail.com

Strategy: on the first poll of each time step, plan moves for ALL idle units
centrally (in unit-ID order, matching the engine's poll/commit order), using
Dijkstra distances plus a per-step reservation ledger for edge/node capacity.
Serve cached moves to later polls in the same step. Full replan every step.
"""
import math
import heapq

_M = {
    'sig': None,        # graph signature
    'adj': None,        # node -> [(nbr, steps)]
    'dist': None,       # src -> {dst: steps}
    'nhop': None,       # src -> {dst: first hop}
    'plan_time': None,  # time step the current plan is for
    'plan': {},         # unit_id -> next node (or None)
    'assign': {},       # unit_id -> pod_id (sticky claims)
    'stuck': {},        # unit_id -> consecutive blocked steps
}


def _graph_sig(state):
    nodes = tuple(sorted((n.id, repr(n.capacity)) for n in state.nodes))
    edges = tuple(sorted(
        (e.from_node, e.to_node, repr(e.weight), repr(e.capacity), e.bidirectional)
        for e in state.edges))
    units = tuple(sorted(u.id for u in state.drive_units))
    return (nodes, edges, units)


def _steps(w):
    return max(1, int(math.ceil(w)))


def _build_graph(state):
    adj = {n.id: [] for n in state.nodes}
    seen = set()
    for e in state.edges:
        pairs = [(e.from_node, e.to_node)]
        if e.bidirectional:
            pairs.append((e.to_node, e.from_node))
        for a, b in pairs:
            if (a, b) not in seen and a in adj:
                seen.add((a, b))
                adj[a].append((b, _steps(e.weight)))
    dist, nhop = {}, {}
    for src in adj:
        d = {src: 0}
        first = {}
        pq = [(0, src, None)]
        while pq:
            dd, node, f = heapq.heappop(pq)
            if dd > d.get(node, float('inf')):
                continue
            if node != src:
                first.setdefault(node, f)
            for nbr, w in adj[node]:
                nd = dd + w
                if nd < d.get(nbr, float('inf')):
                    d[nbr] = nd
                    heapq.heappush(pq, (nd, nbr, f if f is not None else nbr))
        dist[src] = d
        nhop[src] = first
    _M['adj'], _M['dist'], _M['nhop'] = adj, dist, nhop


def _edge_key(state, a, b):
    e = state.get_edge(a, b)
    if e is None:
        return None
    return (min(e.from_node, e.to_node), max(e.from_node, e.to_node)) if e.bidirectional \
        else (e.from_node, e.to_node)


def _hop_ok(state, a, b, ledger):
    """Can a unit start traversing a->b this step, given moves already planned?"""
    e = state.get_edge(a, b)
    if e is None:
        return False
    if e.capacity is not None:
        used = state.edge_occupancy(a, b) + ledger['edge'].get(_edge_key(state, a, b), 0)
        if used >= e.capacity:
            return False
    n = state.get_node(b)
    if n is not None and n.capacity is not None:
        occ = state.node_occupancy(b) + ledger['node'].get(b, 0)
        if occ >= n.capacity:
            return False
    return True


def _route(state, src, dst, ledger):
    """Dijkstra src->dst; first hop must be executable this step. Returns first hop or None."""
    if src == dst:
        return None
    adj = _M['adj']
    d = {src: 0}
    first = {}
    pq = [(0, src, None)]
    while pq:
        dd, node, f = heapq.heappop(pq)
        if dd > d.get(node, float('inf')):
            continue
        if node == dst:
            return f
        for nbr, w in adj.get(node, ()):
            if node == src and not _hop_ok(state, src, nbr, ledger):
                continue
            nd = dd + w
            if nd < d.get(nbr, float('inf')):
                d[nbr] = nd
                heapq.heappush(pq, (nd, nbr, f if f is not None else nbr))
    return None


def _plan_step(state):
    dist = _M['dist']
    units = sorted(state.drive_units, key=lambda u: u.id)
    unit_by_id = {u.id: u for u in units}

    # --- fresh matching every step; old claims only break ties ---
    pods = {p.id: p for p in state.active_pods}
    old_assign = dict(_M['assign'])
    assign = {}
    _M['assign'] = assign
    free_pods = [p for p in state.active_pods
                 if p.carried_by is None and p.current_node is not None]

    # --- effective position (in-transit units claim from their destination) ---
    def eff(u):
        if u.in_transit:
            return u.transit_destination, max(0, u.transit_remaining_time)
        return u.current_node, 0

    # --- greedy matching: empty units (idle or in transit) to free pods ---
    cand = [u for u in units if not u.carrying]
    pairs = []
    for u in cand:
        pos, lag = eff(u)
        for p in free_pods:
            dd = dist.get(pos, {}).get(p.current_node)
            if dd is None:
                continue
            sticky = 0 if old_assign.get(u.id) == p.id else 1
            pairs.append((lag + dd, sticky, p.entry_time, u.id, p.id))
    pairs.sort()
    used_u, used_p = set(), set()
    for cost, _s, _e, uid, pid in pairs:
        if uid in used_u or pid in used_p:
            continue
        used_u.add(uid)
        used_p.add(pid)
        assign[uid] = pid

    # --- opportunistic extra pods for carrying units with spare capacity ---
    claimed = set(assign.values())
    for u in units:
        if u.carrying and u.has_capacity and u.id not in assign:
            pos, lag = eff(u)
            best = None
            dests = [pods[c].destination_station if pods.get(c) else state.get_pod(c).destination_station
                     for c in u.carrying if state.get_pod(c) is not None]
            base = min((dist.get(pos, {}).get(t, float('inf')) for t in dests), default=float('inf'))
            for p in state.active_pods:
                if p.carried_by is not None or p.id in claimed or p.current_node is None:
                    continue
                d_to_pod = dist.get(pos, {}).get(p.current_node)
                if d_to_pod is None:
                    continue
                detour = d_to_pod + min(
                    (dist.get(p.current_node, {}).get(t, float('inf')) for t in dests + [p.destination_station]),
                    default=float('inf'))
                if detour <= base + 3 and (best is None or d_to_pod < best[0]):
                    best = (d_to_pod, p.id)
            if best is not None:
                assign[u.id] = best[1]
                claimed.add(best[1])

    # --- choose targets & plan moves in unit-ID order ---
    ledger = {'edge': {}, 'node': {}}
    plan = {}
    for u in units:
        if u.in_transit:
            continue
        target = None
        if u.carrying:
            # nearest carried destination; an assigned extra pod competes too
            options = []
            for pid in u.carrying:
                p = state.get_pod(pid)
                if p is not None:
                    options.append(p.destination_station)
            pid = assign.get(u.id)
            if pid and u.has_capacity and pid in pods and pods[pid].current_node is not None:
                options.append(pods[pid].current_node)
            if options:
                target = min(options,
                             key=lambda t: dist.get(u.current_node, {}).get(t, float('inf')))
        else:
            pid = assign.get(u.id)
            if pid and pid in pods and pods[pid].current_node is not None:
                target = pods[pid].current_node

        move = None
        if target is not None and target != u.current_node:
            move = _route(state, u.current_node, target, ledger)
            if move is None:
                _M['stuck'][u.id] = _M['stuck'].get(u.id, 0) + 1
                if _M['stuck'][u.id] >= 3:
                    # break deadlock: step to any open neighbor
                    for nbr, _w in _M['adj'].get(u.current_node, ()):
                        if _hop_ok(state, u.current_node, nbr, ledger):
                            move = nbr
                            break
            else:
                _M['stuck'][u.id] = 0
        else:
            _M['stuck'][u.id] = 0
            # idle with no task: clear finite-capacity stations
            node = state.get_node(u.current_node)
            if node is not None and node.node_type == 'station' and node.capacity is not None:
                best = None
                for nbr, _w in _M['adj'].get(u.current_node, ()):
                    if _hop_ok(state, u.current_node, nbr, ledger):
                        nn = state.get_node(nbr)
                        rank = (nn.node_type == 'station', state.node_occupancy(nbr))
                        if best is None or rank < best[0]:
                            best = (rank, nbr)
                if best is not None:
                    move = best[1]

        if move is not None:
            ek = _edge_key(state, u.current_node, move)
            if ek is not None:
                ledger['edge'][ek] = ledger['edge'].get(ek, 0) + 1
            ledger['node'][move] = ledger['node'].get(move, 0) + 1
        plan[u.id] = move
    _M['plan'] = plan
    _M['plan_time'] = state.current_time_step


def drive_unit_next_move(drive_unit_id, state):
    try:
        sig = _graph_sig(state)
        if sig != _M['sig'] or (_M['plan_time'] is not None
                                and state.current_time_step < _M['plan_time']):
            _M.update({'sig': sig, 'plan_time': None, 'plan': {},
                       'assign': {}, 'stuck': {}})
            _build_graph(state)
        if _M['plan_time'] != state.current_time_step:
            _plan_step(state)
        move = _M['plan'].get(drive_unit_id)
        if move is None:
            return None
        u = state.get_drive_unit(drive_unit_id)
        if u is None or u.in_transit:
            return None
        e = state.get_edge(u.current_node, move)
        if e is None:
            return None
        if e.capacity is not None and state.edge_occupancy(u.current_node, move) >= e.capacity:
            return None
        n = state.get_node(move)
        if n is not None and n.capacity is not None and state.node_occupancy(move) >= n.capacity:
            return None
        return move
    except Exception:
        return None
