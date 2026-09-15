"""
Amazon Robotics Hackathon - Routing API (variant vC_rollout)

*****IMPORTANT*****
Team name: vibe coders
Email address: alexliu22111@gmail.com
*******************

Strategy: forward-simulation rollout search.
- Reimplements the engine step loop (spawn-free: future spawns invisible) in a
  tiny internal simulator with engine-identical validity/occupancy semantics.
- Each poll: enumerate {wait} + every currently-valid neighbor as my first
  move, roll each candidate out H steps with a greedy Dijkstra baseline policy
  driving ALL units, score = sum exp(-duration/50) over pods delivered in the
  rollout + a discounted potential for undelivered progress; pick argmax.
- Rollout naturally discovers bay-yields / retreats that greedy misses, since
  a first move that unblocks the corridor scores real deliveries downstream.
"""

import heapq
import math

_INF = float("inf")

H = 70          # rollout horizon in simulated steps
POT_W = 0.5     # weight of undelivered-progress potential term
MARGIN = 0.045  # rollout gain required to deviate from the greedy baseline move

# ---------------- per-graph cached structures ----------------
_G = {
    "sig": None,
    "edges": None,    # list of (a, b, weight, cap, bidi)
    "emap": None,     # (a,b) -> index of FIRST edge connecting (engine order)
    "adj": None,      # node -> list of (nbr, ceil_cost)
    "dist": None,     # src -> {dst: dist}
    "nodecap": None,  # node -> capacity (None = unlimited)
    "nodetype": None, # node -> type string
}


def _graph_sig(state):
    e = tuple((ed.from_node, ed.to_node, ed.weight, ed.capacity, ed.bidirectional)
              for ed in state.edges)
    n = tuple((nd.id, nd.capacity, nd.node_type) for nd in state.nodes)
    return hash((e, n))


def _refresh_graph(state):
    sig = _graph_sig(state)
    if sig == _G["sig"]:
        return
    edges = [(ed.from_node, ed.to_node, float(ed.weight), ed.capacity,
              bool(ed.bidirectional)) for ed in state.edges]
    emap = {}
    for i, (a, b, w, c, bi) in enumerate(edges):
        if (a, b) not in emap:
            emap[(a, b)] = i
        if bi and (b, a) not in emap:
            emap[(b, a)] = i
    adj = {nd.id: [] for nd in state.nodes}
    for (a, b, w, c, bi) in edges:
        cost = max(1, int(math.ceil(w)))
        adj.setdefault(a, []).append((b, cost))
        if bi:
            adj.setdefault(b, []).append((a, cost))
    dist = {}
    for src in adj:
        d = {src: 0}
        pq = [(0, src)]
        while pq:
            du, u = heapq.heappop(pq)
            if du > d.get(u, _INF):
                continue
            for v, c in adj[u]:
                nd = du + c
                if nd < d.get(v, _INF):
                    d[v] = nd
                    heapq.heappush(pq, (nd, v))
        dist[src] = d
    _G.update(sig=sig, edges=edges, emap=emap, adj=adj, dist=dist,
              nodecap={nd.id: nd.capacity for nd in state.nodes},
              nodetype={nd.id: nd.node_type for nd in state.nodes})


def _d(a, b):
    if a == b:
        return 0
    return _G["dist"].get(a, {}).get(b, _INF)


def _connects(e, a, b):
    if e[0] == a and e[1] == b:
        return True
    return e[4] and e[0] == b and e[1] == a


# ---------------- light simulator ----------------
# unit: [id, node, cap, carrying(list), in_transit, dest, rem]
# pod:  {"id","node","dest","entry","carried_by"}

def _build_sim(state):
    units = []
    for u in sorted(state.drive_units, key=lambda x: x.id):
        units.append([u.id, u.current_node, u.capacity, list(u.carrying),
                      bool(u.in_transit), u.transit_destination,
                      float(u.transit_remaining_time or 0)])
    pods = {}
    for p in state.active_pods:
        pods[p.id] = {"id": p.id, "node": p.current_node,
                      "dest": p.destination_station, "entry": p.entry_time,
                      "carried_by": p.carried_by}
    return {"t": state.current_time_step, "units": units, "pods": pods,
            "claims": {}, "score": 0.0}


def _clone(sim):
    return {"t": sim["t"],
            "units": [[u[0], u[1], u[2], list(u[3]), u[4], u[5], u[6]]
                      for u in sim["units"]],
            "pods": {pid: dict(p) for pid, p in sim["pods"].items()},
            "claims": dict(sim["claims"]),
            "score": sim["score"]}


def _move_ok(sim, u, nxt):
    ei = _G["emap"].get((u[1], nxt))
    if ei is None:
        return False
    e = _G["edges"][ei]
    if e[3] is not None:
        occ = 0
        for uu in sim["units"]:
            if uu[4] and _connects(e, uu[1], uu[5]):
                occ += 1
        if occ >= e[3]:
            return False
    ncap = _G["nodecap"].get(nxt)
    if ncap is not None:
        occ = 0
        for uu in sim["units"]:
            if uu[4]:
                if uu[5] == nxt:
                    occ += 1
            elif uu[1] == nxt:
                occ += 1
        if occ >= ncap:
            return False
    return True


def _dp(sim):
    """Deliveries then pickups, ascending unit id (mirrors engine)."""
    pods = sim["pods"]
    for u in sim["units"]:
        if u[4]:
            continue
        node = u[1]
        if u[3]:
            keep = []
            for pid in u[3]:
                p = pods.get(pid)
                if p is not None and p["dest"] == node:
                    sim["score"] += math.exp(-(sim["t"] - p["entry"]) / 50.0)
                    del pods[pid]
                else:
                    keep.append(pid)
            u[3] = keep
        if len(u[3]) < u[2]:
            waiting = [p for p in pods.values()
                       if p["carried_by"] is None and p["node"] == node]
            waiting.sort(key=lambda p: (p["entry"], p["id"]))
            for p in waiting:
                if len(u[3]) >= u[2]:
                    break
                p["carried_by"] = u[0]
                p["node"] = None
                u[3].append(p["id"])


def _pick_pod(sim, u):
    claimed = set()
    for uu in sim["units"]:
        claimed.update(uu[3])
    for uid, pid in sim["claims"].items():
        if uid != u[0]:
            claimed.add(pid)
    best = None
    best_key = None
    for p in sim["pods"].values():
        if p["carried_by"] is not None or p["node"] is None or p["id"] in claimed:
            continue
        dp = _d(u[1], p["node"])
        if dp == _INF or _d(p["node"], p["dest"]) == _INF:
            continue
        key = (dp, p["entry"], p["id"])
        if best_key is None or key < best_key:
            best_key = key
            best = p
    if best is not None:
        sim["claims"][u[0]] = best["id"]
    return best


def _target(sim, u):
    cands = []
    for pid in u[3]:
        p = sim["pods"].get(pid)
        if p is not None and _d(u[1], p["dest"]) < _INF:
            cands.append(p["dest"])
    if not u[3] and u[2] > 0:
        pid = sim["claims"].get(u[0])
        p = sim["pods"].get(pid) if pid is not None else None
        if p is None or p["carried_by"] is not None or p["node"] is None:
            sim["claims"].pop(u[0], None)
            p = _pick_pod(sim, u)
        if p is not None:
            cands.append(p["node"])
    if not cands:
        return None
    return min(cands, key=lambda n: _d(u[1], n))


def _baseline_move(sim, u):
    target = _target(sim, u)
    here = u[1]
    if target is None:
        # idle drift toward nearest storage node (pods spawn there)
        storages = [n for n, t in _G["nodetype"].items() if t == "storage"]
        if storages and here not in storages:
            tgt = min(storages, key=lambda s: _d(here, s))
            if _d(here, tgt) < _INF:
                hop = _best_hop(here, tgt)
                if (hop is not None and _move_ok(sim, u, hop)
                        and not (_G["nodetype"].get(hop) == "station"
                                 and _G["nodecap"].get(hop) is not None)):
                    return hop
        return None
    if target == here:
        return None
    hop = _best_hop(here, target)
    if hop is not None and _move_ok(sim, u, hop):
        return hop
    # blocked: no-retreat detour (strictly closer to target)
    here_d = _d(here, target)
    best = None
    best_k = None
    for nbr, c in _G["adj"].get(here, ()):
        if nbr == hop:
            continue
        dn = _d(nbr, target)
        if dn >= here_d:
            continue
        if not _move_ok(sim, u, nbr):
            continue
        k = (c + dn, nbr)
        if best_k is None or k < best_k:
            best_k = k
            best = nbr
    return best


def _best_hop(here, target):
    best = None
    best_k = None
    for nbr, c in _G["adj"].get(here, ()):
        dn = _d(nbr, target)
        if dn == _INF:
            continue
        k = (c + dn, nbr)
        if best_k is None or k < best_k:
            best_k = k
            best = nbr
    return best


def _step(sim, partial_uid=None, forced=None):
    """One engine step. partial_uid: mid-step entry — deliveries/pickups
    already done and units with id < partial_uid already acted this step."""
    for u in sim["units"]:
        if u[4]:
            continue
        if partial_uid is not None and u[0] < partial_uid:
            continue
        if partial_uid is not None and u[0] == partial_uid:
            mv = forced
        else:
            mv = _baseline_move(sim, u)
        if mv is not None and _move_ok(sim, u, mv):
            ei = _G["emap"][(u[1], mv)]
            u[4] = True
            u[5] = mv
            u[6] = _G["edges"][ei][2]
    for u in sim["units"]:
        if u[4]:
            u[6] -= 1.0
            if u[6] <= 0:
                u[1] = u[5]
                u[4] = False
                u[5] = None
                u[6] = 0.0
    _dp(sim)
    sim["t"] += 1


def _potential(sim):
    pot = 0.0
    units = sim["units"]
    for p in sim["pods"].values():
        elapsed = sim["t"] - p["entry"]
        if p["carried_by"] is not None:
            est = _INF
            for u in units:
                if p["id"] in u[3]:
                    pos = u[5] if u[4] else u[1]
                    est = (math.ceil(u[6]) if u[4] else 0) + _d(pos, p["dest"])
                    break
        else:
            n = p["node"]
            dpick = _INF
            for u in units:
                pos = u[5] if u[4] else u[1]
                du = (math.ceil(u[6]) if u[4] else 0) + _d(pos, n)
                if du < dpick:
                    dpick = du
            est = dpick + _d(n, p["dest"])
        if est == _INF:
            continue
        pot += math.exp(-(elapsed + est) / 50.0)
    return pot


def _rollout(sim0, my_uid, mv):
    sim = _clone(sim0)
    _step(sim, partial_uid=my_uid, forced=mv)
    for _ in range(H - 1):
        if not sim["pods"]:
            break
        _step(sim)
    return sim["score"] + POT_W * _potential(sim)


# ---------------- entry point ----------------
def drive_unit_next_move(drive_unit_id, state):
    try:
        return _decide(drive_unit_id, state)
    except Exception:
        return None


def _decide(my_uid, state):
    _refresh_graph(state)
    sim = _build_sim(state)
    me = None
    for u in sim["units"]:
        if u[0] == my_uid:
            me = u
            break
    if me is None or me[4]:
        return None

    cands = [None]
    for nbr, _c in _G["adj"].get(me[1], ()):
        if nbr not in cands and _move_ok(sim, me, nbr):
            cands.append(nbr)
    if len(cands) == 1:
        return None

    base_mv = _baseline_move(_clone(sim), me)

    best = None
    best_score = -_INF
    base_score = -_INF
    for mv in cands:
        s = _rollout(sim, my_uid, mv)
        if mv == base_mv:
            s += 1e-7
            base_score = s
        elif mv is None:
            s += 5e-8
        if s > best_score:
            best_score = s
            best = mv
    # Deviating from greedy is only trusted when the rollout gain is real:
    # baseline-continuation artifacts are small, deadlock breaks are large.
    if (best != base_mv and base_score > -_INF
            and best_score - base_score < MARGIN):
        return base_mv
    return best
