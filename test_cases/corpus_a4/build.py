#!/usr/bin/env python3
"""
Builds the corpus_a4 prediction of the hidden evaluation set.

Every case is DIDACTIC in the style of the six provided practice cases: one
specific naive strategy is punished, and metadata.description names it.
Run:  python3 build.py
"""

import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))

CASES = {}


def case(name, desc, max_t, nodes, edges, units, pods):
    """nodes: list of (id, name, type, capacity|None)
       edges: list of (a, b, w) | (a, b, w, cap) | (a, b, w, cap, bidir)
       units: list of (id, start) | (id, start, capacity)
       pods:  list of (id, src, dest, entry)"""
    n = []
    for tup in nodes:
        nid, nm, ntype = tup[0], tup[1], tup[2]
        d = {"id": nid, "name": nm, "type": ntype}
        if len(tup) > 3 and tup[3] is not None:
            d["capacity"] = tup[3]
        n.append(d)
    e = []
    for tup in edges:
        d = {"from_node": tup[0], "to_node": tup[1], "weight": tup[2]}
        if len(tup) > 3 and tup[3] is not None:
            d["capacity"] = tup[3]
        if len(tup) > 4 and tup[4] is not None:
            d["bidirectional"] = tup[4]
        e.append(d)
    u = []
    for tup in units:
        d = {"id": tup[0], "start_node": tup[1]}
        if len(tup) > 2 and tup[2] is not None:
            d["capacity"] = tup[2]
        u.append(d)
    p = [{"id": t[0], "source_node": t[1], "destination_station": t[2], "entry_time": t[3]}
         for t in pods]
    CASES[name] = {
        "metadata": {"max_time_steps": max_t, "description": desc},
        "nodes": n, "edges": e, "drive_units": u, "pods": p,
    }


ST = "storage"
SN = "station"
TR = "travel"

# =====================================================================
# LEVEL 1 - single drive unit, shortest-path / sequencing lessons
# =====================================================================

case(
    "l1_01_ceil_vs_raw_weight",
    "Level 1: single unit, fractional aisle weights. The five-hop rack row sums to "
    "6.0 but the engine decrements transit by 1 per step, so each 1.2 aisle really "
    "costs 2 and the row costs 10; the single 7.0 cross-aisle costs 7. Punishes a "
    "Dijkstra that sums raw weights instead of ceil(weight) per edge.",
    90,
    [(0, "Storage-A", ST), (1, "Aisle-1", TR), (2, "Aisle-2", TR), (3, "Aisle-3", TR),
     (4, "Aisle-4", TR), (5, "Station-1", SN)],
    [(0, 1, 1.2), (1, 2, 1.2), (2, 3, 1.2), (3, 4, 1.2), (4, 5, 1.2), (0, 5, 7)],
    [(0, 0)],
    [("P1", 0, 5, 0), ("P2", 0, 5, 14), ("P3", 0, 5, 28), ("P4", 0, 5, 42)],
)

case(
    "l1_02_spine_rib_sequencing",
    "Level 1: single unit on a spine-and-rib floor (one main spine, four rack ribs, "
    "two stations at opposite ends). Five pods arrive over time. Punishes always "
    "chasing the nearest pod: the correct order is the one that minimises total "
    "exponential decay, which sometimes means passing a near pod to clear a far one "
    "that is already ageing.",
    170,
    [(0, "Station-W", SN), (1, "Spine-1", TR), (2, "Spine-2", TR), (3, "Spine-3", TR),
     (4, "Spine-4", TR), (5, "Spine-5", TR), (6, "Storage-R1", ST), (7, "Storage-R2", ST),
     (8, "Storage-R3", ST), (9, "Storage-R4", ST), (10, "Station-E", SN),
     (11, "Cross-Aisle", TR), (12, "Storage-Far", ST)],
    [(1, 2, 3), (2, 3, 3), (3, 4, 3), (4, 5, 3), (6, 1, 2), (7, 2, 2), (8, 3, 2),
     (9, 4, 2), (0, 1, 2), (10, 5, 2), (2, 11, 4), (11, 4, 4), (5, 12, 3)],
    [(0, 1)],
    [("P1", 6, 0, 0), ("P2", 9, 10, 0), ("P3", 7, 10, 6), ("P4", 8, 0, 14),
     ("P5", 12, 0, 26)],
)

case(
    "l1_03_oneway_loop_return",
    "Level 1: single unit on a one-way perimeter loop with two one-way cross aisles "
    "(every edge bidirectional:false). Outbound to Station-1 is 6, the return is 10. "
    "Punishes any planner that builds an undirected adjacency from the edge list: it "
    "will emit moves against the traffic direction, which the referee silently "
    "discards, and the unit waits forever.",
    170,
    [(0, "Storage-A", ST), (1, "Aisle-N1", TR), (2, "Aisle-N2", TR), (3, "Station-1", SN),
     (4, "Aisle-E", TR), (5, "Aisle-S1", TR), (6, "Storage-B", ST), (7, "Aisle-W", TR),
     (8, "Station-2", SN), (9, "Cross-Aisle", TR)],
    [(0, 1, 2, None, False), (1, 2, 2, None, False), (2, 3, 2, None, False),
     (3, 4, 2, None, False), (4, 5, 2, None, False), (5, 6, 2, None, False),
     (6, 7, 2, None, False), (7, 0, 2, None, False), (1, 9, 3, None, False),
     (9, 5, 3, None, False), (5, 8, 2, None, False), (8, 6, 2, None, False)],
    [(0, 0)],
    [("P1", 0, 3, 0), ("P2", 6, 8, 0), ("P3", 0, 3, 18), ("P4", 6, 3, 30)],
)

case(
    "l1_04_pod_born_at_station",
    "Level 1: a returned pod spawns on the very dock it belongs to "
    "(source_node == destination_station) with the unit already standing there. "
    "Waiting zero steps delivers it at duration 0 for a full 100 points. Punishes "
    "drivers whose idle/parking rule fires whenever the goal equals the current node: "
    "they drive the unit off the dock carrying the pod and have to come back.",
    130,
    [(0, "Station-1", SN), (1, "Approach", TR), (2, "Storage-A", ST), (3, "Aisle-1", TR),
     (4, "Station-2", SN), (5, "Storage-B", ST)],
    [(0, 1, 2), (1, 2, 6), (2, 3, 2), (3, 4, 3), (5, 1, 4), (5, 3, 4)],
    [(0, 0)],
    [("R1", 0, 0, 0), ("P1", 2, 0, 0), ("R2", 4, 4, 14), ("P2", 5, 4, 20),
     ("P3", 2, 4, 34)],
)

case(
    "l1_05_storage_decoy_parking",
    "Level 1: every pod spawns at a decant node typed 'travel'; the only node typed "
    "'storage' is an overflow rack at the end of a 15-step dead-end spur that never "
    "holds a pod. Punishes the common idle heuristic 'drift to the nearest storage "
    "node' - it walks the unit into the spur between deliveries and it is never home "
    "when the next pod lands.",
    150,
    [(0, "Decant-1", TR), (1, "Aisle-1", TR), (2, "Aisle-2", TR), (3, "Station-1", SN),
     (4, "Spur-1", TR), (5, "Storage-Overflow", ST), (6, "Decant-2", TR),
     (7, "Aisle-3", TR), (8, "Spur-2", TR)],
    [(0, 1, 2), (1, 2, 2), (2, 3, 2), (1, 4, 5), (4, 8, 5), (8, 5, 5), (6, 7, 2),
     (7, 1, 2)],
    [(0, 0)],
    [("P1", 0, 3, 0), ("P2", 6, 3, 8), ("P3", 0, 3, 16), ("P4", 6, 3, 24),
     ("P5", 0, 3, 32), ("P6", 6, 3, 40)],
)

case(
    "l1_06_prestage_right_storage",
    "Level 1: two storage areas feed two stations, but 4 of 5 pods come from "
    "Storage-A, which sits one weight-1 hop from its station. Punishes parking at "
    "whichever storage node happens to be nearest after the last delivery: idling at "
    "Storage-B leaves an 8-step trek when the late Storage-A pods land, while "
    "pre-staging at Storage-A delivers them at duration 1.",
    60,
    [(0, "Storage-A", ST), (1, "Aisle-1", TR), (2, "Station-1", SN), (3, "Storage-B", ST),
     (4, "Aisle-2", TR), (5, "Station-2", SN), (6, "Aisle-3", TR)],
    [(0, 1, 1), (1, 2, 1), (0, 3, 6), (3, 4, 1), (4, 5, 1), (1, 6, 3), (6, 4, 3)],
    [(0, 0)],
    [("P1", 0, 2, 0), ("P2", 0, 2, 8), ("P3", 3, 5, 18), ("P4", 0, 2, 40),
     ("P5", 0, 2, 44)],
)

case(
    "l1_07_grid5x3_tour",
    "Level 1: a single unit on a realistic 15-node 5x3 floor grid with storage at "
    "three corners and stations at two. Five pods over time, none co-located. "
    "Punishes myopic next-hop greed at a size where several equal-length routes exist: "
    "only true weighted shortest path plus a sensible pod order keeps the average "
    "delivery duration down.",
    190,
    [(0, "Storage-NW", ST), (1, "Aisle-N1", TR), (2, "Storage-N", ST), (3, "Aisle-N2", TR),
     (4, "Station-NE", SN), (5, "Aisle-W1", TR), (6, "Aisle-C1", TR), (7, "Center", TR),
     (8, "Aisle-C2", TR), (9, "Aisle-E1", TR), (10, "Storage-SW", ST),
     (11, "Aisle-S1", TR), (12, "Storage-S", ST), (13, "Aisle-S2", TR),
     (14, "Station-SE", SN)],
    [(a, b, 2) for a, b in
     [(0, 1), (1, 2), (2, 3), (3, 4), (5, 6), (6, 7), (7, 8), (8, 9),
      (10, 11), (11, 12), (12, 13), (13, 14), (0, 5), (5, 10), (1, 6), (6, 11),
      (2, 7), (7, 12), (3, 8), (8, 13), (4, 9), (9, 14)]],
    [(0, 7)],
    [("P1", 0, 14, 0), ("P2", 10, 4, 0), ("P3", 2, 14, 12), ("P4", 12, 4, 26),
     ("P5", 0, 4, 40)],
)

case(
    "l1_08_figure_eight_alternating",
    "Level 1: two rack loops joined at a hub, with a station hanging off each loop. "
    "Six pods alternate between the loops and two of them cross to the far station. "
    "Punishes FIFO service: taking the pods in arrival order forces four hub crossings, "
    "while grouping by loop halves the travel.",
    190,
    [(0, "Hub", TR), (1, "W-North", TR), (2, "Storage-W", ST), (3, "W-South", TR),
     (4, "Station-W", SN), (5, "E-North", TR), (6, "Storage-E", ST), (7, "E-South", TR),
     (8, "Station-E", SN), (9, "W-Link", TR), (10, "E-Link", TR), (11, "Cross-Aisle", TR)],
    [(0, 1, 2), (1, 2, 2), (2, 9, 2), (9, 3, 2), (3, 0, 2), (3, 4, 2), (0, 5, 2),
     (5, 6, 2), (6, 10, 2), (10, 7, 2), (7, 0, 2), (7, 8, 2), (2, 11, 7), (11, 6, 7)],
    [(0, 0)],
    [("P1", 2, 4, 0), ("P2", 6, 8, 0), ("P3", 2, 8, 8), ("P4", 6, 4, 16),
     ("P5", 2, 4, 26), ("P6", 6, 8, 34)],
)

# =====================================================================
# LEVEL 2 - several units, narrow aisles, assignment
# =====================================================================

case(
    "l2_01_passing_bay_deadlock",
    "Level 2: a single-lane cross aisle whose three waypoints each have capacity 1, "
    "with one passing bay hanging off the middle waypoint. Two units must swap ends. "
    "Punishes any router that only ever takes hops which strictly reduce distance to "
    "goal: the two units meet nose to nose on the capacity-1 waypoints and both wait "
    "forever. The win is to pull one unit into the bay - a move that temporarily "
    "increases its distance to goal - and let the other pass.",
    220,
    [(0, "Storage-W", ST), (1, "Aisle-W", TR, 1), (2, "Bay-Junction", TR, 1),
     (3, "Aisle-E", TR, 1), (4, "Storage-E", ST), (5, "Passing-Bay", TR, 1),
     (6, "Station-W", SN), (7, "Station-E", SN)],
    [(0, 1, 2), (1, 2, 2), (2, 3, 2), (3, 4, 2), (2, 5, 2), (0, 6, 2), (4, 7, 2)],
    [(0, 0), (1, 4)],
    [("P1", 4, 6, 0), ("P2", 0, 7, 0), ("P3", 4, 6, 24), ("P4", 0, 7, 24)],
)

case(
    "l2_02_single_lane_yield",
    "Level 2: the same single-lane cross aisle with NO passing bay - three capacity-1 "
    "waypoints between a west and an east storage area. The only way to resolve a "
    "head-on meeting is for one unit to reverse out of the aisle entirely. Punishes "
    "'never retreat' routers: they gridlock on the middle waypoints and deliver zero. "
    "Yielding costs one unit four steps and saves the whole case.",
    220,
    [(0, "Storage-W", ST), (1, "Aisle-1", TR, 1), (2, "Aisle-2", TR, 1),
     (3, "Aisle-3", TR, 1), (4, "Storage-E", ST), (5, "Station-W", SN),
     (6, "Station-E", SN), (7, "Yard-W", TR)],
    [(0, 1, 2), (1, 2, 2), (2, 3, 2), (3, 4, 2), (0, 5, 2), (4, 6, 2), (0, 7, 2)],
    [(0, 0), (1, 4)],
    [("P1", 4, 5, 0), ("P2", 0, 6, 0), ("P3", 4, 5, 30), ("P4", 0, 6, 30)],
)

case(
    "l2_03_two_bridges_split",
    "Level 2: two capacity-1 bridges span the floor, a short one (weight 5) and a long "
    "one (weight 8). Four units and six pods all need the east side. Punishes routing "
    "every unit down the cheapest bridge: it serialises four crossings behind one "
    "aisle. Splitting the fleet across both bridges finishes far sooner even though "
    "half the fleet takes a longer path.",
    200,
    [(0, "Storage-W", ST), (1, "Junction-W", TR), (2, "Bridge-N-W", TR),
     (3, "Bridge-N-E", TR), (4, "Bridge-S-W", TR), (5, "Bridge-S-E", TR),
     (6, "Junction-E", TR), (7, "Station-1", SN), (8, "Station-2", SN),
     (9, "Storage-W2", ST), (10, "Park-W", TR)],
    [(0, 1, 2), (9, 1, 2), (1, 2, 1), (2, 3, 5, 1), (3, 6, 1), (1, 4, 1),
     (4, 5, 8, 1), (5, 6, 1), (6, 7, 2), (6, 8, 2), (1, 10, 2)],
    [(0, 0), (1, 0), (2, 9), (3, 9)],
    [("P1", 0, 7, 0), ("P2", 0, 8, 0), ("P3", 9, 7, 0), ("P4", 9, 8, 2),
     ("P5", 0, 7, 10), ("P6", 9, 8, 12)],
)

case(
    "l2_04_grid4x4_cap1",
    "Level 2: a 16-node 4x4 floor grid where every aisle has capacity 1, four units and "
    "eight pods running diagonally between two storage corners and two station corners. "
    "The scaled-up version of the provided 3x3 case. Punishes shortest-path-for-everyone: "
    "all four units funnel through the same two central aisles and block each other; "
    "spreading routes over the grid's parallel paths costs a couple of steps each and "
    "removes the jam.",
    220,
    [(0, "Storage-NW", ST), (1, "Aisle-N1", TR), (2, "Aisle-N2", TR), (3, "Station-NE", SN),
     (4, "Aisle-W1", TR), (5, "Center-NW", TR), (6, "Center-NE", TR), (7, "Aisle-E1", TR),
     (8, "Aisle-W2", TR), (9, "Center-SW", TR), (10, "Center-SE", TR), (11, "Aisle-E2", TR),
     (12, "Storage-SW", ST), (13, "Aisle-S1", TR), (14, "Aisle-S2", TR),
     (15, "Station-SE", SN)],
    [(a, b, 2, 1) for a, b in
     [(0, 1), (1, 2), (2, 3), (4, 5), (5, 6), (6, 7), (8, 9), (9, 10), (10, 11),
      (12, 13), (13, 14), (14, 15), (0, 4), (4, 8), (8, 12), (1, 5), (5, 9), (9, 13),
      (2, 6), (6, 10), (10, 14), (3, 7), (7, 11), (11, 15)]],
    [(0, 0), (1, 12), (2, 5), (3, 10)],
    [("P1", 0, 15, 0), ("P2", 12, 3, 0), ("P3", 0, 3, 0), ("P4", 12, 15, 4),
     ("P5", 0, 15, 10), ("P6", 12, 3, 10), ("P7", 0, 3, 18), ("P8", 12, 15, 18)],
)

case(
    "l2_05_shared_start_funnel",
    "Level 2: all four units start parked on the same charge dock behind a single "
    "capacity-1 gate aisle, and the gate is shared by outbound and returning traffic. "
    "Punishes polling logic that has every idle unit request the same first hop every "
    "step: three of the four just burn steps colliding. Staggering departures - and "
    "keeping the gate clear for a returning loaded unit - is the whole case.",
    220,
    [(0, "Charge-Dock", TR), (1, "Gate", TR), (2, "Junction", TR), (3, "Storage-A", ST),
     (4, "Storage-B", ST), (5, "Station-1", SN), (6, "Station-2", SN),
     (7, "Aisle-A", TR), (8, "Aisle-B", TR), (9, "Aisle-C", TR)],
    [(0, 1, 3, 1), (1, 2, 2), (2, 3, 2), (2, 4, 2), (3, 7, 2), (7, 5, 2), (4, 8, 2),
     (8, 6, 2), (2, 9, 3), (9, 5, 4)],
    [(0, 0), (1, 0), (2, 0), (3, 0)],
    [("P1", 3, 5, 0), ("P2", 4, 6, 0), ("P3", 3, 6, 4), ("P4", 4, 5, 4),
     ("P5", 3, 5, 12), ("P6", 4, 6, 12)],
)

case(
    "l2_06_idle_units_clog_storage",
    "Level 2: five units, only ever two live pods, and the single storage area sits "
    "behind a capacity-1 feed aisle and has only two drive-unit slots of its own. "
    "Punishes the idle rule 'drift toward the nearest storage node': all five units "
    "queue into the feed aisle, the two that matter cannot get in or out, and the "
    "pods rot. Idle units must hold at the junction and stay out of the way.",
    170,
    [(0, "Storage-A", ST, 2), (1, "Aisle-Feed", TR, 1), (2, "Junction", TR),
     (3, "Station-1", SN), (4, "Bay-1", TR), (5, "Bay-2", TR), (6, "Bay-3", TR),
     (7, "Aisle-2", TR), (8, "Yard", TR)],
    [(0, 1, 2), (1, 2, 2), (2, 3, 3), (2, 4, 2), (2, 5, 2), (2, 6, 2), (2, 7, 2),
     (7, 8, 2)],
    [(0, 4), (1, 5), (2, 6), (3, 7), (4, 8)],
    [("P1", 0, 3, 0), ("P2", 0, 3, 0), ("P3", 0, 3, 18), ("P4", 0, 3, 36)],
)

case(
    "l2_07_pinch_vs_bypass",
    "Level 2: one capacity-1 pinch aisle (weight 4) is the short way east; a three-hop "
    "bypass is 4 steps longer. Three units and six pods, five of them eastbound. "
    "Punishes static shortest-path routing: the second and third unit each wait out the "
    "pinch instead of taking the bypass. The correct policy is queue-aware - take the "
    "bypass exactly when the expected wait exceeds the detour.",
    220,
    [(0, "Storage-W", ST), (1, "Junction-W", TR), (2, "Pinch-W", TR), (3, "Pinch-E", TR),
     (4, "Junction-E", TR), (5, "Station-E", SN), (6, "Bypass-1", TR),
     (7, "Bypass-2", TR), (8, "Bypass-3", TR), (9, "Storage-W2", ST),
     (10, "Station-W", SN), (11, "Park-W", TR)],
    [(0, 1, 2), (9, 1, 2), (1, 2, 2), (2, 3, 4, 1), (3, 4, 2), (4, 5, 2), (1, 6, 3),
     (6, 7, 3), (7, 8, 3), (8, 4, 3), (1, 10, 3), (1, 11, 2)],
    [(0, 0), (1, 9), (2, 11)],
    [("P1", 0, 5, 0), ("P2", 9, 5, 0), ("P3", 0, 5, 6), ("P4", 9, 10, 6),
     ("P5", 0, 5, 14), ("P6", 9, 5, 20)],
)

case(
    "l2_08_oneway_ring_spokes",
    "Level 2: a one-way ring road (six directed segments) with bidirectional spokes to "
    "three storage areas and two stations - a realistic anticlockwise traffic scheme. "
    "Three units, six pods. Punishes symmetric distance assumptions: getting from a "
    "station back to a storage area can cost two thirds of a lap, so unit-to-pod "
    "assignment must be computed on directed distances, not undirected ones.",
    240,
    [(0, "Ring-A", TR), (1, "Ring-B", TR), (2, "Ring-C", TR), (3, "Ring-D", TR),
     (4, "Ring-E", TR), (5, "Ring-F", TR), (6, "Storage-A", ST), (7, "Storage-B", ST),
     (8, "Station-1", SN), (9, "Station-2", SN), (10, "Park", TR),
     (11, "Storage-C", ST)],
    [(0, 1, 3, None, False), (1, 2, 3, None, False), (2, 3, 3, None, False),
     (3, 4, 3, None, False), (4, 5, 3, None, False), (5, 0, 3, None, False),
     (6, 0, 2), (7, 2, 2), (8, 3, 2), (9, 5, 2), (10, 1, 2), (11, 4, 2)],
    [(0, 6), (1, 7), (2, 10)],
    [("P1", 6, 8, 0), ("P2", 7, 9, 0), ("P3", 11, 8, 4), ("P4", 6, 9, 10),
     ("P5", 7, 8, 16), ("P6", 11, 9, 22)],
)

case(
    "l2_09_stale_pod_bait",
    "Level 2: two pods sit 22 steps away at a remote storage area from t=0 while a "
    "steady stream of pods lands at the near storage area every five steps. Two units. "
    "Punishes an age bonus with no cap: once the remote pods are old enough the bonus "
    "outbids every fresh pod and BOTH units set off on a 48-step round trip, stranding "
    "the stream. The right answer sends exactly one unit and only once.",
    260,
    [(0, "Storage-Near", ST), (1, "Aisle-N1", TR), (2, "Station-Near", SN),
     (3, "Long-1", TR), (4, "Long-2", TR), (5, "Long-3", TR), (6, "Storage-Far", ST),
     (7, "Station-Far", SN), (8, "Aisle-N2", TR), (9, "Park", TR), (10, "Long-4", TR)],
    [(0, 1, 2), (1, 2, 2), (0, 8, 3), (8, 2, 3), (0, 3, 5), (3, 4, 5), (4, 10, 5),
     (10, 5, 5), (5, 6, 2), (6, 7, 2), (0, 9, 2)],
    [(0, 0), (1, 9)],
    [("F1", 6, 7, 0), ("F2", 6, 7, 0)] +
    [("N%d" % i, 0, 2, 5 * i) for i in range(1, 13)],
)

case(
    "l2_10_bridge_queue_vs_detour",
    "Level 2: the provided narrow-bridge case, scaled. One capacity-1 bridge of weight "
    "12 is the short way to the station; a two-hop detour costs 14. Three units, six "
    "pods, and the bridge is shared by loaded outbound and empty returning traffic. "
    "Punishes wait-when-blocked: the queue behind the bridge grows to 24+ steps while a "
    "14-step detour sits unused. This is the single highest-value congestion lesson in "
    "the whole set.",
    240,
    [(0, "Storage-W", ST), (1, "Junction-W", TR), (2, "Detour-Mid", TR),
     (3, "Junction-E", TR), (4, "Station-E", SN), (5, "Storage-W2", ST),
     (6, "Park-W", TR)],
    [(0, 1, 2), (5, 1, 2), (1, 3, 12, 1), (1, 2, 7), (2, 3, 7), (3, 4, 2), (1, 6, 2)],
    [(0, 0), (1, 5), (2, 6)],
    [("P1", 0, 4, 0), ("P2", 5, 4, 0), ("P3", 0, 4, 2), ("P4", 5, 4, 8),
     ("P5", 0, 4, 16), ("P6", 5, 4, 24)],
)

case(
    "l2_11_forced_pickup_reroute",
    "Level 2: the short route from the parking bays to the main storage area runs "
    "straight through a quarantine rack holding two pods bound for a station 24 steps "
    "away. Pickups are compulsory, so both capacity-1 units get hijacked the moment "
    "they set foot there and three high-value near pods rot. Punishes ignoring forced "
    "pickups when choosing a path: one unit must take the 10-step detour to keep its "
    "carrying slot free.",
    220,
    [(0, "Park", TR), (1, "Storage-Quarantine", ST), (2, "Storage-Main", ST),
     (3, "Station-Near", SN), (4, "Station-Far", SN), (5, "Detour-1", TR),
     (6, "Detour-2", TR), (7, "Aisle-Far-1", TR), (8, "Aisle-Far-2", TR),
     (9, "Aisle-Far-3", TR)],
    [(0, 1, 2), (1, 2, 2), (0, 5, 4), (5, 6, 4), (6, 2, 2), (2, 3, 2), (1, 7, 6),
     (7, 8, 6), (8, 9, 6), (9, 4, 6)],
    [(0, 0), (1, 0)],
    [("X1", 1, 4, 0), ("X2", 1, 4, 0), ("P1", 2, 3, 0), ("P2", 2, 3, 0),
     ("P3", 2, 3, 6), ("P4", 2, 3, 18)],
)

# =====================================================================
# LEVEL 3 - capacities, docks, batching
# =====================================================================

case(
    "l3_01_dock_hold_approach",
    "Level 3: one capacity-1 station reachable two ways - a 9-step route whose final "
    "approach is a single weight-8 aisle, and a 10-step route whose final approach is "
    "weight 2. Inbound units reserve the dock for the whole traversal, so the 'shorter' "
    "route holds the only dock for 8 steps per pod and throttles four units to one "
    "delivery per 8 steps. Punishes picking routes by total length instead of by how "
    "long the last hop occupies the bottleneck.",
    220,
    [(0, "Storage-A", ST), (1, "Aisle-Short", TR), (2, "Dock-Approach", TR),
     (3, "Station-1", SN, 1), (4, "Storage-B", ST), (5, "Aisle-West", TR),
     (6, "Park-1", TR), (7, "Park-2", TR), (8, "Park-3", TR), (9, "Aisle-North", TR)],
    [(0, 1, 1), (1, 3, 8), (0, 9, 4), (9, 5, 2), (5, 2, 2), (2, 3, 2), (4, 5, 2),
     (4, 0, 5), (0, 6, 2), (0, 7, 2), (0, 8, 2)],
    [(0, 6), (1, 7), (2, 8), (3, 0)],
    [("P1", 0, 3, 0), ("P2", 0, 3, 0), ("P3", 0, 3, 2), ("P4", 0, 3, 4),
     ("P5", 0, 3, 8), ("P6", 0, 3, 12), ("P7", 4, 3, 6), ("P8", 4, 3, 16)],
)

case(
    "l3_02_shared_approach_blocked",
    "Level 3: one capacity-1 waypoint is the only approach to BOTH stations, and "
    "Station-1 has a single dock while Station-2 has three. Punishes queueing at the "
    "shared approach: a unit that rolls up to the approach and finds the single dock "
    "busy parks there and shuts off all Station-2 traffic behind it. The fix is to hold "
    "one node further back, at the junction, until the dock is actually free.",
    240,
    [(0, "Storage-C", ST), (1, "Junction", TR), (2, "Approach", TR, 1),
     (3, "Station-1", SN, 1), (4, "Aisle-2", TR), (5, "Station-2", SN, 3),
     (6, "Park-1", TR), (7, "Park-2", TR), (8, "Park-3", TR), (9, "Park-4", TR),
     (10, "Storage-D", ST), (11, "Aisle-3", TR)],
    [(0, 1, 2), (1, 2, 2), (2, 3, 2), (2, 4, 3), (4, 5, 3), (0, 6, 2), (0, 7, 2),
     (10, 8, 2), (10, 9, 2), (10, 11, 2), (11, 1, 3)],
    [(0, 6), (1, 7), (2, 8), (3, 9)],
    [("P1", 0, 3, 0), ("P2", 0, 5, 0), ("P3", 10, 3, 0), ("P4", 10, 5, 2),
     ("P5", 0, 3, 8), ("P6", 0, 5, 8), ("P7", 10, 3, 16), ("P8", 10, 5, 16)],
)

case(
    "l3_03_batch_across_storage",
    "Level 3: capacity-4 units, a capacity-1 station, and pods split two-and-two across "
    "two storage areas so that filling a unit requires visiting both. The detour "
    "between the racks costs 6 steps; a second trip through the single dock costs far "
    "more. Punishes a fixed detour-slack threshold that refuses any diversion longer "
    "than a few steps - the dock, not the aisle, is the scarce resource here.",
    240,
    [(0, "Storage-A", ST), (1, "Storage-B", ST), (2, "Junction", TR),
     (3, "Dock-Approach", TR), (4, "Station-1", SN, 1), (5, "Park-1", TR),
     (6, "Park-2", TR), (7, "Aisle-A", TR), (8, "Aisle-B", TR), (9, "Yard", TR)],
    [(0, 7, 2), (7, 2, 2), (1, 8, 2), (8, 2, 2), (2, 3, 3), (3, 4, 3), (0, 5, 2),
     (1, 6, 2), (2, 9, 2)],
    [(0, 5, 4), (1, 6, 4)],
    [("A1", 0, 4, 0), ("A2", 0, 4, 0), ("B1", 1, 4, 0), ("B2", 1, 4, 0),
     ("A3", 0, 4, 26), ("A4", 0, 4, 26), ("B3", 1, 4, 26), ("B4", 1, 4, 26),
     ("A5", 0, 4, 60), ("B5", 1, 4, 60)],
)

case(
    "l3_04_vacate_the_dock",
    "Level 3: a capacity-1 station sitting behind a capacity-1 approach waypoint - a "
    "genuine one-in-one-out dock. Punishes rolling onto the approach before the dock is "
    "free: the unit on the dock then cannot leave (the approach is full) and the unit "
    "on the approach cannot advance (the dock is full), and the pair gridlocks "
    "permanently. Correct play is to keep the approach empty until the dock clears.",
    220,
    [(0, "Storage-A", ST), (1, "Aisle-1", TR), (2, "Approach", TR, 1),
     (3, "Station-1", SN, 1), (4, "Park-A", TR), (5, "Park-B", TR), (6, "Park-C", TR),
     (7, "Aisle-2", TR), (8, "Storage-B", ST)],
    [(0, 1, 2), (1, 2, 2), (2, 3, 2), (0, 4, 2), (0, 5, 2), (0, 6, 2), (8, 7, 2),
     (7, 1, 2)],
    [(0, 4), (1, 5), (2, 6)],
    [("P1", 0, 3, 0), ("P2", 0, 3, 0), ("P3", 0, 3, 2), ("P4", 0, 3, 4),
     ("P5", 0, 3, 10), ("P6", 0, 3, 16), ("P7", 8, 3, 6), ("P8", 8, 3, 20)],
)

case(
    "l3_05_zone_warehouse",
    "Level 3: a realistic 18-node floor - two rack rows, a cross aisle, a dock lane "
    "with a capacity-1 segment, three stations with 1/2/1 docks, and four capacity-2 "
    "units starting at charge/park nodes. Eleven pods in three waves with mixed "
    "destinations. This is the integration case: batching, dock discipline, aisle "
    "contention and assignment all bind at once, and a driver that only does one of "
    "them well loses points on the others.",
    260,
    [(0, "Storage-A1", ST), (1, "Storage-A2", ST), (2, "Storage-A3", ST),
     (3, "Storage-B1", ST), (4, "Storage-B2", ST), (5, "Storage-B3", ST),
     (6, "Cross-W", TR), (7, "Cross-M", TR), (8, "Cross-E", TR),
     (9, "Dock-Lane-1", TR), (10, "Dock-Lane-2", TR), (11, "Dock-Lane-3", TR),
     (12, "Station-1", SN, 1), (13, "Station-2", SN, 2), (14, "Station-3", SN, 1),
     (15, "Park-1", TR), (16, "Park-2", TR), (17, "Charge", TR)],
    [(0, 1, 2), (1, 2, 2), (3, 4, 2), (4, 5, 2), (6, 7, 3), (7, 8, 3), (0, 6, 2),
     (2, 7, 2), (3, 7, 2), (5, 8, 2), (6, 9, 3, 2), (9, 10, 3, 1), (10, 11, 3),
     (9, 12, 2), (10, 13, 2), (11, 14, 2), (7, 15, 2), (8, 16, 2), (6, 17, 2),
     (8, 11, 4)],
    [(0, 17, 2), (1, 15, 2), (2, 16, 2), (3, 9, 2)],
    [("P1", 0, 12, 0), ("P2", 1, 12, 0), ("P3", 3, 13, 0), ("P4", 5, 14, 0),
     ("P5", 2, 13, 12), ("P6", 4, 14, 12), ("P7", 0, 13, 12), ("P8", 5, 12, 30),
     ("P9", 1, 14, 30), ("P10", 3, 12, 30), ("P11", 4, 13, 48)],
)

case(
    "l3_06_metered_corridor",
    "Level 3: node capacities used as a traffic meter - a corridor of waypoints with "
    "capacities 2, 1, 2 leading to a two-dock station, plus a 14-step bypass. Four "
    "units, eight pods. Punishes treating node capacity as a station-only feature: the "
    "capacity-1 middle waypoint serialises the whole fleet, and a unit that commits to "
    "the corridor while it is full simply burns steps at the junction instead of "
    "spending them on the bypass.",
    240,
    [(0, "Storage-W", ST), (1, "Meter-1", TR, 2), (2, "Meter-2", TR, 1),
     (3, "Meter-3", TR, 2), (4, "Station-E", SN, 2), (5, "Storage-W2", ST),
     (6, "Park-1", TR), (7, "Park-2", TR), (8, "Park-3", TR), (9, "Bypass-1", TR),
     (10, "Junction-W", TR)],
    [(0, 10, 2), (5, 10, 2), (10, 1, 2), (1, 2, 2), (2, 3, 2), (3, 4, 2), (10, 9, 6),
     (9, 3, 6), (10, 6, 2), (10, 7, 2), (10, 8, 2)],
    [(0, 6), (1, 7), (2, 8), (3, 0)],
    [("P1", 0, 4, 0), ("P2", 5, 4, 0), ("P3", 0, 4, 0), ("P4", 5, 4, 4),
     ("P5", 0, 4, 10), ("P6", 5, 4, 14), ("P7", 0, 4, 22), ("P8", 5, 4, 30)],
)

case(
    "l3_07_cross_dock",
    "Level 3: a cross-docking floor. Pods arrive ON an inbound station node "
    "(source_node is typed 'station', never 'storage') and must be moved to one of two "
    "single-dock outbound stations; the one node typed 'storage' is an empty buffer "
    "rack. Punishes any driver that looks for work at storage nodes or parks idle units "
    "there - the work all happens at the inbound dock, and loitering on it burns one of "
    "its two slots.",
    240,
    [(0, "Station-Inbound", SN, 2), (1, "Aisle-1", TR), (2, "Junction", TR),
     (3, "Station-A", SN, 1), (4, "Station-B", SN, 1), (5, "Aisle-2", TR),
     (6, "Aisle-3", TR), (7, "Storage-Buffer", ST), (8, "Park-1", TR),
     (9, "Park-2", TR), (10, "Aisle-4", TR)],
    [(0, 1, 2), (1, 2, 2), (2, 5, 2), (5, 3, 2), (2, 6, 2), (6, 4, 2), (2, 7, 3),
     (7, 8, 2), (7, 9, 2), (2, 10, 3), (10, 4, 3)],
    [(0, 8), (1, 9), (2, 7)],
    [("P1", 0, 3, 0), ("P2", 0, 4, 0), ("P3", 0, 3, 6), ("P4", 0, 4, 6),
     ("P5", 7, 3, 12), ("P6", 0, 4, 14), ("P7", 0, 3, 20), ("P8", 7, 4, 20)],
)

case(
    "l3_08_pod_at_own_dock",
    "Level 3: two returned pods spawn on the single-dock stations they are addressed to "
    "(source_node == destination_station), mixed into four ordinary hauls. A unit "
    "standing on the dock when such a pod lands delivers it at duration 0 for free - but "
    "only if it does not move that step, and it must clear the capacity-1 dock "
    "immediately afterwards. Punishes both halves: idle rules that drive the unit off "
    "with the pod still aboard, and dock-camping that blocks the next delivery.",
    220,
    [(0, "Storage-A", ST), (1, "Aisle-1", TR), (2, "Station-1", SN, 1),
     (3, "Aisle-2", TR), (4, "Station-2", SN, 1), (5, "Park-1", TR), (6, "Park-2", TR),
     (7, "Storage-B", ST), (8, "Aisle-3", TR), (9, "Junction", TR)],
    [(0, 9, 2), (9, 1, 2), (1, 2, 2), (9, 3, 2), (3, 4, 2), (0, 5, 2), (0, 6, 2),
     (7, 8, 2), (8, 9, 2)],
    [(0, 2), (1, 5)],
    [("R1", 2, 2, 0), ("P1", 0, 2, 0), ("P2", 7, 4, 4), ("R2", 4, 4, 12),
     ("P3", 0, 4, 18), ("P4", 7, 2, 26)],
)

case(
    "l3_09_overload_horizon",
    "Level 3: fourteen pods, two units and a 120-step horizon - strictly more demand "
    "than the fleet can serve. Eight cheap pods cycle on an 8-step loop next door; six "
    "expensive ones sit 18 steps away behind a long aisle. Punishes both fairness and "
    "pure greed: serving the far queue at all costs strands the cheap stream, while "
    "never going costs six pods outright. Score comes from how many pods clear, "
    "weighted by decay, not from equalising lateness.",
    120,
    [(0, "Storage-Near", ST), (1, "Aisle-N1", TR), (2, "Station-Near", SN),
     (3, "Storage-Far", ST), (4, "Aisle-F1", TR), (5, "Aisle-F2", TR),
     (6, "Station-Far", SN), (7, "Park", TR), (8, "Aisle-N2", TR),
     (9, "Aisle-F3", TR), (10, "Junction", TR), (11, "Storage-Mid", ST)],
    [(0, 1, 2), (1, 2, 2), (0, 8, 3), (8, 2, 3), (0, 10, 3), (10, 11, 3), (11, 4, 4),
     (4, 5, 4), (5, 3, 4), (3, 9, 3), (9, 6, 3), (5, 6, 6), (0, 7, 2)],
    [(0, 0), (1, 7)],
    [("N%d" % i, 0, 2, 4 * i) for i in range(8)] +
    [("F%d" % i, 3, 6, 6 * i) for i in range(6)],
)

case(
    "l3_10_poison_pod_detour",
    "Level 3: the quarantine rack on the short route holds two pods addressed to an "
    "orphan station in a disconnected part of the floor. Pickups are compulsory and "
    "these pods can never be delivered, so a capacity-1 unit that steps onto the "
    "quarantine node is dead for the rest of the run. Punishes 'shortest path, then "
    "take whatever pod is there': both units die on the short route and the case scores "
    "zero, while the 10-step detour delivers all four real pods.",
    180,
    [(0, "Park", TR), (1, "Junction-W", TR), (2, "Storage-Quarantine", ST),
     (3, "Storage-Main", ST), (4, "Station-1", SN), (5, "Detour-1", TR),
     (6, "Detour-2", TR), (7, "Aisle-N", TR), (8, "Station-Orphan", SN),
     (9, "Storage-Orphan", ST), (10, "Park-2", TR)],
    [(0, 2, 2), (2, 3, 2), (0, 5, 4), (5, 6, 4), (6, 3, 2), (3, 4, 2), (0, 10, 2),
     (0, 1, 2), (1, 7, 2), (7, 3, 6), (8, 9, 2)],
    [(0, 0), (1, 10)],
    [("X1", 2, 8, 0), ("X2", 2, 8, 0), ("P1", 3, 4, 0), ("P2", 3, 4, 0),
     ("P3", 3, 4, 6), ("P4", 3, 4, 12)],
)

case(
    "l3_11_batch_or_return",
    "Level 3: a capacity-2 unit, pods paired one at each of two storage areas, and a "
    "20-step round trip to the station. Collecting the partner pod costs an 8-step "
    "diversion; skipping it costs a 28-step return trip. Punishes a hard-coded detour "
    "slack (typically about 5 steps) that refuses the diversion: the unit shuttles "
    "single pods and the partner pods age a full cycle each wave.",
    200,
    [(0, "Storage-A", ST), (1, "Aisle-1", TR), (2, "Aisle-2", TR), (3, "Station-1", SN),
     (4, "Park", TR), (5, "Storage-B", ST), (6, "Aisle-3", TR), (7, "Station-2", SN)],
    [(0, 1, 4), (1, 2, 4), (2, 3, 2), (0, 4, 2), (0, 5, 6), (5, 6, 6), (6, 2, 4),
     (3, 7, 4)],
    [(0, 0, 2)],
    [("A1", 0, 3, 0), ("B1", 5, 3, 0), ("A2", 0, 3, 34), ("B2", 5, 3, 34),
     ("A3", 0, 3, 70), ("B3", 5, 3, 70), ("C1", 0, 7, 100)],
)

case(
    "l3_12_horizon_and_late_spawns",
    "Level 3: two pods spawn one and two steps before the horizon closes and are only "
    "catchable by a unit already standing on their storage node (a weight-1 hop "
    "delivers at duration 0); two more are scheduled after max_time_steps and never "
    "spawn at all, yet still sit in the score denominator. Punishes drivers that "
    "wander between waves and drivers that break on pods they never see. Part of this "
    "case is unwinnable by construction - that is the point: the ceiling is below 100 "
    "and chasing it must not cost the pods that are reachable.",
    100,
    [(0, "Storage-A", ST), (1, "Aisle-1", TR), (2, "Station-1", SN, 2),
     (3, "Storage-B", ST), (4, "Aisle-2", TR), (5, "Station-2", SN),
     (6, "Park", TR), (7, "Junction", TR)],
    [(0, 7, 2), (7, 1, 2), (1, 2, 2), (0, 2, 1), (3, 4, 2), (4, 7, 2), (7, 5, 4),
     (0, 6, 2)],
    [(0, 0), (1, 6)],
    [("P1", 0, 2, 0), ("P2", 3, 5, 4), ("P3", 0, 2, 20), ("P4", 3, 5, 40),
     ("P5", 0, 2, 60), ("L1", 0, 2, 98), ("L2", 0, 2, 99),
     ("Z1", 0, 2, 100), ("Z2", 3, 5, 160)],
)


def main():
    out = HERE
    for name, data in sorted(CASES.items()):
        with open(os.path.join(out, name + ".json"), "w") as f:
            json.dump(data, f, indent=2)
            f.write("\n")
    print("wrote %d cases to %s" % (len(CASES), out))


if __name__ == "__main__":
    main()
