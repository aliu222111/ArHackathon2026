#!/usr/bin/env python3
"""
Validates every test-case JSON in this directory.

Checks, in order:
  1. JSON parses and validates against test_cases/schema.json
     (minimal draft-07 subset validator - the repo has no `jsonschema`).
  2. It loads through ar_hackathon.utils.json_loader.load_test_case.
  3. Structural integrity: unique ids, every referenced node exists,
     pod destinations are station-typed, unit start nodes exist.
  4. Solvable in principle, using DIRECTED reachability with ceil(weight)
     costs (the engine's true per-edge cost):
       - some drive unit can reach the pod's source_node, AND
       - the pod's source_node can reach its destination_station, AND
       - entry_time + (best unit->source) + (source->dest) <= max_time_steps.
     A pod failing any of these is UNDELIVERABLE. A file is INVALID only if
     it has no deliverable pod at all (such a case teaches nothing);
     partially-undeliverable files are reported so the report can state
     which zeros are unwinnable by construction.

Usage:  python3 validate.py
Exit status 0 iff 0 invalid files.
"""

import glob
import heapq
import json
import math
import os
import sys

REPO = "/Users/alexliu/ArHackathon2026"
sys.path.insert(0, REPO)
SCHEMA_PATH = os.path.join(REPO, "test_cases", "schema.json")
HERE = os.path.dirname(os.path.abspath(__file__))

from ar_hackathon.utils.json_loader import load_test_case  # noqa: E402


# --------------------------------------------------------------------- schema
def schema_errors(inst, schema, path="$"):
    """Minimal draft-07 subset: type/enum/minimum/required/properties/items."""
    errs = []
    t = schema.get("type")
    if t == "object":
        if not isinstance(inst, dict):
            return ["%s: expected object, got %s" % (path, type(inst).__name__)]
        for req in schema.get("required", []):
            if req not in inst:
                errs.append("%s: missing required property '%s'" % (path, req))
        for key, sub in schema.get("properties", {}).items():
            if key in inst:
                errs += schema_errors(inst[key], sub, "%s.%s" % (path, key))
    elif t == "array":
        if not isinstance(inst, list):
            return ["%s: expected array, got %s" % (path, type(inst).__name__)]
        item_schema = schema.get("items")
        if item_schema:
            for i, item in enumerate(inst):
                errs += schema_errors(item, item_schema, "%s[%d]" % (path, i))
    elif t == "integer":
        if isinstance(inst, bool) or not isinstance(inst, int):
            errs.append("%s: expected integer, got %r" % (path, inst))
    elif t == "number":
        if isinstance(inst, bool) or not isinstance(inst, (int, float)):
            errs.append("%s: expected number, got %r" % (path, inst))
    elif t == "string":
        if not isinstance(inst, str):
            errs.append("%s: expected string, got %r" % (path, inst))
    elif t == "boolean":
        if not isinstance(inst, bool):
            errs.append("%s: expected boolean, got %r" % (path, inst))
    if "enum" in schema and inst not in schema["enum"]:
        errs.append("%s: %r not in enum %r" % (path, inst, schema["enum"]))
    if "minimum" in schema and isinstance(inst, (int, float)) \
            and not isinstance(inst, bool) and inst < schema["minimum"]:
        errs.append("%s: %r below minimum %r" % (path, inst, schema["minimum"]))
    return errs


# ------------------------------------------------------------------ distances
def all_pairs(tc):
    """Directed shortest paths with the engine's real per-edge cost ceil(w)."""
    adj = {n.id: [] for n in tc.nodes}
    for e in tc.edges:
        c = int(math.ceil(e.weight))
        adj.setdefault(e.from_node, []).append((e.to_node, c))
        if e.bidirectional:
            adj.setdefault(e.to_node, []).append((e.from_node, c))
    dist = {}
    for src in adj:
        d = {src: 0}
        pq = [(0, src)]
        seen = set()
        while pq:
            cd, u = heapq.heappop(pq)
            if u in seen:
                continue
            seen.add(u)
            for v, w in adj.get(u, []):
                nd = cd + w
                if nd < d.get(v, float("inf")):
                    d[v] = nd
                    heapq.heappush(pq, (nd, v))
        dist[src] = d
    return dist


def check(path):
    """Returns (ok, notes, stats)."""
    notes = []
    name = os.path.basename(path)
    with open(path) as f:
        raw = json.load(f)

    with open(SCHEMA_PATH) as f:
        schema = json.load(f)
    errs = schema_errors(raw, schema)
    if errs:
        return False, ["SCHEMA " + e for e in errs], {}

    tc = load_test_case(path)

    node_ids = [n.id for n in tc.nodes]
    if len(set(node_ids)) != len(node_ids):
        errs.append("duplicate node ids")
    ids = set(node_ids)
    types = {n.id: n.node_type for n in tc.nodes}
    for e in tc.edges:
        if e.from_node not in ids or e.to_node not in ids:
            errs.append("edge %s->%s references unknown node" % (e.from_node, e.to_node))
    uids = [u.id for u in tc.drive_units]
    if len(set(uids)) != len(uids):
        errs.append("duplicate drive unit ids")
    for u in tc.drive_units:
        if u.current_node not in ids:
            errs.append("unit %s starts on unknown node %s" % (u.id, u.current_node))
    pods = [p for lst in tc.pods_by_time.values() for p in lst]
    pids = [p.id for p in pods]
    if len(set(pids)) != len(pids):
        errs.append("duplicate pod ids")
    for p in pods:
        if p.current_node not in ids:
            errs.append("pod %s source %s unknown" % (p.id, p.current_node))
        if p.destination_station not in ids:
            errs.append("pod %s destination %s unknown" % (p.id, p.destination_station))
        elif types.get(p.destination_station) != "station":
            errs.append("pod %s destination %s is not typed 'station'"
                        % (p.id, p.destination_station))
    if not tc.drive_units:
        errs.append("no drive units")
    if not pods:
        errs.append("no pods")
    if errs:
        return False, errs, {}

    dist = all_pairs(tc)
    starts = [u.current_node for u in tc.drive_units]
    deliverable, why = 0, []
    for p in sorted(pods, key=lambda q: (q.entry_time, q.id)):
        reach = min((dist.get(s, {}).get(p.current_node, float("inf")) for s in starts),
                    default=float("inf"))
        haul = dist.get(p.current_node, {}).get(p.destination_station, float("inf"))
        if p.entry_time >= tc.max_time_steps:
            why.append("%s never spawns (entry %d >= max_time_steps %d)"
                       % (p.id, p.entry_time, tc.max_time_steps))
        elif reach == float("inf"):
            why.append("%s source unreachable by any unit" % p.id)
        elif haul == float("inf"):
            why.append("%s destination unreachable from its source" % p.id)
        elif p.entry_time + reach + haul > tc.max_time_steps:
            why.append("%s cannot arrive before horizon (%d+%d+%d > %d)"
                       % (p.id, p.entry_time, reach, haul, tc.max_time_steps))
        else:
            deliverable += 1
    if deliverable == 0:
        return False, ["no pod is deliverable even in principle"] + why, {}

    notes += why
    stats = {
        "nodes": len(tc.nodes), "edges": len(tc.edges), "units": len(tc.drive_units),
        "pods": len(pods), "deliverable": deliverable, "max_t": tc.max_time_steps,
        "ceiling": 100.0 * deliverable / len(pods),
    }
    return True, notes, stats


def main():
    files = sorted(glob.glob(os.path.join(HERE, "*.json")))
    bad = 0
    print("%-34s %5s %5s %4s %5s %5s %8s  %s"
          % ("case", "nodes", "edges", "du", "pods", "dlvbl", "ceiling", "notes"))
    print("-" * 110)
    for p in files:
        ok, notes, st = check(p)
        name = os.path.basename(p)
        if not ok:
            bad += 1
            print("%-34s INVALID  %s" % (name, "; ".join(notes)))
            continue
        print("%-34s %5d %5d %4d %5d %5d %7.1f%%  %s"
              % (name, st["nodes"], st["edges"], st["units"], st["pods"],
                 st["deliverable"], st["ceiling"], "; ".join(notes)))
    print("-" * 110)
    print("%d files checked, %d invalid" % (len(files), bad))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
