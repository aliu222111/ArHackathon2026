#!/usr/bin/env python3
"""
Scores a candidate driver on the corpus_a4 prediction set.
Modelled on scratchpad/gauntlet.py (which is read-only and not edited).

Usage:
    DRIVER=/abs/path/to/candidate.py python3 run_corpus.py
    DRIVER=... python3 run_corpus.py l2_01            # substring filter

Each case gets a FRESH module instance so module-level caches in the
candidate cannot leak between cases (gauntlet.py does the same).
"""

import glob
import importlib.util
import logging
import os
import sys
import time

logging.disable(logging.CRITICAL)
sys.path.insert(0, "/Users/alexliu/ArHackathon2026")
from ar_hackathon.engine.game_engine import GameEngine  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
DRIVER = os.environ["DRIVER"]
FILT = sys.argv[1] if len(sys.argv) > 1 else ""


def load():
    spec = importlib.util.spec_from_file_location("cand", DRIVER)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m.drive_unit_next_move


cases = sorted(p for p in glob.glob(os.path.join(HERE, "*.json")) if FILT in p)
total = 0.0
rows = []
t0 = time.time()
print("%-34s %8s %8s %9s %7s %6s" % ("case", "score", "pods", "avg dur", "steps", "sec"))
print("-" * 80)
for c in cases:
    name = os.path.basename(c)
    ts = time.time()
    try:
        s = GameEngine(c, load()).run_until_finished()
        sc, dl, tp = s["score"], s["delivered_pods"], s["total_pods"]
        avg, steps = s["average_delivery_time"], s["total_time_steps"]
    except Exception as e:
        sc, dl, tp, avg, steps = 0.0, 0, -1, 0.0, 0
        print("  CRASH %s: %s: %s" % (name, type(e).__name__, e))
    total += sc
    rows.append((sc, name, dl, tp))
    print("%-34s %8.2f %4d/%-3d %9.2f %7d %6.1f"
          % (name, sc, dl, tp, avg, steps, time.time() - ts))
print("-" * 80)
print("CASES %d   TOTAL %.2f   MEAN %.2f   wall %.1fs"
      % (len(cases), total, total / max(1, len(cases)), time.time() - t0))
zeros = [r for r in rows if r[0] < 0.005]
stranded = [r for r in rows if r[3] > 0 and r[2] < r[3]]
print("ZEROS (%d): %s" % (len(zeros), ", ".join(r[1] for r in zeros) or "none"))
print("STRANDED (%d): %s" % (len(stranded),
                             ", ".join("%s %d/%d" % (r[1], r[2], r[3])
                                       for r in stranded) or "none"))
