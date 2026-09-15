#!/usr/bin/env python3
"""Score a driver over the 31-case hidden-set proxy corpus."""
import glob, importlib.util, os, sys, time
def load(path):
    spec = importlib.util.spec_from_file_location('c%d' % time.time_ns(), path)
    m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m); return m.drive_unit_next_move
def main():
    path = os.environ.get('DRIVER', 'ar_hackathon/api/routing.py')
    tot = 0.0; bad = []
    t0 = time.time()
    for case in sorted(glob.glob('test_cases/corpus/*.json')):
        from ar_hackathon.engine.game_engine import GameEngine
        s = GameEngine(case, load(path)).run_until_finished()
        tot += s['score']
        if s['delivered_pods'] < s['total_pods']:
            bad.append(f"{os.path.basename(case)[:-5]:<32} {s['delivered_pods']}/{s['total_pods']} {s['score']:6.2f}")
    print(f"DRIVER {path}")
    print(f"CORPUS {tot:.1f}  wall {time.time()-t0:.1f}s  imperfect {len(bad)}")
    for b in bad: print("   ", b)
if __name__ == '__main__': main()
