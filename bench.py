#!/usr/bin/env python3
"""Benchmark harness: run all test cases, print per-case + total scores.

Usage:
  python3 bench.py                      # benchmarks ar_hackathon/api/routing.py
  DRIVER=path/to/file.py python3 bench.py   # benchmarks any driver file
  python3 bench.py level2               # filter cases by substring
"""
import glob, importlib, importlib.util, os, sys, time

def load_driver():
    path = os.environ.get('DRIVER')
    if not path:
        import ar_hackathon.api.routing as routing
        importlib.reload(routing)
        return routing.drive_unit_next_move
    spec = importlib.util.spec_from_file_location('candidate_driver', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.drive_unit_next_move

def main():
    cases = sorted(glob.glob('test_cases/level*/test_case_*.json'))
    args = [a for a in sys.argv[1:]]
    if args:
        cases = [c for c in cases if any(a in c for a in args)]
    total = 0.0
    print(f"{'case':<40} {'score':>7} {'deliv':>7} {'avg_t':>7} {'steps':>6} {'wall':>6}")
    for case in cases:
        driver = load_driver()  # fresh module state per case
        from ar_hackathon.engine.game_engine import GameEngine
        t0 = time.time()
        eng = GameEngine(case, driver)
        s = eng.run_until_finished()
        wall = time.time() - t0
        total += s['score']
        print(f"{case:<40} {s['score']:>7.2f} {s['delivered_pods']:>3}/{s['total_pods']:<3} "
              f"{s['average_delivery_time']:>7.1f} {s['total_time_steps']:>6} {wall:>5.1f}s")
    print(f"{'TOTAL':<40} {total:>7.2f}")

if __name__ == '__main__':
    main()
