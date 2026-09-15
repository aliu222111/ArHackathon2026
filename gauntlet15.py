#!/usr/bin/env python3
"""15-case gauntlet: 6 official + 9 stress. DRIVER=path/to/file.py python3 gauntlet15.py
Hard constraint for any candidate: OFFICIAL total must stay >= 505.20."""
import glob, importlib.util, os, sys, time

def load_driver():
    path = os.environ.get('DRIVER', 'ar_hackathon/api/routing.py')
    spec = importlib.util.spec_from_file_location('candidate_driver_%d' % (time.time_ns() % 10**9), path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.drive_unit_next_move

def main():
    official = sorted(glob.glob('test_cases/level*/test_case_*.json'))
    stress = sorted(glob.glob('test_cases/extra/s*.json'))
    tot_o = tot_s = 0.0
    t_all = time.time()
    for label, cases in (('OFFICIAL', official), ('STRESS', stress)):
        for case in cases:
            driver = load_driver()
            from ar_hackathon.engine.game_engine import GameEngine
            eng = GameEngine(case, driver)
            s = eng.run_until_finished()
            if label == 'OFFICIAL': tot_o += s['score']
            else: tot_s += s['score']
            print(f"  {label:<8} {os.path.basename(case):<24} {s['score']:>7.2f}  {s['delivered_pods']}/{s['total_pods']}")
    print(f"OFFICIAL {tot_o:.2f}  STRESS {tot_s:.2f}  ALL {tot_o+tot_s:.2f}  wall {time.time()-t_all:.1f}s")

if __name__ == '__main__':
    main()
