#!/usr/bin/env python3
import glob, importlib.util, os, time, traceback
def load(path):
    spec = importlib.util.spec_from_file_location('c%d' % time.time_ns(), path)
    m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
    return m.drive_unit_next_move
official = sorted(glob.glob('test_cases/level*/test_case_*.json'))
stress   = sorted(glob.glob('test_cases/extra/s*.json'))
corpus   = sorted(glob.glob('test_cases/corpus/*.json'))
drivers = sorted(glob.glob('variants/*.py')) + ['ar_hackathon/examples/basic_driver.py']
rows=[]
for d in drivers:
    try:
        from ar_hackathon.engine.game_engine import GameEngine
        fn_name = 'basic_driver' if 'basic' in d else 'drive_unit_next_move'
        spec = importlib.util.spec_from_file_location('c%d' % time.time_ns(), d)
        m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
        fn = getattr(m, fn_name)
        tots={}
        t0=time.time()
        for label,cases in (('OFF',official),('STR',stress),('COR',corpus)):
            s=0.0
            for c in cases:
                spec2 = importlib.util.spec_from_file_location('c%d' % time.time_ns(), d)
                m2 = importlib.util.module_from_spec(spec2); spec2.loader.exec_module(m2)
                s += GameEngine(c, getattr(m2,fn_name)).run_until_finished()['score']
            tots[label]=s
        rows.append((tots['COR'], tots['OFF'], tots['STR'], time.time()-t0, d))
    except Exception as e:
        rows.append((-1,-1,-1,0,d+"  ERROR:"+str(e)[:40]))
rows.sort()
print(f"{'driver':<34} {'OFFICIAL':>9} {'STRESS':>8} {'CORPUS':>8} {'ALL':>9} {'wall':>6}")
for cor,off,st,w,d in rows:
    print(f"{os.path.basename(d):<34} {off:>9.2f} {st:>8.2f} {cor:>8.1f} {off+st+cor:>9.1f} {w:>5.1f}s")
