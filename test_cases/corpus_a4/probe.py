import json, os, sys, logging, tempfile
logging.disable(logging.CRITICAL)
sys.path.insert(0, "/Users/alexliu/ArHackathon2026")
from ar_hackathon.engine.game_engine import GameEngine

def run(tc, driver):
    f = tempfile.NamedTemporaryFile("w", suffix=".json", delete=False)
    json.dump(tc, f); f.close()
    eng = GameEngine(f.name, driver)
    return eng.run_until_finished(), eng

# PROBE 1: fractional weight -> ceil?
tc = {"metadata":{"max_time_steps":50},
 "nodes":[{"id":0,"type":"storage"},{"id":1,"type":"station"}],
 "edges":[{"from_node":0,"to_node":1,"weight":1.2}],
 "drive_units":[{"id":0,"start_node":0}],
 "pods":[{"id":"P1","source_node":0,"destination_station":1,"entry_time":0}]}
s,_ = run(tc, lambda uid, st: 1)
print("P1 fractional w=1.2 ->", s["average_delivery_time"], "(2 => ceil)", s["delivered_pods"])
tc["edges"][0]["weight"]=1.0
s,_ = run(tc, lambda uid, st: 1)
print("P1 w=1.0 ->", s["average_delivery_time"], "(0 => same-step arrive+deliver)")
tc["edges"][0]["weight"]=2.0
s,_ = run(tc, lambda uid, st: 1)
print("P1 w=2.0 ->", s["average_delivery_time"])

# PROBE 2: source == destination_station, unit already there, driver moves away
tc2 = {"metadata":{"max_time_steps":30},
 "nodes":[{"id":0,"type":"station"},{"id":1,"type":"storage"}],
 "edges":[{"from_node":0,"to_node":1,"weight":3}],
 "drive_units":[{"id":0,"start_node":0}],
 "pods":[{"id":"P1","source_node":0,"destination_station":0,"entry_time":0}]}
s,_ = run(tc2, lambda uid, st: None)
print("src==dst, unit waits ->", s["delivered_pods"], "dur", s["average_delivery_time"])
s,_ = run(tc2, lambda uid, st: 1)
print("src==dst, unit drives off ->", s["delivered_pods"], "dur", s["average_delivery_time"])

# PROBE 3: initial occupancy > node capacity legal?
tc3 = {"metadata":{"max_time_steps":30},
 "nodes":[{"id":0,"type":"storage","capacity":1},{"id":1,"type":"station"}],
 "edges":[{"from_node":0,"to_node":1,"weight":2}],
 "drive_units":[{"id":0,"start_node":0},{"id":1,"start_node":0},{"id":2,"start_node":0}],
 "pods":[{"id":"P%d"%i,"source_node":0,"destination_station":1,"entry_time":0} for i in range(3)]}
s,_ = run(tc3, lambda uid, st: 1)
print("3 units on cap-1 node ->", s["delivered_pods"],"/",s["total_pods"], "avg", s["average_delivery_time"])

# PROBE 4: pod entry_time beyond horizon
tc4 = {"metadata":{"max_time_steps":10},
 "nodes":[{"id":0,"type":"storage"},{"id":1,"type":"station"}],
 "edges":[{"from_node":0,"to_node":1,"weight":1}],
 "drive_units":[{"id":0,"start_node":0}],
 "pods":[{"id":"A","source_node":0,"destination_station":1,"entry_time":0},
         {"id":"B","source_node":0,"destination_station":1,"entry_time":9},
         {"id":"C","source_node":0,"destination_station":1,"entry_time":10},
         {"id":"D","source_node":0,"destination_station":1,"entry_time":50}]}
s,_ = run(tc4, lambda uid, st: 1 if st.get_drive_unit(uid).current_node==0 else 0)
print("horizon pods ->", s["delivered_pods"],"/",s["total_pods"], "score", round(s["score"],2))
