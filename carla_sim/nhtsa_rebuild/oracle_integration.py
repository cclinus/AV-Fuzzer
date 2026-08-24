import carla, json, os, sys, random, math
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.append(HERE); sys.path.append(r"C:\Users\10184\AV-Fuzzer\carla_sim")
import tools, nhtsa_sim, safety_oracle

random.seed(1)
TICK, FRAMES = 0.05, 400
NUM = FRAMES // 20 + 1
client = carla.Client('localhost', 2000); client.set_timeout(60.0)
world = client.load_world('Town03'); m = world.get_map(); nhtsa_sim._cur['map'] = 'Town03'
routes = json.load(open(os.path.join(HERE, 'verified_routes_Town03.json')))['routes']
CROSS = next(s for s in json.load(open(os.path.join(HERE, 'scenarios.json'))) if s['conflict'] == 'crossing')
STR = [r for r in routes if r['spawn_index'] in (132, 198, 264)]


def wp(xyz): return m.get_waypoint(carla.Location(x=xyz[0], y=xyz[1], z=xyz[2]))
def loc(w): l = w.transform.location; return [round(l.x, 2), round(l.y, 2), round(l.z + 0.3, 2)]
def tp(cfg): return tools.generate_npc_behaviors(cfg, NUM, True)
def adj(ew):
    a = ew.get_left_lane()
    if a and a.lane_type == carla.LaneType.Driving and a.lane_id * ew.lane_id > 0: return a, -1.0
    a = ew.get_right_lane()
    if a and a.lane_type == carla.LaneType.Driving and a.lane_id * ew.lane_id > 0: return a, 1.0
    return None, None


def tailgater():
    r = random.choice(STR); lead = wp(r['start']).next(15)[0]
    return {'map': 'Town03', 'ego_mode': 'reckless', 'ego_throttle_floor': 0.7,
            'ev': {'start': r['start'], 'rotation': r['rotation'], 'end': r['end']},
            'npcs': [{'start': loc(lead), 'rotation': round(lead.transform.rotation.yaw, 1)}]}, \
        [tp({'throttle_range': [0.3, 0.5], 'brake_chance': 0.45, 'brake_range': [0.2, 0.55], 'steer_range': [0, 0.02]})]

def trailer():
    r = random.choice(STR); back = wp(r['start']).previous(12)[0]
    return {'map': 'Town03', 'ego_mode': 'normal',
            'ev': {'start': r['start'], 'rotation': r['rotation'], 'end': r['end']},
            'npcs': [{'start': loc(back), 'rotation': round(back.transform.rotation.yaw, 1)}]}, \
        [tp({'throttle_range': [0.85, 1.0], 'brake_chance': 0.0, 'brake_range': [0, 0], 'steer_range': [0, 0.02]})]

def row():
    sc = dict(CROSS); sc['ego_mode'] = 'reckless'; sc['ego_throttle_floor'] = 0.5; sc['row_yielder'] = 'ego'
    return sc, [tp({'throttle_range': [0.2, 0.35], 'brake_chance': 0.1, 'brake_range': [0, 0.3], 'steer_range': [0, 0.04]})]

def drift():
    for r in routes:
        a, s = adj(wp(r['start']))
        if a:
            w = a.next(4)[0]
            return {'map': 'Town03', 'ego_mode': 'drift', 'drift': {'onset': 12, 'side': s, 's_drift': 0.3},
                    'ev': {'start': r['start'], 'rotation': r['rotation'], 'end': r['end']}, 'axis_yaw': r['rotation'],
                    'npcs': [{'start': loc(w), 'rotation': round(w.transform.rotation.yaw, 1)}]}, \
                [[carla.VehicleControl(throttle=0.0, brake=1.0)] * NUM]
    return None


TESTS = [('reckless tailgater', tailgater, 'rear_end_collision'),
         ('NPC rear-ends ego', trailer, None),
         ('reckless runs yield', row, 'failure_to_yield'),
         ('ego drifts into lane', drift, 'lane_invasion')]

for name, mk, expect in TESTS:
    rep = None; res = None
    for _ in range(6):
        made = mk()
        if made is None: break
        sc, ind = made
        res = nhtsa_sim.run_one(client, sc, ind, tick=TICK, max_frames=FRAMES, record_traj=True)
        if res['isHit']:
            rep = safety_oracle.evaluate(res, sc, tick=TICK)
            break
    ok = rep and ((expect is None and not rep['is_safety_case']) or (expect and expect in rep['safety_case_types']))
    print(f"\n[{'OK' if ok else '??'}] {name}  (expect {expect})", flush=True)
    if rep is None:
        print("   no collision"); continue
    print(f"   case={res['case']} egoFault={res['egoFault']} is_safety_case={rep['is_safety_case']}", flush=True)
    if rep['violations']:
        v = rep['violations'][0]
        print(f"   VIOLATION: type={v['type']} layer={v['layer']} rule={v['violated_rule']} "
              f"ego_at_fault={v['ego_at_fault']} severity={v['severity']} "
              f"ttc_min={v['ttc_min_s']} collision_speed={v['collision_speed_mph']}mph conf={v['confidence']}", flush=True)
    else:
        r = rep['layers']['responsibility']
        print(f"   NO violation (responsibility: ego_at_fault={r['ego_at_fault']} npc_at_fault={r['npc_at_fault']})", flush=True)
