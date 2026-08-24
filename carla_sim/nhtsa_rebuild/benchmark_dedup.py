import carla, json, os, sys, random, math
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.append(HERE); sys.path.append(r"C:\Users\10184\AV-Fuzzer\carla_sim")
import tools, nhtsa_sim, safety_oracle

random.seed(7)
TICK, FRAMES = 0.05, 400
NUM = FRAMES // 20 + 1
R = int(os.environ.get('REPS', 30))
client = carla.Client('localhost', 2000); client.set_timeout(60.0)
world = client.load_world('Town03'); m = world.get_map(); nhtsa_sim._cur['map'] = 'Town03'
routes = json.load(open(os.path.join(HERE, 'verified_routes_Town03.json')))['routes']
STR = [r for r in routes if r['spawn_index'] in (132, 198, 264)]
CROSS = next(s for s in json.load(open(os.path.join(HERE, 'scenarios.json'))) if s['conflict'] == 'crossing')


def wp(xyz): return m.get_waypoint(carla.Location(x=xyz[0], y=xyz[1], z=xyz[2]))
def loc(w): l = w.transform.location; return [round(l.x, 2), round(l.y, 2), round(l.z + 0.3, 2)]
def tape(cfg): return tools.generate_npc_behaviors(cfg, NUM, True)
def adj(ew):
    for g, s in ((ew.get_left_lane, -1.0), (ew.get_right_lane, 1.0)):
        a = g()
        if a and a.lane_type == carla.LaneType.Driving and a.lane_id * ew.lane_id > 0:
            return a, s
    return None, None
def traj_sig(ft, k=8):
    if not ft:
        return []
    idx = [int(i * (len(ft) - 1) / (k - 1)) for i in range(k)]
    x0, y0 = ft[0][1], ft[0][2]
    return [round(ft[i][1] - x0, 1) for i in idx] + [round(ft[i][2] - y0, 1) for i in idx]


def event_seq(ft, step=10):
    toks = []; prev = None
    for k in range(0, len(ft or []), step):
        row = ft[k]; eyaw, evx, evy = row[3], row[4], row[5]; ve = math.hypot(evx, evy)
        sp = 'S0' if ve < 2 else 'S1' if ve < 6 else 'S2' if ve < 10 else 'S3' if ve < 15 else 'S4'
        st = 'x'
        if prev is not None:
            dy = ((eyaw - prev + 180) % 360) - 180
            st = 'L' if dy < -3 else 'R' if dy > 3 else 's'
        prev = eyaw
        n = row[6]; rel = 'na'
        if n:
            r = math.radians(eyaw); ux, uy = math.cos(r), math.sin(r)
            lon = ux * (n[0] - row[1]) + uy * (n[1] - row[2])
            dist = 'nr' if abs(lon) < 8 else 'md' if abs(lon) < 20 else 'fr'
            clos = (ux * evx + uy * evy) - (ux * n[3] + uy * n[4])
            rel = dist + ('C' if clos > 0.5 else 'O' if clos < -0.5 else 'e')
        toks.append(sp + st + rel)
    return toks


def npc_ctrl_summary(ind):
    thr = [c.throttle for seq in ind for c in seq]; brk = [c.brake for seq in ind for c in seq]
    return {'npc_throttle_mean': round(sum(thr) / len(thr), 3) if thr else 0.0,
            'npc_brake_mean': round(sum(brk) / len(brk), 3) if brk else 0.0}


def rearA():
    r = STR[0]; lead = wp(r['start']).next(15)[0]
    return {'map': 'Town03', 'ego_mode': 'reckless', 'ego_throttle_floor': 0.7,
            'ev': {'start': r['start'], 'rotation': r['rotation'], 'end': r['end']},
            'npcs': [{'start': loc(lead), 'rotation': round(lead.transform.rotation.yaw, 1),
                      'cfg': {'throttle_range': [0.3, 0.5], 'brake_chance': 0.45, 'brake_range': [0.2, 0.55], 'steer_range': [0, 0.02]}}]}


def rearB():
    r = STR[1]; lead = wp(r['start']).next(18)[0]
    return {'map': 'Town03', 'ego_mode': 'reckless', 'ego_throttle_floor': 0.75,
            'ev': {'start': r['start'], 'rotation': r['rotation'], 'end': r['end']},
            'npcs': [{'start': loc(lead), 'rotation': round(lead.transform.rotation.yaw, 1),
                      'cfg': {'throttle_range': [0.3, 0.5], 'brake_chance': 0.45, 'brake_range': [0.2, 0.55], 'steer_range': [0, 0.02]}}]}


def crossg(lo, hi):
    sc = dict(CROSS); sc['ego_mode'] = 'reckless'; sc['ego_throttle_floor'] = 0.5; sc['row_yielder'] = 'ego'
    sc['npcs'] = [dict(CROSS['npcs'][0], cfg={'throttle_range': [lo, hi], 'brake_chance': 0.1, 'brake_range': [0, 0.3], 'steer_range': [0, 0.04]})]
    return sc


def mergeg():
    for r in routes:
        a, s = adj(wp(r['start']))
        if a:
            w = a.next(4)[0]
            return {'map': 'Town03', 'ego_mode': 'drift', 'drift': {'onset': 12, 'side': s, 's_drift': 0.3},
                    'ev': {'start': r['start'], 'rotation': r['rotation'], 'end': r['end']}, 'axis_yaw': r['rotation'],
                    'npcs': [{'start': loc(w), 'rotation': round(w.transform.rotation.yaw, 1),
                              'cfg': {'throttle_range': [0.0, 0.15], 'brake_chance': 0.5, 'brake_range': [0.5, 1.0], 'steer_range': [0, 0.01]}}]}
    return None


GROUPS = [('rearA', 'Rear-end', 'lead-brakes-A', rearA, 30), ('rearB', 'Rear-end', 'lead-brakes-B', rearB, 30),
          ('crossSlow', 'Intersection', 'crossing-slow', lambda: crossg(0.12, 0.20), 20),
          ('crossMid', 'Intersection', 'crossing-mid', lambda: crossg(0.26, 0.36), 20),
          ('merge', 'Merge', 'sideswipe', mergeg, 30)]

out = open(os.path.join(HERE, 'results', 'benchmark_cases.jsonl'), 'w')
cid = 0
for gi, (gname, cat, desc, builder, cnt) in enumerate(GROUPS):
    kept = 0; tries = 0
    while kept < cnt and tries < cnt * 5:
        sc = builder(); tries += 1
        if sc is None:
            break
        ind = [tape(n['cfg']) for n in sc['npcs']]
        try:
            res = nhtsa_sim.run_one(client, sc, ind, tick=TICK, max_frames=FRAMES, record_traj=True)
        except Exception:
            continue
        rep = safety_oracle.evaluate(res, sc, tick=TICK)
        if not rep['is_safety_case']:
            continue
        v = rep['violations'][0]
        rec = {'id': cid, 'gt_group': gi, 'gt_name': gname, 'category': cat, 'scenario': desc,
               'type': v['type'], 'violated_rule': v['violated_rule'], 'layer': v['layer'], 'severity': v['severity'],
               'ttc_min_s': v['ttc_min_s'], 'collision_speed_mph': v['collision_speed_mph'], 'confidence': v['confidence'],
               'ev_start': sc['ev']['start'], 'npc_start': sc['npcs'][0]['start'], **npc_ctrl_summary(ind),
               'traj_sig': traj_sig(res.get('full_traj')), 'event_seq': event_seq(res.get('full_traj'))}
        out.write(json.dumps(rec) + '\n'); out.flush(); cid += 1; kept += 1
    print(f"[group {gi} {gname}] {kept} cases", flush=True)
out.close()
print(f"TOTAL benchmark cases: {cid}", flush=True)
print("BENCHMARK_DONE", flush=True)
