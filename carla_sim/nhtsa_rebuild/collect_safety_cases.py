import carla, json, os, sys, random, math
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.append(HERE); sys.path.append(r"C:\Users\10184\AV-Fuzzer\carla_sim")
import tools, nhtsa_sim, safety_oracle
import run_ga2 as R2

random.seed(41)
TICK, FRAMES = 0.05, 400
NUM = FRAMES // 20 + 1
POP = 10
N_EVAL = int(os.environ.get('N_EVAL', 40))
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

REAR = {'throttle_range': [0.3, 0.5], 'brake_chance': 0.45, 'brake_range': [0.2, 0.55], 'steer_range': [0.0, 0.02]}
CROSS_CFG = {'throttle_range': [0.15, 0.4], 'brake_chance': 0.1, 'brake_range': [0.0, 0.3], 'steer_range': [0.0, 0.04]}


def rear_lead(idx):
    r = STR[idx % len(STR)]; lead = wp(r['start']).next(random.uniform(12, 18))[0]
    return {'map': 'Town03', 'ego_mode': 'reckless', 'ego_throttle_floor': 0.7,
            'ev': {'start': r['start'], 'rotation': r['rotation'], 'end': r['end']},
            'npcs': [{'start': loc(lead), 'rotation': round(lead.transform.rotation.yaw, 1), 'cfg': REAR}]}


def row_cross():
    sc = dict(CROSS); sc['ego_mode'] = 'reckless'; sc['ego_throttle_floor'] = round(random.uniform(0.4, 0.6), 2)
    sc['row_yielder'] = 'ego'; sc['npcs'] = [dict(CROSS['npcs'][0], cfg=CROSS_CFG)]
    return sc


def drift_sideswipe():
    for r in routes:
        a, s = adj(wp(r['start']))
        if a:
            w = a.next(random.uniform(3, 5))[0]
            return {'map': 'Town03', 'ego_mode': 'drift', 'drift': {'onset': 12, 'side': s, 's_drift': 0.3},
                    'ev': {'start': r['start'], 'rotation': r['rotation'], 'end': r['end']}, 'axis_yaw': r['rotation'],
                    'npcs': [{'start': loc(w), 'rotation': round(w.transform.rotation.yaw, 1),
                              'cfg': {'throttle_range': [0.0, 0.15], 'brake_chance': 0.5, 'brake_range': [0.5, 1.0], 'steer_range': [0, 0.01]}}]}
    return None


SCENS = [('Rear-end', 'lead-brakes', lambda: rear_lead(0)),
         ('Rear-end', 'highway', lambda: rear_lead(1)),
         ('Intersection', 'crossing', row_cross),
         ('Crossing-traffic', 'straight-crossing-path', row_cross),
         ('Merge', 'lane-change-sideswipe', drift_sideswipe)]


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
    thr = [c.throttle for seq in ind for c in seq]
    brk = [c.brake for seq in ind for c in seq]
    return {'npc_throttle_mean': round(sum(thr) / len(thr), 3) if thr else 0.0,
            'npc_brake_mean': round(sum(brk) / len(brk), 3) if brk else 0.0}


out = open(os.path.join(HERE, 'results', 'safety_cases.jsonl'), 'w')
traj_out = open(os.path.join(HERE, 'results', 'trajectories.jsonl'), 'w')
cid = 0
for cat, desc, builder in SCENS:
    base = builder()
    if base is None:
        continue
    pop = [[tape(n['cfg']) for n in base['npcs']] for _ in range(POP)]
    gens = max(1, N_EVAL // POP); n = 0; kept = 0
    for g in range(gens):
        fits = []
        for ind in pop:
            sc = builder()
            try:
                res = nhtsa_sim.run_one(client, sc, ind, tick=TICK, max_frames=FRAMES, record_traj=True)
            except Exception:
                fits.append(-1); continue
            n += 1
            rep = safety_oracle.evaluate(res, sc, tick=TICK)
            fits.append(100.0 if rep['is_safety_case'] else 20.0 if res['isHit'] else max(0.0, 30 - res['minDist']))
            if rep['is_safety_case']:
                v = rep['violations'][0]
                rec = {'id': cid, 'category': cat, 'scenario': desc, 'ego_mode': sc.get('ego_mode'),
                       'type': v['type'], 'violated_rule': v['violated_rule'], 'layer': v['layer'],
                       'severity': v['severity'], 'ttc_min_s': v['ttc_min_s'], 'collision_speed_mph': v['collision_speed_mph'],
                       'confidence': v['confidence'], 'ev_start': sc['ev']['start'], 'npc_start': sc['npcs'][0]['start'],
                       **npc_ctrl_summary(ind), 'traj_sig': traj_sig(res.get('full_traj')),
                       'event_seq': event_seq(res.get('full_traj'))}
                out.write(json.dumps(rec) + '\n'); out.flush()
                traj_out.write(json.dumps({'id': cid, 'category': cat, 'scenario': desc,
                                           'full_traj': res.get('full_traj')}) + '\n'); traj_out.flush()
                cid += 1; kept += 1
            if n >= N_EVAL:
                break
        if n >= N_EVAL:
            break
        best = max(range(len(pop)), key=lambda i: fits[i]); newp = [pop[best]]
        while len(newp) < POP:
            p1 = R2.tournament(pop, fits); p2 = R2.tournament(pop, fits)
            child = R2.crossover(p1, p2) if random.random() < 0.8 else p1
            newp.append(R2.mutate(child, base))
        pop = newp
    print(f"[{cat}/{desc}] {kept} safety cases from {n} runs", flush=True)
out.close(); traj_out.close()
print(f"TOTAL safety cases collected: {cid}", flush=True)
print("COLLECT_DONE", flush=True)
