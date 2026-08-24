import carla, json, os, sys, random, math, collections
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.append(HERE); sys.path.append(r"C:\Users\10184\AV-Fuzzer\carla_sim")
import tools, nhtsa_sim, safety_oracle
import run_ga2 as R2

random.seed(31)
TICK, FRAMES = 0.05, 400
NUM = FRAMES // 20 + 1
POP = 10
N_EVAL = int(os.environ.get('N_EVAL', 50))
client = carla.Client('localhost', 2000); client.set_timeout(60.0)
world = client.load_world('Town03'); m = world.get_map(); nhtsa_sim._cur['map'] = 'Town03'
routes = json.load(open(os.path.join(HERE, 'verified_routes_Town03.json')))['routes']
STR = [r for r in routes if r['spawn_index'] in (132, 198, 264)]
CROSS = next(s for s in json.load(open(os.path.join(HERE, 'scenarios.json'))) if s['conflict'] == 'crossing')


def wp(xyz): return m.get_waypoint(carla.Location(x=xyz[0], y=xyz[1], z=xyz[2]))
def loc(w): l = w.transform.location; return [round(l.x, 2), round(l.y, 2), round(l.z + 0.3, 2)]
def tape(cfg): return tools.generate_npc_behaviors(cfg, NUM, True)
def adj(ew):
    a = ew.get_left_lane()
    if a and a.lane_type == carla.LaneType.Driving and a.lane_id * ew.lane_id > 0: return a, -1.0
    a = ew.get_right_lane()
    if a and a.lane_type == carla.LaneType.Driving and a.lane_id * ew.lane_id > 0: return a, 1.0
    return None, None


def turning_routes():
    out = []
    for sp in m.get_spawn_points():
        w = m.get_waypoint(sp.location); cur = w
        for _ in range(18):
            nx = cur.next(4.0)
            if not nx: break
            cur = nx[0]
            if cur.is_junction:
                junc = cur.get_junction()
                eyaw = w.transform.rotation.yaw
                for a, b in junc.get_waypoints(carla.LaneType.Driving):
                    d = abs(((a.transform.rotation.yaw - eyaw + 90) % 180) - 90)
                    if 40 < d < 140:
                        ex = b.next(8.0)
                        if ex:
                            out.append({'start': loc(w), 'rotation': round(eyaw, 1), 'end': loc(ex[0]),
                                        'jyaw': round(a.transform.rotation.yaw, 1)})
                        break
                break
        if len(out) >= 6:
            break
    return out


TURNS = turning_routes()
print(f"turning routes found: {len(TURNS)}", flush=True)

REAR = {'throttle_range': [0.3, 0.5], 'brake_chance': 0.45, 'brake_range': [0.2, 0.55], 'steer_range': [0.0, 0.02]}
CROSS_CFG = {'throttle_range': [0.15, 0.4], 'brake_chance': 0.1, 'brake_range': [0.0, 0.3], 'steer_range': [0.0, 0.04]}
ONC = {'throttle_range': [0.4, 0.7], 'brake_chance': 0.05, 'brake_range': [0.0, 0.2], 'steer_range': [0.0, 0.02]}


def rear_lead(idx):
    r = STR[idx % len(STR)]; lead = wp(r['start']).next(15)[0]
    return {'map': 'Town03', 'ego_mode': 'reckless', 'ego_throttle_floor': 0.7,
            'ev': {'start': r['start'], 'rotation': r['rotation'], 'end': r['end']},
            'npcs': [{'start': loc(lead), 'rotation': round(lead.transform.rotation.yaw, 1), 'cfg': REAR}]}


def row_cross(row_yielder='ego'):
    sc = dict(CROSS); sc['ego_mode'] = 'reckless'; sc['ego_throttle_floor'] = 0.5; sc['row_yielder'] = row_yielder
    sc['npcs'] = [dict(CROSS['npcs'][0], cfg=CROSS_CFG)]
    return sc


def turn_across(oncoming=True):
    if not TURNS:
        return None
    t = random.choice(TURNS)
    ex = wp(t['end'])
    npc_w = ex.previous(20.0)
    npc = {'start': loc(npc_w[0]) if npc_w else t['end'], 'rotation': t['jyaw'], 'cfg': ONC}
    return {'map': 'Town03', 'ego_mode': 'reckless', 'ego_throttle_floor': 0.45, 'row_yielder': 'ego',
            'ev': {'start': t['start'], 'rotation': t['rotation'], 'end': t['end']}, 'npcs': [npc]}


def drift_sideswipe():
    for r in routes:
        a, s = adj(wp(r['start']))
        if a:
            w = a.next(4)[0]
            return {'map': 'Town03', 'ego_mode': 'drift', 'drift': {'onset': 12, 'side': s, 's_drift': 0.3},
                    'ev': {'start': r['start'], 'rotation': r['rotation'], 'end': r['end']}, 'axis_yaw': r['rotation'],
                    'npcs': [{'start': loc(w), 'rotation': round(w.transform.rotation.yaw, 1),
                              'cfg': {'throttle_range': [0.0, 0.15], 'brake_chance': 0.5, 'brake_range': [0.5, 1.0], 'steer_range': [0, 0.01]}}]}
    return None


def drift_oncoming():
    for r in routes:
        ew = wp(r['start']); opp = ew.get_left_lane()
        side = -1.0
        if not opp or opp.lane_type != carla.LaneType.Driving:
            opp = ew.get_right_lane(); side = 1.0
        if not opp or opp.lane_type != carla.LaneType.Driving:
            continue
        oncoming = opp.lane_id * ew.lane_id < 0
        ahead = opp.previous(30) if oncoming else opp.next(30)
        if not ahead:
            continue
        return {'map': 'Town03', 'ego_mode': 'drift', 'drift': {'onset': 20, 'side': side, 's_drift': 0.28},
                'ev': {'start': r['start'], 'rotation': r['rotation'], 'end': r['end']}, 'axis_yaw': r['rotation'],
                'npcs': [{'start': loc(ahead[0]), 'rotation': round(ahead[0].transform.rotation.yaw, 1), 'cfg': ONC}]}
    return None


SCENARIOS = [
    ('Rear-end', 'Lead vehicle brakes suddenly; ego rear-ends the lead vehicle', lambda: rear_lead(0)),
    ('Rear-end', 'Highway rear-end scenario', lambda: rear_lead(1)),
    ('Intersection', 'Turning conflict at an intersection', lambda: turn_across()),
    ('Intersection', 'Crossing conflict at an intersection', lambda: row_cross()),
    ('Intersection', 'Urban intersection scenario', lambda: row_cross()),
    ('Left-turn', 'Left turn across oncoming traffic', lambda: turn_across(oncoming=True)),
    ('Left-turn', 'Turning into the opposing lane', lambda: drift_oncoming()),
    ('Merge', 'Lane-change / merge sideswipe conflict', lambda: drift_sideswipe()),
    ('Crossing traffic', 'Straight crossing-path conflict', lambda: row_cross()),
    ('Opposing direction', 'Head-on conflict with opposing traffic', lambda: drift_oncoming()),
]


def make_ind(sc):
    return [tape(n.get('cfg', CROSS_CFG)) for n in sc['npcs']]


def ega_fitness(res):
    rep = safety_oracle.evaluate(res, res.get('_sc', {}), tick=TICK)
    if rep['is_safety_case']:
        return 100.0
    if res['isHit']:
        return 20.0
    return max(0.0, 30.0 - res['minDist'])


def run_scenario(builder):
    base = builder()
    if base is None:
        return {'crashes': 0, 'ego_at_fault': 0, 'note': 'not-buildable'}
    pop = [make_ind(base) for _ in range(POP)]
    gens = max(1, N_EVAL // POP)
    crashes = 0; egofault = 0; n = 0
    for g in range(gens):
        fits = []
        for ind in pop:
            sc = builder()
            try:
                res = nhtsa_sim.run_one(client, sc, ind, tick=TICK, max_frames=FRAMES, record_traj=True)
            except Exception:
                fits.append(-1); continue
            n += 1
            res['_sc'] = sc
            rep = safety_oracle.evaluate(res, sc, tick=TICK)
            fits.append(ega_fitness(res))
            if res['isHit']:
                crashes += 1
            if rep['is_safety_case']:
                egofault += 1
            if n >= N_EVAL:
                break
        if n >= N_EVAL:
            break
        best = max(range(len(pop)), key=lambda i: fits[i])
        newp = [pop[best]]
        while len(newp) < POP:
            p1 = R2.tournament(pop, fits); p2 = R2.tournament(pop, fits)
            child = R2.crossover(p1, p2) if random.random() < 0.8 else p1
            newp.append(R2.mutate(child, base))
        pop = newp
    return {'crashes': crashes, 'ego_at_fault': egofault, 'runs': n}


rows = []
for cat, desc, builder in SCENARIOS:
    r = run_scenario(builder)
    rows.append((cat, desc, r))
    print(f"[{cat:<18}] {desc[:40]:<40} runs={r.get('runs', 0):>3} crashes={r.get('crashes', 0):>3} "
          f"EGO-AT-FAULT={r.get('ego_at_fault', 0):>3} {r.get('note', '')}", flush=True)

print("\n================ EGO-AT-FAULT PER SCENARIO ================", flush=True)
tot = 0
for cat, desc, r in rows:
    ef = r.get('ego_at_fault', 0); tot += ef
    print(f"  {cat:<18} | {desc[:38]:<38} | Runs={N_EVAL} | Ego-at-fault={ef}", flush=True)
print(f"  TOTAL Ego-at-fault = {tot}", flush=True)
json.dump([{'category': c, 'desc': d, **r} for c, d, r in rows],
          open(os.path.join(HERE, 'results', 'atfault_scenario_campaign.json'), 'w'), indent=1)
print("SECTION5_CAMPAIGN_DONE", flush=True)
