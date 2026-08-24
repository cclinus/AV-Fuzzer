import carla, json, os, sys, random, math, collections
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.append(HERE); sys.path.append(r"C:\Users\10184\AV-Fuzzer\carla_sim")
import tools, nhtsa_sim, groundtruth

random.seed(13)
TICK, FRAMES = 0.05, 400
NUM = FRAMES // 20 + 1
N_PER = 50
MAX_TRIALS = 160
OUT = os.path.join(HERE, 'results')
os.makedirs(OUT, exist_ok=True)

client = carla.Client('localhost', 2000); client.set_timeout(60.0)
world = client.load_world('Town03'); m = world.get_map(); nhtsa_sim._cur['map'] = 'Town03'
routes = json.load(open(os.path.join(HERE, 'verified_routes_Town03.json')))['routes']
STRAIGHT = [r for r in routes if r['spawn_index'] in (132, 198, 264)] or routes[:3]
CROSS = next(s for s in json.load(open(os.path.join(HERE, 'scenarios.json'))) if s['conflict'] == 'crossing')


def wp_of(xyz):
    return m.get_waypoint(carla.Location(x=xyz[0], y=xyz[1], z=xyz[2]), project_to_road=True, lane_type=carla.LaneType.Driving)


def loc(wp, dz=0.3):
    l = wp.transform.location
    return [round(l.x, 2), round(l.y, 2), round(l.z + dz, 2)]


def ev_of(r):
    return {'start': r['start'], 'rotation': r['rotation'], 'end': r['end']}


def tape(cfg):
    return tools.generate_npc_behaviors(cfg, NUM, True)


def reverser_tape(fwd):
    seq = []
    for i in range(NUM):
        c = carla.VehicleControl(); c.steer = 0.0; c.brake = 0.0
        c.throttle = 0.4 if i < fwd else 0.5; c.reverse = i >= fwd
        seq.append(c)
    return seq


def pinned_tape(steer):
    seq = []
    for i in range(NUM):
        c = carla.VehicleControl(); c.throttle = random.uniform(0.4, 0.6); c.brake = 0.0; c.steer = steer
        seq.append(c)
    return seq


def adjacent_same_dir(ew):
    a = ew.get_left_lane()
    if a and a.lane_type == carla.LaneType.Driving and a.lane_id * ew.lane_id > 0:
        return a, -1.0
    a = ew.get_right_lane()
    if a and a.lane_type == carla.LaneType.Driving and a.lane_id * ew.lane_id > 0:
        return a, 1.0
    return None, None


LANE_ROUTES = []
for _r in routes:
    _a, _s = adjacent_same_dir(wp_of(_r['start']))
    if _a:
        LANE_ROUTES.append((_r, _a, _s))
print(f"lane-invasion usable routes (same-dir adjacent lane): {len(LANE_ROUTES)}", flush=True)


def ego_tailgater():
    r = random.choice(STRAIGHT); lead = wp_of(r['start']).next(random.uniform(12, 20))[0]
    return {'map': 'Town03', 'ego_mode': 'reckless', 'ego_throttle_floor': round(random.uniform(0.6, 0.85), 2),
            'ev': ev_of(r), 'npcs': [{'start': loc(lead), 'rotation': round(lead.transform.rotation.yaw, 1)}],
            'gt_family': 'rear_end', 'gt_role': 'ego_tailgater'}, \
        [tape({'throttle_range': [0.3, 0.5], 'brake_chance': 0.45, 'brake_range': [0.2, 0.55], 'steer_range': [0.0, 0.02]})]


def npc_trailer():
    r = random.choice(STRAIGHT); back = wp_of(r['start']).previous(random.uniform(8, 16))
    if not back:
        return None
    w = back[0]
    return {'map': 'Town03', 'ego_mode': 'normal', 'ev': ev_of(r),
            'npcs': [{'start': loc(w), 'rotation': round(w.transform.rotation.yaw, 1)}],
            'gt_family': 'rear_end', 'gt_role': 'npc_trailer'}, \
        [tape({'throttle_range': [0.85, 1.0], 'brake_chance': 0.0, 'brake_range': [0.0, 0.0], 'steer_range': [0.0, 0.02]})]


def npc_reverser():
    r = random.choice(STRAIGHT); lead = wp_of(r['start']).next(random.uniform(7, 11))[0]
    return {'map': 'Town03', 'ego_mode': 'normal', 'ev': ev_of(r),
            'npcs': [{'start': loc(lead), 'rotation': round(lead.transform.rotation.yaw, 1)}],
            'gt_family': 'rear_end', 'gt_role': 'npc_reverser'}, [reverser_tape(random.randint(2, 4))]


def _row(yielder):
    sc = dict(CROSS); sc['ego_mode'] = 'reckless'; sc['ego_throttle_floor'] = round(random.uniform(0.35, 0.6), 2)
    sc['gt_family'] = 'row'; sc['gt_yielder'] = yielder; sc['row_yielder'] = yielder
    lo = round(random.uniform(0.12, 0.45), 2)
    return sc, [tape({'throttle_range': [lo, round(lo + 0.1, 2)], 'brake_chance': 0.1, 'brake_range': [0.0, 0.3], 'steer_range': [0.0, 0.04]})]


def row_ego():
    return _row('ego')


def row_npc():
    return _row('npc')


def lane_ego_drift():
    r, adj, side = random.choice(LANE_ROUTES)
    w = adj.next(random.uniform(3, 6))[0]
    return {'map': 'Town03', 'ego_mode': 'drift', 'drift': {'onset': 12, 'side': side, 's_drift': round(random.uniform(0.28, 0.35), 3)},
            'ev': ev_of(r), 'axis_yaw': r['rotation'], 'gt_family': 'lane', 'gt_role': 'ego_drifter',
            'npcs': [{'start': loc(w), 'rotation': round(w.transform.rotation.yaw, 1)}]}, \
        [tape({'throttle_range': [0.0, 0.15], 'brake_chance': 0.5, 'brake_range': [0.5, 1.0], 'steer_range': [0.0, 0.01]})]


def lane_npc_drift():
    r, adj, side = random.choice(LANE_ROUTES)
    w = adj.next(random.uniform(2.5, 5))[0]
    return {'map': 'Town03', 'ego_mode': 'normal', 'ev': ev_of(r), 'axis_yaw': r['rotation'],
            'gt_family': 'lane', 'gt_role': 'npc_drifter',
            'npcs': [{'start': loc(w), 'rotation': round(w.transform.rotation.yaw, 1)}]}, \
        [pinned_tape(-side * random.uniform(0.3, 0.45))]


GROUPS = {
    'rear_end_following': {'ego': [ego_tailgater], 'npc': [npc_trailer, npc_reverser]},
    'intersection_row':   {'ego': [row_ego], 'npc': [row_npc]},
    'lane_invasion':      {'ego': [lane_ego_drift], 'npc': [lane_npc_drift]},
}


def pred_of(case, ego_fault):
    if case == 'unknown':
        return 'abstain'
    if case.endswith('_veer'):
        return 'npc'
    return 'ego' if ego_fault else 'npc'


def collect(makers, want_gt, n, tag):
    got = 0; trials = 0; recs = []
    while got < n and trials < MAX_TRIALS:
        made = random.choice(makers)(); trials += 1
        if made is None:
            continue
        sc, ind = made
        try:
            res = nhtsa_sim.run_one(client, sc, ind, tick=TICK, max_frames=FRAMES, record_traj=True)
        except Exception as e:
            print(f"    [{tag} err {type(e).__name__}]", flush=True); continue
        if not res['isHit']:
            continue
        gt, reason = groundtruth.resolve(sc, res)
        if gt != want_gt:
            continue
        got += 1
        recs.append({'group': tag, 'gt': gt, 'pred': pred_of(res['case'], res['egoFault']), 'case': res['case']})
    print(f"  [{tag}] collected {got} (GT {want_gt}) in {trials} trials", flush=True)
    return recs


def f1(recs):
    s = [r for r in recs if r['pred'] in ('ego', 'npc')]
    tp = sum(1 for r in s if r['gt'] == 'ego' and r['pred'] == 'ego')
    fp = sum(1 for r in s if r['gt'] == 'npc' and r['pred'] == 'ego')
    fn = sum(1 for r in s if r['gt'] == 'ego' and r['pred'] == 'npc')
    tn = sum(1 for r in s if r['gt'] == 'npc' and r['pred'] == 'npc')
    acc = (tp + tn) / len(s) if s else 0.0
    p = tp / (tp + fp) if (tp + fp) else float('nan')
    rc = tp / (tp + fn) if (tp + fn) else float('nan')
    ff = 2 * p * rc / (p + rc) if (p + rc) and not math.isnan(p) and not math.isnan(rc) else float('nan')
    return {'n': len(s), 'TP': tp, 'FP': fp, 'FN': fn, 'TN': tn, 'accuracy': round(acc, 3),
            'precision': None if math.isnan(p) else round(p, 3), 'recall': None if math.isnan(rc) else round(rc, 3),
            'f1': None if math.isnan(ff) else round(ff, 3)}


def main():
    only = sys.argv[1] if len(sys.argv) > 1 else None
    groups = {only: GROUPS[only]} if only else GROUPS
    sfile = os.path.join(OUT, 'atfault_f1_groups_summary.json')
    summary = json.load(open(sfile)) if (only and os.path.exists(sfile)) else {}
    logf = open(os.path.join(OUT, 'atfault_f1_groups_records.jsonl'), 'a' if only else 'w')
    for gname, mk in groups.items():
        print(f"================ {gname} ================", flush=True)
        recs = collect(mk['ego'], 'ego', N_PER, gname + ':ego') + collect(mk['npc'], 'npc', N_PER, gname + ':npc')
        for r in recs:
            logf.write(json.dumps(r) + '\n')
        summary[gname] = f1(recs)
        print(f"  -> {gname}: {summary[gname]}", flush=True)
    logf.close()
    print("\n================ AT-FAULT F1 PER GROUP ================", flush=True)
    for g, mm in summary.items():
        print(f"  {g:<20} N={mm['n']:>3} TP={mm['TP']} FP={mm['FP']} FN={mm['FN']} TN={mm['TN']} "
              f"acc={mm['accuracy']} P={mm['precision']} R={mm['recall']} F1={mm['f1']}", flush=True)
    json.dump(summary, open(os.path.join(OUT, 'atfault_f1_groups_summary.json'), 'w'), indent=1)
    print("SECTION6_DONE", flush=True)


if __name__ == '__main__':
    main()
