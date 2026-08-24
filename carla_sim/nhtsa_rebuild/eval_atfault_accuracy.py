import carla, json, os, sys, random, math, collections
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.append(HERE); sys.path.append(r"C:\Users\10184\AV-Fuzzer\carla_sim")
import tools, nhtsa_sim, groundtruth

random.seed(11)
TICK, FRAMES = 0.05, 400
NUM = FRAMES // 20 + 1
N_PER = 50
MAX_TRIALS = 150
OUT = os.path.join(HERE, 'results')
os.makedirs(OUT, exist_ok=True)

routes = json.load(open(os.path.join(HERE, 'verified_routes_Town03.json')))['routes']
STRAIGHT = [r for r in routes if r['spawn_index'] in (132, 198, 264)] or routes[:3]
CROSS = next(s for s in json.load(open(os.path.join(HERE, 'scenarios.json'))) if s['conflict'] == 'crossing')


def offset(route, d):
    x, y, z = route['start']; r = math.radians(route['rotation'])
    return [round(x + d * math.cos(r), 2), round(y + d * math.sin(r), 2), z]


def ev_of(route):
    return {'start': route['start'], 'rotation': route['rotation'], 'end': route['end']}


def tape(cfg):
    return tools.generate_npc_behaviors(cfg, NUM, True)


def reverser_tape(fwd_secs):
    seq = []
    for i in range(NUM):
        c = carla.VehicleControl(); c.steer = 0.0; c.brake = 0.0
        if i < fwd_secs:
            c.throttle = 0.4; c.reverse = False
        else:
            c.throttle = 0.5; c.reverse = True
        seq.append(c)
    return seq


LEAD_DECEL = {'throttle_range': [0.3, 0.5], 'brake_chance': 0.45, 'brake_range': [0.2, 0.55], 'steer_range': [0.0, 0.02]}
AGGRO = {'throttle_range': [0.85, 1.0], 'brake_chance': 0.0, 'brake_range': [0.0, 0.0], 'steer_range': [0.0, 0.02]}


def make_ego_tailgater():
    r = random.choice(STRAIGHT)
    sc = {'map': 'Town03', 'ego_mode': 'reckless', 'ego_throttle_floor': round(random.uniform(0.6, 0.85), 2),
          'ev': ev_of(r), 'npcs': [{'start': offset(r, random.uniform(12, 22)), 'rotation': r['rotation']}],
          'gt_family': 'rear_end', 'gt_role': 'ego_tailgater'}
    return sc, [tape(LEAD_DECEL)]


def make_npc_trailer():
    r = random.choice(STRAIGHT)
    sc = {'map': 'Town03', 'ego_mode': 'normal', 'ev': ev_of(r),
          'npcs': [{'start': offset(r, -random.uniform(8, 16)), 'rotation': r['rotation']}],
          'gt_family': 'rear_end', 'gt_role': 'npc_trailer'}
    return sc, [tape(AGGRO)]


def make_npc_reverser():
    r = random.choice(STRAIGHT)
    sc = {'map': 'Town03', 'ego_mode': 'normal', 'ev': ev_of(r),
          'npcs': [{'start': offset(r, random.uniform(7, 11)), 'rotation': r['rotation']}],
          'gt_family': 'rear_end', 'gt_role': 'npc_reverser'}
    return sc, [reverser_tape(random.randint(2, 4))]


def make_row_violation():
    sc = dict(CROSS)
    sc['ego_mode'] = 'reckless'; sc['ego_throttle_floor'] = round(random.uniform(0.35, 0.6), 2)
    sc['gt_family'] = 'row'; sc['gt_role'] = 'ego_runs_yield'; sc['gt_yielder'] = 'ego'
    sc['row_yielder'] = 'ego'
    lo = round(random.uniform(0.12, 0.45), 2)
    cfg = {'throttle_range': [lo, round(lo + 0.1, 2)], 'brake_chance': 0.1, 'brake_range': [0.0, 0.3], 'steer_range': [0.0, 0.04]}
    return sc, [tape(cfg)]


CONSTRUCTIONS = [('ego_tailgater', make_ego_tailgater), ('npc_trailer', make_npc_trailer),
                 ('npc_reverser', make_npc_reverser), ('row_violation', make_row_violation)]


def pred_of(case, ego_fault):
    if case == 'unknown':
        return 'abstain'
    if case.endswith('_veer'):
        return 'npc'
    return 'ego' if ego_fault else 'npc'


def metrics(records, key):
    scored = [r for r in records if r[key] in ('ego', 'npc') and r['gt'] in ('ego', 'npc')]
    tp = sum(1 for r in scored if r['gt'] == 'ego' and r[key] == 'ego')
    fp = sum(1 for r in scored if r['gt'] == 'npc' and r[key] == 'ego')
    fn = sum(1 for r in scored if r['gt'] == 'ego' and r[key] == 'npc')
    tn = sum(1 for r in scored if r['gt'] == 'npc' and r[key] == 'npc')
    acc = (tp + tn) / len(scored) if scored else 0.0
    prec = tp / (tp + fp) if (tp + fp) else float('nan')
    rec = tp / (tp + fn) if (tp + fn) else float('nan')
    f1 = 2 * prec * rec / (prec + rec) if (prec + rec) and not math.isnan(prec) and not math.isnan(rec) else float('nan')
    return {'n': len(scored), 'TP': tp, 'FP': fp, 'FN': fn, 'TN': tn,
            'accuracy': round(acc, 3), 'precision': None if math.isnan(prec) else round(prec, 3),
            'recall': None if math.isnan(rec) else round(rec, 3), 'f1': None if math.isnan(f1) else round(f1, 3)}


def main():
    client = carla.Client('localhost', 2000); client.set_timeout(60.0)
    logf = open(os.path.join(OUT, 'atfault_accuracy_records.jsonl'), 'w')
    records = []

    print("=== REPRODUCIBILITY: ego_tailgater x5 (same input) ===", flush=True)
    sc0, ind0 = make_ego_tailgater()
    reps = []
    for i in range(5):
        r = nhtsa_sim.run_one(client, sc0, ind0, tick=TICK, max_frames=FRAMES)
        reps.append((r['isHit'], r['case'], r['egoFault']))
        print(f"  rep{i}: hit={r['isHit']} case={r['case']} egoFault={r['egoFault']}", flush=True)
    stable = len(set(reps)) == 1
    print(f"  -> STABLE: {stable}", flush=True)

    for name, maker in CONSTRUCTIONS:
        got = 0; trials = 0
        while got < N_PER and trials < MAX_TRIALS:
            sc, ind = maker(); trials += 1
            try:
                res = nhtsa_sim.run_one(client, sc, ind, tick=TICK, max_frames=FRAMES, record_traj=True)
            except Exception as e:
                print(f"  [{name} err {type(e).__name__}]", flush=True); continue
            if not res['isHit']:
                continue
            got += 1
            gt, reason = groundtruth.resolve(sc, res)
            rec = {'construction': name, 'gt': gt, 'gt_reason': reason,
                   'pred_geom': pred_of(res['case_geom'], res['egoFault_geom']),
                   'pred_row': pred_of(res['case'], res['egoFault']),
                   'case_geom': res['case_geom'], 'case_row': res['case']}
            records.append(rec); logf.write(json.dumps(rec) + '\n'); logf.flush()
        print(f"[{name}] {got} collisions / {trials} trials", flush=True)
    logf.close()

    mg = metrics(records, 'pred_geom')
    mr = metrics(records, 'pred_row')
    by_con = {c: metrics([r for r in records if r['construction'] == c], 'pred_row') for c, _ in CONSTRUCTIONS}
    print("\n================ AT-FAULT ACCURACY RESULT ================", flush=True)
    print(f"labeled collisions: {len(records)}", flush=True)
    print(f"[geometry + causation]  N={mg['n']} acc={mg['accuracy']} P={mg['precision']} R={mg['recall']} F1={mg['f1']}"
          f"  (TP={mg['TP']} FP={mg['FP']} FN={mg['FN']} TN={mg['TN']})", flush=True)
    print(f"[+ right-of-way      ]  N={mr['n']} acc={mr['accuracy']} P={mr['precision']} R={mr['recall']} F1={mr['f1']}"
          f"  (TP={mr['TP']} FP={mr['FP']} FN={mr['FN']} TN={mr['TN']})", flush=True)
    print("per-construction (row-aware):", flush=True)
    for c, m in by_con.items():
        print(f"   {c:<15} n={m['n']:>3} acc={m['accuracy']}", flush=True)
    print(f"reproducible: {stable}", flush=True)
    json.dump({'labeled': len(records), 'geom': mg, 'row': mr, 'by_construction': by_con, 'reproducible': stable},
              open(os.path.join(OUT, 'atfault_accuracy_summary.json'), 'w'), indent=1)
    print("SECTION4_DONE", flush=True)


if __name__ == '__main__':
    main()
