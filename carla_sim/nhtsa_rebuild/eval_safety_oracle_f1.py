import os, sys, json, math, random, collections
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.append(HERE); sys.path.append(r"C:\Users\10184\AV-Fuzzer\carla_sim")
import nhtsa_sim, groundtruth, safety_oracle
import eval_atfault_f1_groups as E6

random.seed(23)
client = E6.client
TICK, FRAMES, NUM = E6.TICK, E6.FRAMES, E6.NUM
N_PER, MAX_TRIALS = 50, 380

CFG_CONT = {'oracles': [{'name': 'safe_distance', 'mode': 'both'}, {'name': 'yield_row'},
                        {'name': 'lane_invasion', 'detection': 'both'}, {'name': 'red_light', 'enabled': False}]}
CFG_COLL = {'oracles': [{'name': 'safe_distance', 'mode': 'at_collision'}, {'name': 'yield_row'},
                        {'name': 'lane_invasion', 'detection': 'at_collision'}, {'name': 'red_light', 'enabled': False}]}


def pred(is_sc):
    return 'ego' if is_sc else 'npc'


def collect(makers, want_gt, n, tag):
    recs = []; got = 0; trials = 0
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
        gt, _ = groundtruth.resolve(sc, res)
        if gt != want_gt:
            continue
        got += 1
        cont = safety_oracle.evaluate(res, sc, CFG_CONT, tick=TICK)
        coll = safety_oracle.evaluate(res, sc, CFG_COLL, tick=TICK)
        recs.append({'group': tag, 'gt': gt, 'pred_cont': pred(cont['is_safety_case']),
                     'pred_coll': pred(coll['is_safety_case']), 'types_cont': cont['safety_case_types'],
                     'case': res['case']})
    print(f"  [{tag}] {got} (GT {want_gt}) in {trials} trials", flush=True)
    return recs


def f1(recs, key):
    tp = sum(1 for r in recs if r['gt'] == 'ego' and r[key] == 'ego')
    fp = sum(1 for r in recs if r['gt'] == 'npc' and r[key] == 'ego')
    fn = sum(1 for r in recs if r['gt'] == 'ego' and r[key] == 'npc')
    tn = sum(1 for r in recs if r['gt'] == 'npc' and r[key] == 'npc')
    p = tp / (tp + fp) if (tp + fp) else float('nan')
    rc = tp / (tp + fn) if (tp + fn) else float('nan')
    ff = 2 * p * rc / (p + rc) if (p + rc) and not math.isnan(p) and not math.isnan(rc) else float('nan')
    acc = (tp + tn) / len(recs) if recs else 0.0
    return {'N': len(recs), 'TP': tp, 'FP': fp, 'FN': fn, 'TN': tn, 'accuracy': round(acc, 3),
            'precision': None if math.isnan(p) else round(p, 3), 'recall': None if math.isnan(rc) else round(rc, 3),
            'f1': None if math.isnan(ff) else round(ff, 3)}


def main():
    summary = {}
    logf = open(os.path.join(HERE, 'results', 'safety_oracle_f1_records.jsonl'), 'w')
    for gname, mk in E6.GROUPS.items():
        print(f"================ {gname} ================", flush=True)
        recs = collect(mk['ego'], 'ego', N_PER, gname) + collect(mk['npc'], 'npc', N_PER, gname)
        for r in recs:
            logf.write(json.dumps(r) + '\n')
        summary[gname] = {'continuous': f1(recs, 'pred_cont'), 'at_collision': f1(recs, 'pred_coll')}
        print(f"  -> continuous={summary[gname]['continuous']}", flush=True)
        print(f"  -> at_collision={summary[gname]['at_collision']}", flush=True)
    logf.close()
    print("\n================ SAFETY-ORACLE F1 ================", flush=True)
    for g, mm in summary.items():
        c = mm['continuous']; a = mm['at_collision']
        print(f"  {g:<20} N={c['N']}  continuous F1={c['f1']} (P={c['precision']} R={c['recall']} FP={c['FP']} FN={c['FN']})"
              f"  | at_collision F1={a['f1']}", flush=True)
    json.dump(summary, open(os.path.join(HERE, 'results', 'safety_oracle_f1_summary.json'), 'w'), indent=1)
    print("SECTION6_ORACLE_DONE", flush=True)


if __name__ == '__main__':
    main()
