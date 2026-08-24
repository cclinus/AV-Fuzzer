import carla, json, os, sys, math, random
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.append(HERE); sys.path.append(r"C:\Users\10184\AV-Fuzzer\carla_sim")
import tools, nhtsa_sim, safety_oracle
import four_way_stop as FWS

random.seed(19)
TICK, FRAMES, NUM = 0.05, 400, 21
N_TARGET = int(os.environ.get('N_TARGET', 50))
N_MAX = int(os.environ.get('N_MAX', 300))
CAT, DESC = 'Four-way-stop', 'all-way-stop-run'

client = carla.Client('localhost', 2000); client.set_timeout(120.0)
w = client.get_world()
if 'Town03' not in w.get_map().name:
    w = client.load_world('Town03_Opt', map_layers=carla.MapLayer.NONE)
m = w.get_map(); nhtsa_sim._cur['map'] = 'Town03'


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


sc_path = os.path.join(HERE, 'results', 'safety_cases.jsonl')
tj_path = os.path.join(HERE, 'results', 'trajectories.jsonl')
existing = [json.loads(l) for l in open(sc_path)]
cid = max((c['id'] for c in existing), default=-1) + 1
have = sum(1 for c in existing if c.get('category') == CAT)
print(f"existing Four-way-stop={have}; target total={N_TARGET}; starting id={cid}", flush=True)

out = open(sc_path, 'a'); traj_out = open(tj_path, 'a')
kept = 0; tries = 0; hits = 0; consec_fail = 0
while (have + kept) < N_TARGET and tries < N_MAX:
    sc = FWS.build(m); tries += 1
    if sc is None:
        continue
    ind = [tools.generate_npc_behaviors(n['cfg'], NUM, True) for n in sc['npcs']]
    try:
        res = nhtsa_sim.run_one(client, sc, ind, tick=TICK, max_frames=FRAMES, record_traj=True)
        consec_fail = 0
    except Exception:
        consec_fail += 1
        if consec_fail >= 12:
            print("CARLA appears down (12 consecutive failures) -- stopping for restart", flush=True)
            break
        continue
    if res['isHit']:
        hits += 1
    rep = safety_oracle.evaluate(res, sc, tick=TICK)
    if not rep['is_safety_case']:
        continue
    v = rep['violations'][0]
    rec = {'id': cid, 'category': CAT, 'scenario': DESC, 'ego_mode': sc.get('ego_mode'),
           'type': v['type'], 'violated_rule': v['violated_rule'], 'layer': v['layer'],
           'severity': v['severity'], 'ttc_min_s': v['ttc_min_s'], 'collision_speed_mph': v['collision_speed_mph'],
           'confidence': v['confidence'], 'ev_start': sc['ev']['start'], 'npc_start': sc['npcs'][0]['start'],
           **npc_ctrl_summary(ind), 'traj_sig': traj_sig(res.get('full_traj')),
           'event_seq': event_seq(res.get('full_traj'))}
    out.write(json.dumps(rec) + '\n'); out.flush()
    traj_out.write(json.dumps({'id': cid, 'category': CAT, 'scenario': DESC,
                               'full_traj': res.get('full_traj')}) + '\n'); traj_out.flush()
    cid += 1; kept += 1
    if kept % 10 == 0:
        print(f"  kept {kept}/{N_TARGET} (tries {tries}, hits {hits})", flush=True)
out.close(); traj_out.close()
print(f"\n[Four-way-stop] +{kept} this run (total now {have + kept}/{N_TARGET}) in {tries} tries "
      f"({hits} collisions)", flush=True)
print("FWSTOP_COLLECT_DONE", flush=True)
