import carla, os, sys, math, random
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.append(HERE); sys.path.append(r"C:\Users\10184\AV-Fuzzer\carla_sim")
import tools, nhtsa_sim, safety_oracle

FAST = {'throttle_range': [0.45, 0.7], 'brake_chance': 0.05, 'brake_range': [0.0, 0.2], 'steer_range': [0.0, 0.02]}


def loc(w):
    l = w.transform.location
    return [round(l.x, 2), round(l.y, 2), round(l.z + 0.3, 2)]


def angdiff(a, b):
    return abs(((a - b + 180) % 360) - 180)


def perp(a, b):
    return 60 < angdiff(a, b) < 120


def stop_junctions(m):
    out = {}
    try:
        stops = m.get_all_landmarks_of_type('206')
    except Exception:
        stops = []
    for lm in stops:
        cur = m.get_waypoint(lm.transform.location)
        if cur is None:
            continue
        for _ in range(15):
            nx = cur.next(3.0)
            if not nx:
                break
            cur = nx[0]
            if cur.is_junction:
                out.setdefault(cur.get_junction().id, cur.get_junction())
                break
    return out, len(stops)


def four_way_junctions(m):
    js = {}
    for wp in m.generate_waypoints(5.0):
        if wp.is_junction:
            js[wp.get_junction().id] = wp.get_junction()
    fw = []
    for j in js.values():
        yaws = set(round(a.transform.rotation.yaw / 45) * 45 % 360 for a, b in j.get_waypoints(carla.LaneType.Driving))
        if len(yaws) >= 4:
            fw.append(j)
    return fw


def build_from_junction(m, j, ego_approach=None, npc_approach=None):
    ego_approach = ego_approach or random.uniform(16, 24)
    npc_approach = npc_approach or random.uniform(12, 20)
    pairs = j.get_waypoints(carla.LaneType.Driving)
    straights = [(a, b) for a, b in pairs if angdiff(a.transform.rotation.yaw, b.transform.rotation.yaw) < 20]
    random.shuffle(straights)
    for a, b in straights:
        eprev = a.previous(ego_approach); enext = b.next(10.0)
        if not eprev or not enext:
            continue
        eyaw = a.transform.rotation.yaw
        crosses = [(c, d) for c, d in pairs if perp(c.transform.rotation.yaw, eyaw)]
        random.shuffle(crosses)
        for c, d in crosses:
            nprev = c.previous(npc_approach)
            if not nprev:
                continue
            es, ns = eprev[0], nprev[0]
            if es.transform.location.distance(ns.transform.location) < 6:
                continue
            return {'map': 'Town03', 'ego_mode': 'reckless', 'ego_throttle_floor': 0.55, 'row_yielder': 'ego',
                    'scenario': 'four-way-stop',
                    'ev': {'start': loc(es), 'rotation': round(es.transform.rotation.yaw, 1), 'end': loc(enext[0])},
                    'npcs': [{'start': loc(ns), 'rotation': round(ns.transform.rotation.yaw, 1), 'cfg': FAST}],
                    '_junction': j.id}
    return None


_CACHE = {'juncs': None, 'stop_controlled': None}


def _candidates(m):
    if _CACHE['juncs'] is None:
        sj, nstop = stop_junctions(m)
        if sj:
            _CACHE['juncs'] = list(sj.values()); _CACHE['stop_controlled'] = True
        else:
            _CACHE['juncs'] = four_way_junctions(m); _CACHE['stop_controlled'] = False
    return _CACHE['juncs']


def build(m):
    cands = _candidates(m)
    random.shuffle(cands)
    for j in cands:
        sc = build_from_junction(m, j)
        if sc:
            return sc
    return None


if __name__ == '__main__':
    random.seed(7)
    c = carla.Client('localhost', 2000); c.set_timeout(120.0)
    w = c.get_world()
    if 'Town03' not in w.get_map().name:
        w = c.load_world('Town03')
    m = w.get_map(); nhtsa_sim._cur['map'] = 'Town03'
    sj, nstop = stop_junctions(m); fw = four_way_junctions(m)
    print(f"STOP signs in Town03: {nstop} | stop-controlled junctions: {len(sj)} | 4-way junctions: {len(fw)}", flush=True)
    print(f"using {'STOP-controlled' if sj else '4-way (designed all-way-stop)'} junctions", flush=True)
    hits = ego = 0; N = 16
    for i in range(N):
        sc = build(m)
        if sc is None:
            print("  no buildable junction"); break
        ind = [tools.generate_npc_behaviors(n['cfg'], 21, True) for n in sc['npcs']]
        try:
            res = nhtsa_sim.run_one(c, sc, ind, tick=0.05, max_frames=400, record_traj=True)
        except Exception as e:
            print("  run err:", e); continue
        rep = safety_oracle.evaluate(res, sc, tick=0.05)
        if res['isHit']: hits += 1
        if rep['is_safety_case']: ego += 1
        print(f"  run {i:>2} junc={sc['_junction']} hit={res['isHit']} case={res.get('case')} "
              f"ego_fault={rep['is_safety_case']} minDist={res['minDist']}", flush=True)
    print(f"\n[four-way-stop] {N} runs: collisions={hits}  EGO-AT-FAULT={ego} ({100*ego//N}%)", flush=True)
    print("FWSTOP_DONE", flush=True)
