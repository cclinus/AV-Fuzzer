import json, math, os, collections

SEV = {'low': 0.0, 'medium': 1.0, 'high': 2.0, 'unknown': 0.5}


def load(path):
    return [json.loads(l) for l in open(path)]


def featurize(cases):
    cats = sorted({c['category'] for c in cases})
    rules = sorted({c['violated_rule'] for c in cases})
    types = sorted({c['type'] for c in cases})

    def num(c):
        return [c.get('collision_speed_mph') or 0.0,
                c['ttc_min_s'] if c.get('ttc_min_s') is not None else 5.0,
                SEV.get(c.get('severity'), 0.5),
                c.get('npc_throttle_mean', 0.0), c.get('npc_brake_mean', 0.0),
                c['ev_start'][0], c['ev_start'][1], c['npc_start'][0], c['npc_start'][1]]

    raw = [num(c) for c in cases]
    cols = list(zip(*raw)) if raw else []
    mn = [min(col) for col in cols]; mx = [max(col) for col in cols]

    def norm(v):
        return [(v[i] - mn[i]) / (mx[i] - mn[i]) if mx[i] > mn[i] else 0.0 for i in range(len(v))]

    sigs = [c.get('traj_sig') or [] for c in cases]
    maxabs = max((abs(x) for s in sigs for x in s), default=1.0) or 1.0

    vecs = []
    for c, rn in zip(cases, raw):
        f = norm(rn)
        f += [1.0 if c['category'] == k else 0.0 for k in cats]
        f += [1.0 if c['violated_rule'] == k else 0.0 for k in rules]
        f += [1.0 if c['type'] == k else 0.0 for k in types]
        sig = c.get('traj_sig') or [0.0] * 16
        f += [x / maxabs for x in sig]
        vecs.append(f)
    return vecs


def _dist(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def identify_representative_safety_cases(cases, eps=1.2, sort_key=None):
    if not cases:
        return [], {}, {}
    vecs = featurize(cases)
    order = sorted(range(len(cases)),
                   key=sort_key or (lambda i: (-SEV.get(cases[i].get('severity'), 0.0),
                                               -(cases[i].get('collision_speed_mph') or 0.0))))
    reps = []; members = {}; assign = {}
    for i in order:
        best, bestd = None, None
        for ridx in reps:
            d = _dist(vecs[i], vecs[ridx])
            if bestd is None or d < bestd:
                bestd, best = d, ridx
        if best is None or bestd > eps:
            reps.append(i); members[i] = [i]; assign[i] = i
        else:
            members[best].append(i); assign[i] = best
    return reps, members, assign


def label(c):
    ttc = c.get('ttc_min_s')
    return (f"{c['category']}/{c['scenario']} · {c['type']} · {c['severity']} · "
            f"{c.get('collision_speed_mph')}mph"
            + (f" · TTC {ttc}s" if ttc is not None else ""))


def report(cases, eps=1.2):
    reps, members, assign = identify_representative_safety_cases(cases, eps=eps)
    n, m = len(cases), len(reps)
    vecs = featurize(cases); edges = 0
    for i in range(n):
        for j in range(i + 1, n):
            if _dist(vecs[i], vecs[j]) <= eps:
                edges += 1
    print(f"Safety Case Analyzer (Task 3.1)")
    print(f"  input safety cases     : {n}")
    print(f"  representative (unique) : {m}")
    print(f"  redundant removed       : {n - m}  ({100 * (n - m) / n:.0f}%)")
    print(f"  similarity-graph edges  : {edges} (pairs within eps={eps})")
    print(f"\n  by NHTSA category:  cases -> representatives")
    bycat = collections.Counter(c['category'] for c in cases)
    repcat = collections.Counter(cases[r]['category'] for r in reps)
    for cat in sorted(bycat):
        print(f"    {cat:<18} {bycat[cat]:>4} -> {repcat.get(cat, 0)}")
    print(f"\n  representative safety cases (rep · cluster size):")
    for r in sorted(reps, key=lambda r: -len(members[r])):
        print(f"    [x{len(members[r]):>3}] {label(cases[r])}")
    return reps, members, assign


if __name__ == '__main__':
    HERE = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(HERE, 'results', 'safety_cases.jsonl')
    cases = load(path)
    eps = float(os.environ.get('EPS', 1.2))
    reps, members, assign = report(cases, eps=eps)
    out = [{'representative': cases[r], 'cluster_size': len(members[r]),
            'member_ids': [cases[i]['id'] for i in members[r]]} for r in reps]
    json.dump(out, open(os.path.join(HERE, 'results', 'representative_safety_cases.json'), 'w'), indent=1)
    print(f"\n  wrote {len(out)} representative cases -> results/representative_safety_cases.json")
