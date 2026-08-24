import json, os, time, collections
import numpy as np

SEV = {'low': 0.0, 'medium': 1.0, 'high': 2.0, 'unknown': 0.5}

SPD = {'S0': 'the car is stopped', 'S1': 'the car is slow', 'S2': 'the car is at moderate speed',
       'S3': 'the car is fast', 'S4': 'the car is very fast'}
STE = {'L': 'turning left', 's': 'going straight', 'R': 'turning right', 'x': 'starting off'}
DIST = {'nr': 'another vehicle is very close', 'md': 'another vehicle is at medium range',
        'fr': 'another vehicle is far ahead'}
CLOS = {'C': 'and closing fast', 'e': 'and holding distance', 'O': 'and pulling away'}


def token_to_phrase(tok):
    sp, st, rel = tok[:2], tok[2:3], tok[3:]
    parts = [SPD.get(sp, 'the car is moving'), STE.get(st, 'driving')]
    if rel == 'na' or not rel:
        parts.append('with no vehicle nearby')
    else:
        parts.append(DIST.get(rel[:2], 'another vehicle is near') + ' ' + CLOS.get(rel[2:], ''))
    return ', '.join(p.strip() for p in parts if p)


def case_to_sentence(case):
    head = (f"{case['category']} scenario. {case['type'].replace('_', ' ')} "
            f"violating {case['violated_rule'].replace('_', ' ')}, {case.get('severity', 'unknown')} severity.")
    body = '. '.join(token_to_phrase(t) for t in (case.get('event_seq') or []))
    tail = []
    cs = case.get('collision_speed_mph'); ttc = case.get('ttc_min_s')
    if cs is not None: tail.append(f"impact speed {cs} miles per hour")
    if ttc is not None: tail.append(f"time to collision {ttc} seconds")
    ev = case.get('ev_start'); npc = case.get('npc_start'); ctx = []
    if ev:
        ns = 'northern' if ev[1] < -170 else 'southern' if ev[1] > -150 else 'central'
        ew = 'western' if ev[0] < 20 else 'eastern' if ev[0] > 40 else 'middle'
        ctx.append(f"the crash is in the {ns} {ew} area at grid {round(ev[0] / 12)} {round(ev[1] / 12)}")
    if ev and npc:
        dx, dy = npc[0] - ev[0], npc[1] - ev[1]
        pos = ('ahead' if dx > 3 else 'behind' if dx < -3 else 'alongside') + (' left' if dy < -3 else ' right' if dy > 3 else '')
        ctx.append(f"the other vehicle starts {pos}")
    thr = case.get('npc_throttle_mean')
    if thr is not None:
        ctx.append('the other vehicle drives ' + ('slowly' if thr < 0.2 else 'moderately' if thr < 0.45 else 'fast'))
    sent = head + ' ' + body + ('. ' + ', '.join(tail) if tail else '')
    if ctx:
        sent += '. ' + '. '.join(ctx)
    return (sent + '.').strip()


def load(path):
    return [json.loads(l) for l in open(path)]


_MODEL = None
def get_model(name='all-MiniLM-L6-v2'):
    global _MODEL
    if _MODEL is None:
        from sentence_transformers import SentenceTransformer
        _MODEL = SentenceTransformer(name)
    return _MODEL


def embed(cases, model=None):
    model = model or get_model()
    sents = [case_to_sentence(c) for c in cases]
    emb = np.asarray(model.encode(sents, normalize_embeddings=True, show_progress_bar=False))
    if len(cases) >= 3:
        Ec = emb - emb.mean(0)
        _, _, Vt = np.linalg.svd(Ec, full_matrices=False)
        emb = Ec - Ec @ Vt[:1].T @ Vt[:1]
    n = np.linalg.norm(emb, axis=1, keepdims=True); n[n == 0] = 1.0
    return emb / n


def identify_representative_safety_cases(cases, sim_threshold=0.90, model=None, emb=None):
    if not cases:
        return [], {}, {}, None
    if emb is None:
        emb = embed(cases, model)
    sim = emb @ emb.T
    order = sorted(range(len(cases)),
                   key=lambda i: (-SEV.get(cases[i].get('severity'), 0.0),
                                  -(cases[i].get('collision_speed_mph') or 0.0)))
    reps = []; members = {}; assign = {}
    for i in order:
        best, bestsim = None, -1.0
        for r in reps:
            if sim[i, r] > bestsim:
                bestsim, best = sim[i, r], r
        if best is None or bestsim < sim_threshold:
            reps.append(i); members[i] = [i]; assign[i] = i
        else:
            members[best].append(i); assign[i] = best
    return reps, members, assign, sim


def pairwise_pr(cases, assign):
    import math
    n = len(cases); tp = fp = fn = tn = 0
    for i in range(n):
        for j in range(i + 1, n):
            ps = assign[i] == assign[j]; ts = cases[i]['gt_group'] == cases[j]['gt_group']
            if ps and ts: tp += 1
            elif ps: fp += 1
            elif ts: fn += 1
            else: tn += 1
    p = tp / (tp + fp) if (tp + fp) else float('nan')
    r = tp / (tp + fn) if (tp + fn) else float('nan')
    f = 2 * p * r / (p + r) if (p + r) else float('nan')
    return {'TP': tp, 'FP': fp, 'FN': fn, 'TN': tn,
            'precision': round(p, 3), 'recall': round(r, 3),
            'f1': None if f != f else round(f, 3)}


if __name__ == '__main__':
    HERE = os.path.dirname(os.path.abspath(__file__))
    print("loading transformer (all-MiniLM-L6-v2, distilled BERT)...", flush=True)
    t0 = time.perf_counter()
    model = get_model()
    print(f"  model loaded in {time.perf_counter() - t0:.1f}s", flush=True)

    real = load(os.path.join(HERE, 'results', 'safety_cases.jsonl'))
    print("\nexample scenario -> language:\n  " + case_to_sentence(real[0]), flush=True)

    bench = load(os.path.join(HERE, 'results', 'benchmark_cases.jsonl'))
    t0 = time.perf_counter(); bemb = embed(bench, model); enc_bench = time.perf_counter() - t0
    print(f"\nencoded {len(bench)} benchmark sentences in {enc_bench:.2f}s "
          f"({1000 * enc_bench / len(bench):.0f} ms/case)", flush=True)
    print("== BERT analyzer: pairwise P/R vs labeled benchmark ==", flush=True)
    best = None
    for sim in [0.80, 0.85, 0.90, 0.93, 0.95, 0.97]:
        reps, _, assign, _ = identify_representative_safety_cases(bench, sim_threshold=sim, emb=bemb)
        m = pairwise_pr(bench, assign)
        print(f"  sim={sim:<5} clusters={len(reps):>2}  P={m['precision']} R={m['recall']} F1={m['f1']}", flush=True)
        if m['f1'] is not None and (best is None or m['f1'] > best[1]):
            best = (sim, m['f1'], m)
    print(f"  -> best F1={best[1]} at sim={best[0]}  ({best[2]})", flush=True)

    t0 = time.perf_counter(); remb = embed(real, model); enc_real = time.perf_counter() - t0
    print(f"\ntraining/encoding time = {enc_real:.2f}s on {len(real)} real cases "
          f"({1000 * enc_real / len(real):.0f} ms/case)", flush=True)

    by = collections.defaultdict(list)
    for c in real: by[c['category']].append(c)
    cross = by['Intersection'] + by['Crossing-traffic']
    fams = {'Rear-end': by['Rear-end'][:50], 'Intersection / crossing': cross[:50], 'Merge / lane-change': by['Merge'][:50]}
    sim_op = best[0]
    print(f"\n== representative crash cases (per-category dedup, sim={sim_op}) ==", flush=True)
    tot_in = tot_rep = 0
    rows = []
    for name, cs in fams.items():
        reps, _, _, _ = identify_representative_safety_cases(cs, sim_threshold=sim_op, model=model)
        rows.append((name, len(cs), len(reps)))
        print(f"   {name:<24} input={len(cs):>3}  representatives={len(reps)}", flush=True)
        tot_in += len(cs); tot_rep += len(reps)
    print(f"   {'TOTAL':<24} input={tot_in:>3}  representatives={tot_rep}", flush=True)

    json.dump({'encoder': 'all-MiniLM-L6-v2 (distilled BERT)',
               'benchmark_best': {'threshold': best[0], **best[2]},
               'encode_time_s_real': round(enc_real, 2), 'ms_per_case': round(1000 * enc_real / len(real), 1),
               'table2': [{'category': n, 'input': i, 'representatives': r} for n, i, r in rows],
               'table2_total': {'input': tot_in, 'representatives': tot_rep}},
              open(os.path.join(HERE, 'results', 'analyzer_bert.json'), 'w'), indent=1)
    print("\nBERT_ANALYZER_DONE", flush=True)
