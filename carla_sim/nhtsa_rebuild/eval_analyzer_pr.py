import json, os, math, collections
import safety_analyzer as PA
import safety_analyzer_bert as BA

HERE = os.path.dirname(os.path.abspath(__file__))


def load(path):
    return [json.loads(l) for l in open(path)]


def pairwise_pr(cases, assign):
    n = len(cases); tp = fp = fn = tn = 0
    for i in range(n):
        for j in range(i + 1, n):
            ps = assign[i] == assign[j]
            ts = cases[i]['gt_group'] == cases[j]['gt_group']
            if ps and ts: tp += 1
            elif ps and not ts: fp += 1
            elif (not ps) and ts: fn += 1
            else: tn += 1
    p = tp / (tp + fp) if (tp + fp) else float('nan')
    r = tp / (tp + fn) if (tp + fn) else float('nan')
    f = 2 * p * r / (p + r) if (p + r) and not math.isnan(p) and not math.isnan(r) else float('nan')
    return {'TP': tp, 'FP': fp, 'FN': fn, 'TN': tn,
            'precision': None if math.isnan(p) else round(p, 3),
            'recall': None if math.isnan(r) else round(r, 3),
            'f1': None if math.isnan(f) else round(f, 3)}


def main():
    cases = load(os.path.join(HERE, 'results', 'benchmark_cases.jsonl'))
    print(f"benchmark: {len(cases)} cases, groups = {dict(collections.Counter(c['gt_name'] for c in cases))}\n")

    print("== Parameter analyzer (features + trajectory, threshold eps) ==")
    best_p = None
    for eps in [0.6, 0.8, 1.0, 1.2, 1.5, 2.0]:
        reps, members, assign = PA.identify_representative_safety_cases(cases, eps=eps)
        m = pairwise_pr(cases, assign)
        print(f"  eps={eps:<4} clusters={len(reps):>2}  P={m['precision']} R={m['recall']} F1={m['f1']}")
        if m['f1'] is not None and (best_p is None or m['f1'] > best_p[1]):
            best_p = (eps, m['f1'], m)
    print(f"  -> best F1={best_p[1]} at eps={best_p[0]}  ({best_p[2]})\n")

    print("== BERT / transformer semantic-encoder analyzer (cosine threshold) ==")
    best_b = None
    emb = BA.embed(cases)
    for sim in [0.80, 0.85, 0.90, 0.93, 0.95, 0.97]:
        reps, members, assign, _ = BA.identify_representative_safety_cases(cases, sim_threshold=sim, emb=emb)
        m = pairwise_pr(cases, assign)
        print(f"  sim={sim:<5} clusters={len(reps):>2}  P={m['precision']} R={m['recall']} F1={m['f1']}")
        if m['f1'] is not None and (best_b is None or m['f1'] > best_b[1]):
            best_b = (sim, m['f1'], m)
    print(f"  -> best F1={best_b[1]} at sim={best_b[0]}  ({best_b[2]})")

    json.dump({'parameter_best': {'threshold': best_p[0], **best_p[2]},
               'bert_best': {'threshold': best_b[0], **best_b[2]}},
              open(os.path.join(HERE, 'results', 'analyzer_pr.json'), 'w'), indent=1)
    print("\nANALYZER_PR_DONE")


if __name__ == '__main__':
    main()
