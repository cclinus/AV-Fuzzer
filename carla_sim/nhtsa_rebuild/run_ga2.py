import carla, json, os, sys, random, time
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(r"C:\Users\10184\AV-Fuzzer\carla_sim")
import tools
import nhtsa_sim

random.seed(7)
HERE = os.path.dirname(os.path.abspath(__file__))
TICK, FRAMES = 0.05, 500
NUM = FRAMES // int(1.0 / TICK) + 1
POP = int(os.environ.get('GA_POP', 8))
GENS = int(os.environ.get('GA_GENS', 30))
weather = {'cloudiness': 10, 'precipitation': 0, 'sun_altitude_angle': 70}


def make_individual(scenario):
    return [tools.generate_npc_behaviors(n['cfg'], NUM, True) for n in scenario['npcs']]


def ga_fitness(res):
    if res['isHit'] and res['egoFault']:
        return 100.0
    if res['isHit']:
        return 40.0
    return max(0.0, 30.0 - res['minDist'])


def tournament(pop, fits, k=3):
    best = random.randrange(len(pop))
    for _ in range(k - 1):
        c = random.randrange(len(pop))
        if fits[c] > fits[best]:
            best = c
    return pop[best]


def crossover(a, b):
    child = []
    for sa, sb in zip(a, b):
        cut = random.randrange(1, len(sa)) if len(sa) > 1 else 1
        child.append(sa[:cut] + sb[cut:])
    return child


def mutate(ind, scenario, rate=0.1):
    out = []
    for seq, npc in zip(ind, scenario['npcs']):
        fresh = tools.generate_npc_behaviors(npc['cfg'], len(seq), True)
        out.append([fresh[i] if random.random() < rate else seq[i] for i in range(len(seq))])
    return out


def robust_eval(client, scenario, individual):
    try:
        return nhtsa_sim.run_one(client, scenario, individual, tick=TICK, max_frames=FRAMES)
    except Exception as e:
        print(f"    [eval err {type(e).__name__}: {e}]", flush=True)
        return None


def run_scenario(client, scenario, outdir):
    name = scenario['name']
    logf = open(os.path.join(outdir, f'{name}.jsonl'), 'w')
    pop = [make_individual(scenario) for _ in range(POP)]
    crashes = 0; ego = 0; n = 0
    for gen in range(GENS):
        fits = []
        for idx, ind in enumerate(pop):
            res = robust_eval(client, scenario, ind); n += 1
            if res is None:
                fits.append(-1.0); continue
            fits.append(ga_fitness(res))
            rec = {'gen': gen, 'idx': idx, 'isHit': res['isHit'], 'case': res['case'],
                   'egoFault': res['egoFault'], 'minDist': res['minDist']}
            logf.write(json.dumps(rec) + '\n'); logf.flush()
            if res['isHit']:
                crashes += 1
                if res['egoFault']:
                    ego += 1
        best = max(range(len(pop)), key=lambda i: fits[i])
        newp = [pop[best]]
        while len(newp) < POP:
            p1, p2 = tournament(pop, fits), tournament(pop, fits)
            child = crossover(p1, p2) if random.random() < 0.8 else p1
            newp.append(mutate(child, scenario))
        pop = newp
        if gen % 5 == 0:
            print(f"    {name} gen{gen}: crashes={crashes} egoFault={ego} ({n} evals)", flush=True)
    logf.close()
    print(f"  >>> [{name}] {scenario['nhtsa']}: evals={n} crashes={crashes} egoFault={ego}", flush=True)
    return {'name': name, 'nhtsa': scenario['nhtsa'], 'conflict': scenario['conflict'],
            'evals': n, 'crashes': crashes, 'egoFault': ego}


def main():
    which = sys.argv[1] if len(sys.argv) > 1 else 'ALL'
    outdir = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, 'results')
    os.makedirs(outdir, exist_ok=True)
    scenarios = json.load(open(os.path.join(HERE, 'scenarios.json')))
    if which != 'ALL':
        scenarios = [s for s in scenarios if s['name'] == which]
    client = carla.Client('localhost', 2000); client.set_timeout(60.0)
    summary = []
    for s in scenarios:
        print(f"================ {s['name']} ({s['nhtsa']}) ================", flush=True)
        summary.append(run_scenario(client, s, outdir))
    json.dump(summary, open(os.path.join(outdir, 'summary.json'), 'w'), indent=1)
    print("GA2_DONE", flush=True)
    for r in summary:
        print(f"  {r['name']:<12} {r['nhtsa']:<28} crashes={r['crashes']:>3} egoFault={r['egoFault']:>3}", flush=True)


if __name__ == '__main__':
    main()
