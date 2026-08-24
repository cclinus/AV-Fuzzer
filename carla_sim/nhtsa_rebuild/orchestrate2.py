import os, subprocess, sys, socket, time, json

HERE = os.path.dirname(os.path.abspath(__file__))
PY = sys.executable
CARLA = r"C:\Users\10184\CARLA_0.9.15\WindowsNoEditor\CarlaUE4.exe"
OUT = os.path.join(HERE, 'results2')
GENS = os.environ.get('GA_GENS', '30')
POP = int(os.environ.get('GA_POP', '10'))
FULL = int(GENS) * POP
os.makedirs(OUT, exist_ok=True)


def port_up():
    s = socket.socket(); s.settimeout(1)
    try:
        s.connect(('localhost', 2000)); s.close(); return True
    except Exception:
        return False


def ensure_carla():
    if port_up():
        return True
    subprocess.Popen([CARLA, "-quality-level=Epic", "-carla-rpc-port=2000"])
    for _ in range(50):
        if port_up():
            time.sleep(8); return True
        time.sleep(3)
    return False


def linecount(p):
    if not os.path.exists(p):
        return 0
    with open(p) as f:
        return sum(1 for _ in f)


scenarios = json.load(open(os.path.join(HERE, 'scenarios.json')))
env = dict(os.environ); env['GA_GENS'] = GENS; env['GA_POP'] = str(POP)
env['PYTHONPATH'] = r"C:\Users\10184\AV-Fuzzer\carla_sim;C:\Users\10184\CARLA_0.9.15\WindowsNoEditor\PythonAPI\carla"

for i, s in enumerate(scenarios, 1):
    name = s['name']
    log = os.path.join(OUT, f'{name}.jsonl')
    if linecount(log) >= FULL:
        print(f"[{i}/{len(scenarios)}] {name}: complete -- skip", flush=True)
        continue
    if not ensure_carla():
        print(f"[{i}] CARLA down, cannot start -- skip {name}", flush=True)
        continue
    print(f"================ [{i}/{len(scenarios)}] {name} ({s['nhtsa']}) ================", flush=True)
    con = os.path.join(OUT, f'{name}.console.txt')
    with open(con, 'w') as cf:
        try:
            rc = subprocess.run([PY, os.path.join(HERE, 'run_ga2.py'), name, OUT],
                                stdout=cf, stderr=subprocess.STDOUT, env=env, timeout=14400).returncode
        except subprocess.TimeoutExpired:
            rc = 'TIMEOUT'
    print(f"  rc={rc} evals_logged={linecount(log)}", flush=True)

print("================ SUMMARY ================", flush=True)
import collections
rows = []
for s in scenarios:
    p = os.path.join(OUT, f"{s['name']}.jsonl")
    recs = [json.loads(l) for l in open(p)] if os.path.exists(p) else []
    cr = [r for r in recs if r['isHit']]
    cases = dict(collections.Counter(c['case'] for c in cr))
    rows.append((s['name'], s['nhtsa'], len(recs), len(cr), sum(1 for c in cr if c['egoFault']), cases))
    print(f"  {s['name']:<12} {s['nhtsa']:<26} evals={len(recs):>3} crashes={len(cr):>3} "
          f"egoFault={sum(1 for c in cr if c['egoFault']):>3} {cases}", flush=True)
json.dump([{'name': r[0], 'nhtsa': r[1], 'evals': r[2], 'crashes': r[3], 'egoFault': r[4], 'cases': r[5]} for r in rows],
          open(os.path.join(OUT, 'summary.json'), 'w'), indent=1)
print("FULL_RUN_DONE", flush=True)
