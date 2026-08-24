import carla, json, os, sys, math

HERE = os.path.dirname(os.path.abspath(__file__))
MAP = sys.argv[1] if len(sys.argv) > 1 else 'Town03'

CFG = {
    'rear_end':  {'throttle_range': [0.2, 0.5], 'brake_chance': 0.5, 'brake_range': [0.3, 1.0], 'steer_range': [0.0, 0.02]},
    'head_on':   {'throttle_range': [0.4, 0.8], 'brake_chance': 0.1, 'brake_range': [0.0, 0.3], 'steer_range': [0.0, 0.03]},
    'sideswipe': {'throttle_range': [0.4, 0.7], 'brake_chance': 0.1, 'brake_range': [0.0, 0.2], 'steer_range': [0.03, 0.09]},
    'crossing':  {'throttle_range': [0.11, 0.17], 'brake_chance': 0.1, 'brake_range': [0.0, 0.3], 'steer_range': [0.0, 0.06]},
}
NHTSA = {'rear_end': 'Rear-end (LVD)', 'head_on': 'Opposing-direction (OD)',
         'sideswipe': 'Lane-change / merge (LC)', 'crossing': 'Intersection / crossing (SCP)'}


def loc(wp):
    l = wp.transform.location
    return [round(l.x, 2), round(l.y, 2), round(l.z + 0.3, 2)]


def yaw(wp): return round(wp.transform.rotation.yaw, 1)


def build(world, m, route, conflict):
    start = carla.Location(x=route['start'][0], y=route['start'][1], z=route['start'][2])
    ew = m.get_waypoint(start, project_to_road=True, lane_type=carla.LaneType.Driving)
    sc = {'map': MAP, 'conflict': conflict, 'nhtsa': NHTSA[conflict],
          'ev': {'start': route['start'], 'rotation': route['rotation'], 'end': route['end']},
          'npcs': []}
    if conflict == 'rear_end':
        nxt = ew.next(18.0)
        if not nxt: return None
        w = nxt[0]
        sc['npcs'].append({'start': loc(w), 'rotation': yaw(w), 'cfg': CFG['rear_end']})
    elif conflict == 'head_on':
        nxt = ew.next(45.0)
        if not nxt: return None
        w = nxt[0]
        sc['npcs'].append({'start': loc(w), 'rotation': (yaw(w) + 180.0) % 360.0, 'cfg': CFG['head_on']})
    elif conflict == 'sideswipe':
        adj = ew.get_left_lane() or ew.get_right_lane()
        if not adj or adj.lane_type != carla.LaneType.Driving: return None
        nxt = adj.next(10.0)
        if not nxt: return None
        w = nxt[0]
        sc['npcs'].append({'start': loc(w), 'rotation': yaw(w), 'cfg': CFG['sideswipe']})
    elif conflict == 'crossing':
        w = ew; junc = None
        for _ in range(30):
            nx = w.next(5.0)
            if not nx: break
            w = nx[0]
            if w.is_junction:
                junc = w.get_junction(); break
        if junc is None: return None
        ego_yaw = w.transform.rotation.yaw
        best = None
        for a, b in junc.get_waypoints(carla.LaneType.Driving):
            d = abs(((a.transform.rotation.yaw - ego_yaw + 90) % 180) - 90)
            if d > 60:
                prev = a.previous(12.0)
                if prev:
                    best = prev[0]; break
        if best is None: return None
        sc['npcs'].append({'start': loc(best), 'rotation': yaw(best), 'cfg': CFG['crossing']})
    return sc if sc['npcs'] else None


def main():
    c = carla.Client('localhost', 2000); c.set_timeout(60.0)
    world = c.load_world(MAP); m = world.get_map()
    routes = json.load(open(os.path.join(HERE, f'verified_routes_{MAP}.json')))['routes']
    print(f"{len(routes)} verified routes on {MAP}", flush=True)
    plan = ['rear_end', 'head_on', 'crossing', 'rear_end', 'head_on',
            'sideswipe', 'crossing', 'head_on', 'rear_end', 'sideswipe']
    scenarios = []
    ri = 0
    for conflict in plan:
        made = None
        for _ in range(len(routes)):
            r = routes[ri % len(routes)]; ri += 1
            made = build(world, m, r, conflict)
            if made:
                break
        if made:
            made['name'] = f"{conflict}_{sum(1 for s in scenarios if s['conflict']==conflict)+1}"
            scenarios.append(made)
            n = made['npcs'][0]
            print(f"  [{made['name']:<12}] {made['nhtsa']:<28} ego{made['ev']['start']} npc{n['start']}", flush=True)
        else:
            print(f"  [{conflict}] could not place on any route", flush=True)
    out = os.path.join(HERE, 'scenarios.json')
    json.dump(scenarios, open(out, 'w'), indent=1)
    print(f"\n{len(scenarios)} scenarios -> {out}", flush=True)


if __name__ == '__main__':
    main()
