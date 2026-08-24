import carla, sys, json, os
from agents.navigation.behavior_agent import BehaviorAgent

MAP = sys.argv[1] if len(sys.argv) > 1 else 'Town03'
N = int(sys.argv[2]) if len(sys.argv) > 2 else 12
NEXT_DIST = 90.0
TICK = 0.05
MAX_FRAMES = 700


def drive_route(world, m, start_tf, end_loc):
    bl = world.get_blueprint_library()
    bp = bl.filter('vehicle.tesla.model3')[0]
    ego = world.try_spawn_actor(bp, start_tf)
    if ego is None:
        return {'spawned': False}
    s = world.get_settings(); s.synchronous_mode = True; s.fixed_delta_seconds = TICK
    world.apply_settings(s)
    try:
        agent = BehaviorAgent(ego, behavior='normal')
        agent.set_destination(end_loc)
        for _ in range(10):
            world.tick()
        offroad = 0; reached = False; mind = 1e9; f = 0
        for f in range(MAX_FRAMES):
            world.tick()
            loc = ego.get_location()
            d = loc.distance(end_loc); mind = min(mind, d)
            on = m.get_waypoint(loc, project_to_road=False,
                                lane_type=carla.LaneType.Driving) is not None
            if not on:
                offroad += 1
            if agent.done() or d < 3.0:
                reached = True; break
            ego.apply_control(agent.run_step())
        end_dist = ego.get_location().distance(end_loc)
        return {'spawned': True, 'reached': reached, 'offroad': offroad,
                'min_dist': round(mind, 1), 'end_dist': round(end_dist, 1), 'frames': f + 1}
    finally:
        if ego.is_alive:
            ego.destroy()
        s = world.get_settings(); s.synchronous_mode = False; world.apply_settings(s)


def main():
    c = carla.Client('localhost', 2000); c.set_timeout(60.0)
    print(f"loading {MAP} ...", flush=True)
    world = c.load_world(MAP)
    m = world.get_map()
    sps = m.get_spawn_points()
    print(f"{MAP}: {len(sps)} recommended spawn points; testing {min(N,len(sps))}", flush=True)
    step = max(1, len(sps) // N)
    valid = []
    for idx in range(0, len(sps), step):
        start_tf = sps[idx]
        wp = m.get_waypoint(start_tf.location, project_to_road=True, lane_type=carla.LaneType.Driving)
        nxt = wp.next(NEXT_DIST) if wp else []
        if not nxt:
            print(f"  sp[{idx:3d}] no lane {NEXT_DIST}m ahead -> skip", flush=True)
            continue
        end_loc = nxt[0].transform.location
        v = drive_route(world, m, start_tf, end_loc)
        ok = v.get('reached') and v.get('offroad') == 0
        s, r = start_tf.location, start_tf.rotation
        print(f"  sp[{idx:3d}] {'VALID  ' if ok else 'reject '} "
              f"reached={v.get('reached')} offroad={v.get('offroad')} "
              f"end_dist={v.get('end_dist')} frames={v.get('frames')}", flush=True)
        if ok:
            valid.append({
                'spawn_index': idx,
                'start': [round(s.x, 2), round(s.y, 2), round(s.z, 2)],
                'rotation': round(r.yaw, 1),
                'end': [round(end_loc.x, 2), round(end_loc.y, 2), round(end_loc.z, 2)],
                'verify': v,
            })
        if len(valid) >= N:
            break
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), f'verified_routes_{MAP}.json')
    json.dump({'map': MAP, 'next_dist': NEXT_DIST, 'routes': valid}, open(out, 'w'), indent=1)
    print(f"\n{len(valid)} VALID verified routes -> {out}", flush=True)
    for v in valid:
        print(f"  start={v['start']} yaw={v['rotation']} -> end={v['end']}", flush=True)


if __name__ == '__main__':
    main()
