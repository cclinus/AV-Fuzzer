import carla, math, sys, os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from agents.navigation.behavior_agent import BehaviorAgent
import liability2 as L


def make_tf(xyz, yaw=0.0):
    return carla.Transform(carla.Location(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2])),
                           carla.Rotation(yaw=float(yaw)))


def clear_vehicles(world):
    for a in list(world.get_actors().filter('vehicle.*')):
        try: a.destroy()
        except Exception: pass
    for a in list(world.get_actors().filter('walker.*')):
        try: a.destroy()
        except Exception: pass


_cur = {'map': None}
def ensure_map(client, name):
    if _cur['map'] != name:
        client.load_world(name)
        _cur['map'] = name
    return client.get_world()


def run_one(client, scenario, individual, tick=0.05, max_frames=500, debug=False, record_traj=False):
    client = carla.Client('localhost', 2000); client.set_timeout(60.0)
    world = ensure_map(client, scenario['map'])
    m = world.get_map()
    clear_vehicles(world)
    for tl in world.get_actors().filter('*traffic_light*'):
        tl.set_state(carla.TrafficLightState.Green)
        tl.set_green_time(9999.0)
        tl.freeze(True)
    bl = world.get_blueprint_library()
    ego_bp = bl.filter('vehicle.tesla.model3')[0]
    npc_bp = bl.filter('vehicle.audi.tt')[0]

    ev = scenario['ev']
    mode = scenario.get('ego_mode', 'normal')
    thr_floor = float(scenario.get('ego_throttle_floor', 0.6))
    drift_cfg = scenario.get('drift', {})
    ego = world.spawn_actor(ego_bp, make_tf(ev['start'], ev.get('rotation', 0.0)))
    agent = BehaviorAgent(ego, behavior='aggressive' if mode == 'reckless' else 'normal')
    agent.set_destination(carla.Location(x=float(ev['end'][0]), y=float(ev['end'][1]), z=float(ev['end'][2])))

    def ego_control(f):
        if mode == 'reckless':
            c = agent.get_local_planner().run_step()
            c.throttle = 0.35 if abs(c.steer) > 0.35 else thr_floor
            c.brake = 0.0; c.hand_brake = False
            return c
        if mode == 'drift':
            c = agent.run_step()
            if f >= drift_cfg.get('onset', 30):
                c.steer = max(-1.0, min(1.0, drift_cfg.get('side', 1.0) * drift_cfg.get('s_drift', 0.05)))
                c.brake = 0.0; c.throttle = max(c.throttle, 0.4)
            return c
        return agent.run_step()

    npcs = []
    for nc in scenario['npcs']:
        v = None
        for dz in (0.0, 0.5, 1.0):
            v = world.try_spawn_actor(npc_bp, make_tf([nc['start'][0], nc['start'][1], nc['start'][2] + dz],
                                                      nc.get('rotation', 0.0)))
            if v:
                break
        npcs.append(v)

    ego_ext = ego.bounding_box.extent
    row_yielder = scenario.get('row_yielder')
    hit = {'v': False, 'ego': False, 'case': 'none'}
    velbuf = {'e': [], 'n': []}

    def _avg(lst):
        if not lst:
            return None
        return (sum(a for a, _ in lst) / len(lst), sum(b for _, b in lst) / len(lst))

    def on_col(e):
        if hit['v']:
            return
        other = e.other_actor
        if ('vehicle' in e.actor.type_id and 'vehicle' in other.type_id and
                (e.actor.id == ego.id or other.id == ego.id)):
            hit['v'] = True
            o = other if other.id != ego.id else e.actor
            etf = ego.get_transform(); otf = o.get_transform()
            lane_wp = m.get_waypoint(etf.location)
            on_road = m.get_waypoint(etf.location, project_to_road=False,
                                     lane_type=carla.LaneType.Driving) is not None
            oe = o.bounding_box.extent
            evel = _avg(velbuf['e'])
            ovel = _avg(velbuf['n']) if (npcs and npcs[0] and o.id == npcs[0].id) else None
            if evel is None:
                ev_ = ego.get_velocity(); evel = (ev_.x, ev_.y)
            if ovel is None:
                ov_ = o.get_velocity(); ovel = (ov_.x, ov_.y)
            args = ((etf.location.x, etf.location.y, etf.rotation.yaw),
                    (otf.location.x, otf.location.y, otf.rotation.yaw),
                    (lane_wp.transform.location.x, lane_wp.transform.location.y, lane_wp.transform.rotation.yaw),
                    (ego_ext.x, ego_ext.y), (oe.x, oe.y))
            common = dict(ego_on_road=on_road, is_junction=lane_wp.is_junction, ego_vel=evel, npc_vel=ovel)
            fault_g, case_g = L.is_ego_fault(*args, **common)
            fault_r, case_r = L.is_ego_fault(*args, row_yielder=row_yielder, **common)
            hit['ego'] = fault_r; hit['case'] = case_r
            hit['ego_geom'] = fault_g; hit['case_geom'] = case_g
            hit['ego_pose'] = (round(etf.location.x, 2), round(etf.location.y, 2), round(etf.rotation.yaw, 1))
            hit['npc_pose'] = (round(otf.location.x, 2), round(otf.location.y, 2), round(otf.rotation.yaw, 1))
            hit['lane_pose'] = (round(lane_wp.transform.location.x, 2), round(lane_wp.transform.location.y, 2),
                                round(lane_wp.transform.rotation.yaw, 1))
            hit['on_road'] = on_road; hit['is_junction'] = lane_wp.is_junction

    cs = world.spawn_actor(bl.find('sensor.other.collision'), carla.Transform(), attach_to=ego)
    cs.listen(on_col)

    s = world.get_settings(); s.synchronous_mode = True; s.fixed_delta_seconds = tick
    world.apply_settings(s)
    try:
        for _ in range(10):
            world.tick()
        for i, v in enumerate(npcs):
            if v and individual[i]:
                v.apply_control(individual[i][0])
        world.tick(); agent.run_step()
        li = int(1.0 / tick); mind = 1e9; f = 0; reached = False; traj = []; full_traj = []
        for f in range(max_frames):
            world.tick()
            if hit['v']:
                break
            if agent.done():
                reached = True; break
            ego.apply_control(ego_control(f))
            if f % li == 0:
                idx = f // li
                for i, v in enumerate(npcs):
                    if v and idx < len(individual[i]):
                        v.apply_control(individual[i][idx])
            et = ego.get_transform(); evv = ego.get_velocity(); el = et.location
            nv0 = npcs[0].get_velocity() if (npcs and npcs[0]) else None
            velbuf['e'].append((evv.x, evv.y))
            velbuf['n'].append((nv0.x, nv0.y) if nv0 else (0.0, 0.0))
            if len(velbuf['e']) > 6:
                velbuf['e'].pop(0); velbuf['n'].pop(0)
            for v in npcs:
                if v:
                    mind = min(mind, el.distance(v.get_location()))
            if record_traj:
                nrec = None
                if npcs and npcs[0]:
                    nt = npcs[0].get_transform()
                    nrec = (round(nt.location.x, 2), round(nt.location.y, 2), round(nt.rotation.yaw, 1),
                            round(nv0.x, 2), round(nv0.y, 2))
                full_traj.append((f, round(el.x, 2), round(el.y, 2), round(et.rotation.yaw, 1),
                                  round(evv.x, 2), round(evv.y, 2), nrec))
            if debug and f % 20 == 0:
                nl = npcs[0].get_location() if npcs and npcs[0] else None
                traj.append((f, round(el.x, 1), round(el.y, 1),
                             (round(nl.x, 1), round(nl.y, 1)) if nl else None, round(mind, 1)))
        ee = ego.get_location()
        out = {'isHit': hit['v'], 'egoFault': hit['ego'], 'case': hit['case'],
               'egoFault_geom': hit.get('ego_geom'), 'case_geom': hit.get('case_geom'),
               'minDist': round(mind, 2), 'frames': f + 1, 'reached': reached,
               'egoEnd': (round(ee.x, 1), round(ee.y, 1)), 'ego_mode': mode,
               'impact': ({'ego_pose': hit.get('ego_pose'), 'npc_pose': hit.get('npc_pose'),
                           'lane_pose': hit.get('lane_pose'), 'on_road': hit.get('on_road'),
                           'is_junction': hit.get('is_junction')} if hit['v'] else None)}
        if debug:
            out['traj'] = traj
        if record_traj:
            out['full_traj'] = full_traj
        return out
    finally:
        try: cs.destroy()
        except Exception: pass
        clear_vehicles(world)
        s = world.get_settings(); s.synchronous_mode = False; world.apply_settings(s)
