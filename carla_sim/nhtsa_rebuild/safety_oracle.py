import math
import liability2 as L
import groundtruth as G

MPS_TO_MPH = 2.23694


def brake_distance(v):
    return max(0.0467 * v * v + 0.4116 * v - 1.9913 + 0.5, 0.0)


def _fwd(yaw_deg, vx, vy):
    r = math.radians(yaw_deg)
    return math.cos(r) * vx + math.sin(r) * vy


def _lon_lat(ex, ey, eyaw_deg, px, py):
    r = math.radians(eyaw_deg); ux, uy = math.cos(r), math.sin(r)
    dx, dy = px - ex, py - ey
    return ux * dx + uy * dy, abs(-uy * dx + ux * dy)


def _head_diff(a, b):
    return abs(((a - b + 180) % 360) - 180)


def ttc_min(full_traj, buf=4.5):
    best = None
    for row in full_traj or []:
        n = row[6]
        if not n:
            continue
        eyaw, evx, evy = row[3], row[4], row[5]
        lon, _ = _lon_lat(row[1], row[2], eyaw, n[0], n[1])
        if lon <= 0:
            continue
        gap = lon - buf
        closing = _fwd(eyaw, evx, evy) - _fwd(eyaw, n[3], n[4])
        if closing > 0.1 and gap > 0:
            t = gap / closing
            if best is None or t < best:
                best = t
    return None if best is None else round(best, 2)


def _impact_speed_mph(result):
    ft = result.get('full_traj')
    if not ft:
        return None
    row = ft[-1]
    return round(math.hypot(row[4], row[5]) * MPS_TO_MPH, 1)


def _severity(is_hit, speed_mph, ttc):
    if is_hit and speed_mph is not None:
        return 'high' if speed_mph >= 25 else 'medium' if speed_mph >= 10 else 'low'
    if ttc is not None:
        return 'high' if ttc < 1.0 else 'medium' if ttc < 2.5 else 'low'
    return 'unknown'


def responsibility(result):
    is_hit = bool(result.get('isHit'))
    ego = bool(result.get('egoFault'))
    case = result.get('case', 'none')
    return {'ego_at_fault': ego, 'npc_at_fault': is_hit and not ego,
            'could_ego_avoid': (not ego) if is_hit else None,
            'at_fault_case': case, 'veer_artifact': case.endswith('_veer')}


def _mk(oracle, layer, vtype, rule, result, ctx, confidence, detail, ego_at_fault=None):
    resp = ctx['responsibility']
    return {'oracle': oracle, 'layer': layer, 'type': vtype, 'violated_rule': rule,
            'ego_at_fault': resp['ego_at_fault'] if ego_at_fault is None else ego_at_fault,
            'severity': _severity(result.get('isHit'), ctx['impact_speed_mph'], ctx['ttc_min']),
            'ttc_min_s': ctx['ttc_min'], 'collision_speed_mph': ctx['impact_speed_mph'],
            'confidence': round(confidence, 2), 'frame': detail.get('frame'),
            'time_s': None if detail.get('frame') is None else round(detail['frame'] * ctx['tick'], 2),
            'detail': detail}


def check_safe_distance(result, scenario, cfg, ctx):
    mode = cfg['mode']; buf = cfg['ego_half_len'] + cfg['npc_half_len']
    if mode in ('at_collision', 'both') and result.get('isHit'):
        imp = result.get('impact') or {}; case = result.get('case', '')
        ok_case = case in cfg['collision_cases'] or (cfg.get('collision_case_fallback') and case.startswith('rear_end'))
        if result.get('egoFault') and ok_case and not (cfg.get('skip_if_impact_junction') and imp.get('is_junction')):
            conf = 0.95 if case == 'rear_end' else 0.8
            return _mk('safe_distance', 'physical', 'rear_end_collision', 'safe_following_distance',
                       result, ctx, conf, {'frame': ctx['impact_frame'], 'case': case, 'trigger': 'collision'})
    if mode in ('continuous', 'both'):
        ft = result.get('full_traj')
        if not ft:
            return None
        run = 0; min_gap = None; worst = None
        for row in ft:
            n = row[6]
            if not n:
                run = 0; continue
            eyaw, evx, evy = row[3], row[4], row[5]; ve = math.hypot(evx, evy)
            lon, lat = _lon_lat(row[1], row[2], eyaw, n[0], n[1])
            valid = lon > 0 and lat <= cfg['lateral_tolerance'] and _head_diff(eyaw, n[2]) <= cfg['same_direction_tol_deg']
            if not valid or ve <= cfg['min_ego_speed']:
                run = 0; continue
            gap = lon - buf
            d_req = max(brake_distance(ve) if cfg['use_brake_distance'] else 0.0, cfg['headway_time'] * ve, cfg['min_standstill_gap'])
            closing = _fwd(eyaw, evx, evy) - _fwd(eyaw, n[3], n[4])
            if gap < d_req and (cfg['causation'] == 'off' or closing >= cfg['min_closing_mps']):
                run += 1
                if min_gap is None or gap < min_gap:
                    min_gap = gap; worst = row[0]
                if run >= cfg['persist_frames']:
                    conf = max(0.4, min(0.9, 1.0 - (gap / d_req if d_req else 0)))
                    return _mk('safe_distance', 'physical', 'unsafe_following_distance', 'safe_following_distance',
                               result, ctx, conf, {'frame': worst, 'min_gap_m': round(min_gap, 2), 'trigger': 'continuous'},
                               ego_at_fault=True)
            else:
                run = 0
    return None


def check_yield_row(result, scenario, cfg, ctx):
    if cfg.get('row_source', 'scenario') == 'map':
        return None
    yielder = scenario.get('row_yielder')
    if yielder != 'ego' or not result.get('isHit'):
        return None
    imp = result.get('impact') or {}; case = result.get('case', '')
    junction_ctx = imp.get('is_junction') or result.get('case_geom') in ('turn_across_opp', 'turn_into')
    if result.get('egoFault') and junction_ctx and case.endswith('_row'):
        return _mk('yield_row', 'traffic_rules', 'failure_to_yield', 'right_of_way', result, ctx, 0.9,
                   {'frame': ctx['impact_frame'], 'case': case, 'is_junction': imp.get('is_junction')})
    return None


def check_lane_invasion(result, scenario, cfg, ctx):
    det = cfg['detection']; imp = result.get('impact') or {}; case = result.get('case', '')
    collision_at_fault = bool(result.get('isHit') and case.startswith('sideswipe')
                              and result.get('egoFault') and not case.endswith('_veer'))
    if det in ('at_collision', 'both') and collision_at_fault and not imp.get('is_junction'):
        ep = imp.get('ego_pose'); lp = imp.get('lane_pose')
        if ep and lp and L.lane_angle_diff(ep[2], lp[2]) >= cfg['Steering_Angle_at_Collision'][0]:
            return _mk('lane_invasion', 'traffic_rules', 'lane_invasion', 'lane_keeping', result, ctx, 0.85,
                       {'frame': ctx['impact_frame'], 'case': case, 'trigger': 'collision'})
    if det in ('continuous', 'both'):
        ft = result.get('full_traj')
        if ft:
            axis = scenario.get('axis_yaw')
            if axis is None:
                axis = ft[0][3]
            B = cfg['lane_cross_offset_m']; r = math.radians(axis); nx, ny = -math.sin(r), math.cos(r)
            ex0, ey0 = ft[0][1], ft[0][2]; run = 0; maxlat = 0.0; cross_f = None
            for row in ft:
                d = abs(nx * (row[1] - ex0) + ny * (row[2] - ey0)); maxlat = max(maxlat, d)
                if d >= B:
                    run += 1
                    if cross_f is None:
                        cross_f = row[0]
                else:
                    run = 0
                if run >= cfg['min_hold_frames']:
                    break
            sustained = run >= cfg['min_hold_frames']
            initiator = G.incursion_initiator(ft, axis, thresh=B)
            illegit = collision_at_fault
            if sustained and initiator == 'ego' and (illegit or not cfg.get('illegitimacy')):
                return _mk('lane_invasion', 'traffic_rules', 'lane_invasion', 'lane_keeping', result, ctx, 0.8,
                           {'frame': cross_f, 'max_lateral_offset_m': round(maxlat, 2), 'trigger': 'continuous'},
                           ego_at_fault=True)
    return None


def check_red_light(result, scenario, cfg, ctx):
    return None


ORACLES = {'safe_distance': check_safe_distance, 'yield_row': check_yield_row,
           'lane_invasion': check_lane_invasion, 'red_light': check_red_light}
LAYER = {'safe_distance': 'physical', 'yield_row': 'traffic_rules',
         'lane_invasion': 'traffic_rules', 'red_light': 'traffic_rules'}

DEFAULTS = {
    'safe_distance': {'mode': 'both', 'use_brake_distance': True, 'headway_time': 1.5, 'min_standstill_gap': 2.0,
                      'ego_half_len': 2.4, 'npc_half_len': 2.1, 'lateral_tolerance': 2.0, 'same_direction_tol_deg': 45,
                      'min_ego_speed': 1.0, 'persist_frames': 10, 'causation': 'closing', 'min_closing_mps': 0.0,
                      'collision_cases': ['rear_end'], 'collision_case_fallback': False, 'skip_if_impact_junction': False},
    'yield_row': {'detection': 'at_collision', 'row_source': 'scenario', 'require_junction': True},
    'lane_invasion': {'detection': 'both', 'Steering_Angle_at_Collision': [10, 90], 'lane_cross_offset_m': 1.75,
                      'min_hold_frames': 6, 'ignore_junctions': True, 'illegitimacy': ['collision_at_fault']},
    'red_light': {'detection': 'both', 'signal_source': 'recorded_channel'},
}


def _cfg(name, entry):
    c = dict(DEFAULTS[name]); c.update({k: v for k, v in (entry or {}).items() if k not in ('name', 'enabled')})
    return c


def default_config():
    return {'ADS_model': 'tesla.model3', 'oracles': [
        {'name': 'safe_distance', 'enabled': True}, {'name': 'yield_row', 'enabled': True},
        {'name': 'lane_invasion', 'enabled': True}, {'name': 'red_light', 'enabled': False}]}


def evaluate(result, scenario, config=None, map_ctx=None, tick=0.05):
    config = config or default_config()
    ft = result.get('full_traj')
    impact_frame = (result['frames'] - 1) if result.get('frames') is not None else (ft[-1][0] if ft else None)
    resp = responsibility(result)
    ctx = {'tick': tick, 'impact_frame': impact_frame, 'map_ctx': map_ctx, 'responsibility': resp,
           'ttc_min': ttc_min(ft), 'impact_speed_mph': _impact_speed_mph(result) if result.get('isHit') else None}
    violations = []
    for entry in config.get('oracles', []):
        if not entry.get('enabled', True):
            continue
        v = ORACLES[entry['name']](result, scenario, _cfg(entry['name'], entry), ctx)
        if v:
            violations.append(v)
    layers = {'physical': [v for v in violations if v['layer'] == 'physical'],
              'traffic_rules': [v for v in violations if v['layer'] == 'traffic_rules'],
              'responsibility': resp}
    return {'is_safety_case': len(violations) > 0,
            'safety_case_types': [v['type'] for v in violations],
            'violations': violations, 'layers': layers,
            'meta': {'tick': tick, 'impact_frame': impact_frame, 'ttc_min_s': ctx['ttc_min'],
                     'collision_speed_mph': ctx['impact_speed_mph'], 'ego_mode': result.get('ego_mode'),
                     'used_map_ctx': map_ctx is not None}}


def safety_metric(result, tick=0.05, buf=4.5):
    if result.get('isHit'):
        return 100.0
    score = 0.0
    md = result.get('minDist')
    if md is not None:
        score += max(0.0, 40.0 - md)
    t = ttc_min(result.get('full_traj'), buf)
    if t is not None:
        score += max(0.0, 20.0 - t * 4.0)
    return round(score, 2)


if __name__ == '__main__':
    ok = 0; tot = 0
    def check(name, got, want):
        global ok, tot; tot += 1; good = got == want; ok += good
        print(f"  {'PASS' if good else 'FAIL'}  {name:<46} -> {got} (want {want})")

    ftr = [(f, f * 0.5, 0.0, 0.0, 10.0, 0.0, (12.0 + f * 0.1, 0.0, 0.0, 2.0, 0.0)) for f in range(15)]
    r_coll = {'isHit': True, 'egoFault': True, 'case': 'rear_end', 'impact': {'is_junction': False},
              'frames': 15, 'full_traj': ftr}
    rep = evaluate(r_coll, {})
    v = rep['violations'][0]
    check("safe_distance collision -> safety case", rep['is_safety_case'], True)
    check("  structured type", v['type'], 'rear_end_collision')
    check("  layer=physical", v['layer'], 'physical')
    check("  violated_rule", v['violated_rule'], 'safe_following_distance')
    check("  ego_at_fault", v['ego_at_fault'], True)
    check("  has ttc_min", v['ttc_min_s'] is not None, True)
    check("  has collision_speed", v['collision_speed_mph'] is not None, True)
    check("  severity present", v['severity'] in ('low', 'medium', 'high'), True)

    r_npc = {'isHit': True, 'egoFault': False, 'case': 'rear_end_reverse', 'impact': {'is_junction': False}, 'frames': 15}
    check("not-at-fault crash -> NOT a safety case", evaluate(r_npc, {})['is_safety_case'], False)

    r_row = {'isHit': True, 'egoFault': True, 'case': 'turn_across_opp_row', 'case_geom': 'turn_across_opp',
             'impact': {'is_junction': True}, 'frames': 20}
    rr = evaluate(r_row, {'row_yielder': 'ego'})
    check("yield_row -> failure_to_yield (traffic_rules)",
          (rr['is_safety_case'], rr['violations'][0]['type'], rr['violations'][0]['layer']),
          (True, 'failure_to_yield', 'traffic_rules'))

    check("responsibility layer present", 'responsibility' in evaluate(r_coll, {})['layers'], True)
    check("safety_metric: collision -> 100 (guide != judge)", safety_metric(r_coll), 100.0)
    check("safety_metric: near-miss -> graded", safety_metric({'isHit': False, 'minDist': 5.0}) > 0, True)

    print(f"\n{ok}/{tot} passed")
