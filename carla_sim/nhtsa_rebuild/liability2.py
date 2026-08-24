import math

def norm360(a): return a % 360.0

def in_range(yaw, lo, hi):
    y = norm360(yaw)
    return lo <= y <= hi

def is_forward(yaw):
    return in_range(yaw, 0, 60) or in_range(yaw, 300, 360)

def is_opposing(yaw):
    return in_range(yaw, 120, 240)

def is_crossing(yaw):
    return in_range(yaw, 60, 120) or in_range(yaw, 240, 300)

def is_straight(yaw):
    return is_forward(yaw) or is_opposing(yaw)

def lane_angle_diff(actor_yaw, lane_yaw):
    return abs(((actor_yaw - lane_yaw + 90.0) % 180.0) - 90.0)


def adjust_to_lane(lane_x, lane_y, lane_yaw, ax, ay, ayaw):
    th = math.radians(lane_yaw)
    dx, dy = ax - lane_x, ay - lane_y
    xp = dx * math.cos(-th) - dy * math.sin(-th)
    yp = dx * math.sin(-th) + dy * math.cos(-th)
    return xp, yp, norm360(ayaw - lane_yaw)


def _overlap(a, b):
    reach_a = max(a['half_len'], a['half_wid'])
    reach_b = max(b['half_len'], b['half_wid'])
    d = math.hypot(a['x'] - b['x'], a['y'] - b['y'])
    return d <= reach_a + reach_b + 2.0

def head_on(ego, npc):
    ef, nf = is_forward(ego['yaw']), is_forward(npc['yaw'])
    er, nr = is_opposing(ego['yaw']), is_opposing(npc['yaw'])
    if not (er ^ nr):   return (False, False)
    if not (ef ^ nf):   return (False, False)
    if not _overlap(ego, npc): return (False, False)
    return (True, er)

def rear_end(ego, npc):
    if not (is_forward(ego['yaw']) and is_forward(npc['yaw'])): return (False, False)
    if not _overlap(ego, npc): return (False, False)
    ego_behind = ego['x'] < npc['x']
    return (True, ego_behind)

def turn_across_opp(ego, npc):
    ec, nc = is_crossing(ego['yaw']), is_crossing(npc['yaw'])
    if not (ec ^ nc): return (False, False)
    through = npc if ec else ego
    if not is_straight(through['yaw']): return (False, False)
    if not _overlap(ego, npc): return (False, False)
    return (True, ec)

def _shallow(yaw):
    return in_range(yaw, 10, 70) or in_range(yaw, 290, 350)

def sideswipe(ego, npc):
    ea, na = _shallow(ego['yaw']), _shallow(npc['yaw'])
    if ea == na: return (False, False)
    other = npc if ea else ego
    if not is_forward(other['yaw']): return (False, False)
    if not _overlap(ego, npc): return (False, False)
    return (True, ea)

def turn_into(ego, npc):
    er, nr = is_opposing(ego['yaw']), is_opposing(npc['yaw'])
    ef, nf = is_forward(ego['yaw']), is_forward(npc['yaw'])
    if not (er ^ nr): return (False, False)
    if not (ef ^ nf): return (False, False)
    if not _overlap(ego, npc): return (False, False)
    return (True, er)

CASES = [head_on, sideswipe, rear_end, turn_across_opp, turn_into]
ALIGN_THRESH = 50.0


def classify(ego, npc):
    for c in CASES:
        applies, fault = c(ego, npc)
        if applies:
            return fault, c.__name__
    return False, "unknown"


def _reversing(yaw, vel):
    if vel is None:
        return False
    r = math.radians(yaw)
    return (math.cos(r) * vel[0] + math.sin(r) * vel[1]) < -0.5


def is_ego_fault(ego_pose, npc_pose, lane_pose, ego_boxes, npc_boxes,
                 ego_on_road=True, is_junction=False, ego_vel=None, npc_vel=None,
                 row_yielder=None):
    ex, ey, eyaw = adjust_to_lane(lane_pose[0], lane_pose[1], lane_pose[2], *ego_pose)
    nx, ny, nyaw = adjust_to_lane(lane_pose[0], lane_pose[1], lane_pose[2], *npc_pose)
    ego = {'x': ex, 'y': ey, 'yaw': eyaw, 'half_len': ego_boxes[0], 'half_wid': ego_boxes[1]}
    npc = {'x': nx, 'y': ny, 'yaw': nyaw, 'half_len': npc_boxes[0], 'half_wid': npc_boxes[1]}
    fault, case = classify(ego, npc)
    if case == 'rear_end':
        front_is_ego = ex >= nx
        f_yaw = ego_pose[2] if front_is_ego else npc_pose[2]
        f_vel = ego_vel if front_is_ego else npc_vel
        if _reversing(f_yaw, f_vel):
            fault = front_is_ego
            case = 'rear_end_reverse'
    if row_yielder in ('ego', 'npc') and (is_junction or case in ('turn_across_opp', 'turn_into')):
        fault = (row_yielder == 'ego')
        case = case.replace('_veer', '') + '_row'
    if fault:
        aligned = is_junction or (lane_angle_diff(ego_pose[2], lane_pose[2]) <= ALIGN_THRESH)
        if not (ego_on_road and aligned):
            return False, case + "_veer"
    return fault, case


if __name__ == '__main__':
    L = (0.0, 0.0, 0.0)
    BOX = (2.4, 1.0)
    ok = 0; tot = 0
    def check(name, got, want):
        global ok, tot; tot += 1
        good = got == want
        ok += good
        print(f"  {'PASS' if good else 'FAIL'}  {name:<38} -> {got}  (want {want})")

    check("head_on: ego fwd vs npc opposing",
          is_ego_fault((0,0,0), (3.0,0,180), L, BOX, BOX), (False, 'head_on'))
    check("head_on: ego wrong-way -> ego fault",
          is_ego_fault((3.0,0,180), (0,0,0), L, BOX, BOX), (True, 'head_on'))
    check("rear_end: ego trailing -> ego fault",
          is_ego_fault((0,0,0), (4.0,0,0), L, BOX, BOX), (True, 'rear_end'))
    check("rear_end: ego lead -> npc fault",
          is_ego_fault((4.0,0,0), (0,0,0), L, BOX, BOX), (False, 'rear_end'))
    check("turn_across: perpendicular on straight road -> veer",
          is_ego_fault((0,0,90), (2.0,0,0), L, BOX, BOX), (False, 'turn_across_opp_veer'))
    check("veer: ego off-road -> stripped",
          is_ego_fault((0,0,90), (2.0,0,0), L, BOX, BOX, ego_on_road=False), (False, 'turn_across_opp_veer'))
    check("junction: crossing at junction stays fault",
          is_ego_fault((0,0,90), (2.0,0,0), L, BOX, BOX, is_junction=True), (True, 'turn_across_opp'))
    check("rear_end: lead reversing -> npc fault",
          is_ego_fault((0,0,0), (4.0,0,0), L, BOX, BOX, ego_vel=(5,0), npc_vel=(-3,0)), (False, 'rear_end_reverse'))
    check("rear_end: lead forward -> ego fault",
          is_ego_fault((0,0,0), (4.0,0,0), L, BOX, BOX, ego_vel=(5,0), npc_vel=(2,0)), (True, 'rear_end'))
    check("row: ego must yield -> ego fault",
          is_ego_fault((0,0,0), (2.0,0,90), L, BOX, BOX, is_junction=True, row_yielder='ego'), (True, 'turn_across_opp_row'))
    check("row: ego had priority -> npc fault",
          is_ego_fault((0,0,0), (2.0,0,90), L, BOX, BOX, is_junction=True, row_yielder='npc'), (False, 'turn_across_opp_row'))
    print(f"\n{ok}/{tot} passed")
