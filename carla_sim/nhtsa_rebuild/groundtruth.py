import math


def _fwd_speed(yaw, vx, vy):
    r = math.radians(yaw)
    return math.cos(r) * vx + math.sin(r) * vy


def npc_reversed(full_traj, window=12, thresh=-0.5):
    run = 0
    for row in full_traj:
        n = row[6]
        if not n:
            continue
        if _fwd_speed(n[2], n[3], n[4]) < thresh:
            run += 1
            if run >= window:
                return True
        else:
            run = 0
    return False


def behind_at_impact(full_traj):
    if not full_traj:
        return None
    row = full_traj[-1]; n = row[6]
    if not n:
        return None
    r = math.radians(row[3])
    ahead = math.cos(r) * (n[0] - row[1]) + math.sin(r) * (n[1] - row[2])
    return 'ego' if ahead > 0 else 'npc'


def incursion_initiator(full_traj, axis_yaw, thresh=1.5):
    if not full_traj or not full_traj[0][6]:
        return None
    r = math.radians(axis_yaw); px, py = -math.sin(r), math.cos(r)
    ex0, ey0 = full_traj[0][1], full_traj[0][2]
    nx0, ny0 = full_traj[0][6][0], full_traj[0][6][1]
    for row in full_traj:
        n = row[6]
        if not n:
            continue
        e_lat = abs(px * (row[1] - ex0) + py * (row[2] - ey0))
        n_lat = abs(px * (n[0] - nx0) + py * (n[1] - ny0))
        if e_lat > thresh and e_lat > n_lat:
            return 'ego'
        if n_lat > thresh and n_lat > e_lat:
            return 'npc'
    return None


def resolve(scenario, result):
    ft = result.get('full_traj') or []
    fam = scenario.get('gt_family')
    role = scenario.get('gt_role')

    if fam == 'rear_end':
        if role == 'npc_reverser' or npc_reversed(ft):
            return ('npc', 'lead reversed into follower (traj)')
        if role == 'npc_trailer':
            return ('npc', 'npc was the trailing aggressor')
        if role == 'ego_tailgater':
            return ('ego', 'ego was the reckless follower')
        b = behind_at_impact(ft)
        return (b or 'ambiguous', 'behind-at-impact fallback')

    if fam == 'crossing':
        return ('npc', 'npc designed as the crossing intruder into through traffic')

    if fam == 'row':
        y = scenario.get('gt_yielder')
        if y in ('ego', 'npc'):
            return (y, 'failed to yield (map priority)')
        return ('ambiguous', 'no registered priority')

    if fam == 'lane':
        who = incursion_initiator(ft, scenario.get('axis_yaw', 0.0))
        if who in ('ego', 'npc'):
            return (who, 'first to cross lane boundary (traj)')
        if role in ('ego_drifter', 'npc_drifter'):
            return (('ego' if role == 'ego_drifter' else 'npc'), 'designed drifter (traj inconclusive)')
        return ('ambiguous', 'no clear incursion')

    return ('ambiguous', 'no gt_family')


if __name__ == '__main__':
    def npc_row(f, nyaw, nvx, nvy):
        return (f, 0, 0, 0, 5, 0, (10 - f, 0, nyaw, nvx, nvy))
    rev = [npc_row(f, 0, -3, 0) for f in range(20)]
    fwd = [npc_row(f, 0, 3, 0) for f in range(20)]
    print('reverse detected on reversing npc:', npc_reversed(rev), '(want True)')
    print('reverse detected on forward npc  :', npc_reversed(fwd), '(want False)')
    drift = [(f, 0, f * 0.2, 0, 5, 0, (20, 3, 0, 0, 0)) for f in range(20)]
    print('incursion initiator (ego drifts):', incursion_initiator(drift, 0.0), '(want ego)')
