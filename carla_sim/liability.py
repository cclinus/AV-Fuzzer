import tools

BUFFER = 0.35

#Turn into collision type
#   visuals/turn_into
def turn_into(params, ego_params, npc_params):
    return (False, False)


#Turn across opposite lane collision type
#   visuals/turn_across
def turn_across_opp(params, ego_params, npc_params):
    return (False, False)


#Head-on collision type
#   visuals/head_on

#Main idea: determine the direction of the lane, whoever doesn't match is at fault
def head_on(params, ego_params, npc_params):

    head = None
    reverse = None

    ego_yaw = ego_params['tf'].rotation.yaw
    npc_yaw = npc_params['tf'].rotation.yaw

    ego_forward = tools.in_degree_range(ego_yaw, 0, 60) or tools.in_degree_range(ego_yaw, 300, 360)
    npc_forward = tools.in_degree_range(npc_yaw, 0, 60) or tools.in_degree_range(npc_yaw, 300, 360)

    #Consideration: As of writing this (9/10/2025), the allowed angles for rear_end and head_on are different. Should this not be the case?
    ego_reverse = tools.in_degree_range(ego_yaw, 120, 240)
    npc_reverse = tools.in_degree_range(npc_yaw, 120, 240)


    #at least one of them has to be turned around, but they both cant be turned around
    if not (ego_reverse ^ npc_reverse):
        return (False, False)
    
    #at least one of them has to be going forward, but they cant both be going forward
    if not (ego_forward ^ npc_forward):
        return (False, False)
    
    if ego_reverse:
        reverse = ego_params
        head = npc_params
    else:
        reverse = npc_params
        head = ego_params
    
    #If the reverse is behind the head, this isn't a head-on
    if reverse['tf'].location.x < head['tf'].location.x:
        return (False, False)
    
    head_vertices = head['box'].get_world_vertices(head['tf'])
    reverse_vertices = reverse['box'].get_world_vertices(reverse['tf'])

    head_front = head_vertices[0].x
    reverse_back = reverse_vertices[0].x

    for i in range(8):
        head_v = head_vertices[i]
        reverse_v = reverse_vertices[i]

        head_front = max(head_v.x, head_front)
        reverse_back = min(reverse_v.x, reverse_back)

    #Loose check for the reverse actually being in "front" of the head
    if head_front - BUFFER > reverse_back:
        return (False, False)
    
    return (True, ego_reverse)


#Sideswipe collision type
#   visuals/side_swipe

#Main idea: if the contact point is in the front 2/3rds of the angled vehicle, it is at fault. otherwise, the straight vehicle is at fault
#TO-DO: determine if the angle vehicle was even allowed to change lanes
def sideswipe(params, ego_params, npc_params):
    angle = None
    straight = None

    #If both vehicles are straight, this is not a sideswipe
    ego_yaw = ego_params['tf'].rotation.yaw
    npc_yaw = npc_params['tf'].rotation.yaw

    ego_straight = tools.is_straight(ego_yaw)
    npc_straight = tools.is_straight(npc_yaw)

    ego_angle = not ego_straight
    
    if (ego_straight == npc_straight):
        return (False, False)
    
    #Decide who is angled and who is straight
    if ego_angle:
        angle = ego_params
        straight = npc_params
    else:
        angle = npc_params
        straight = ego_params
    

    #If the angled vehicle is at an extreme angle, this is not a sideswipe
    angle_yaw = angle['tf'].rotation.yaw

    direction = angle['tf'].location.y < straight['tf'].location.y

    if direction:
        if not tools.in_degree_range(angle_yaw, 15, 75):
            return (False, False)
    else:
        if not tools.in_degree_range(angle_yaw, 285, 345):
            return (False, False)

    
    #if the angled vehicle is completely behind the straight vehicle, this is not a sideswipe
    angle_vertices = angle['box'].get_world_vertices(angle['tf'])
    angle_front = angle_vertices[0].x
    angle_back = angle_vertices[0].x

    straight_vertices = straight['box'].get_world_vertices(straight['tf'])
    straight_front = straight_vertices[0].x
    straight_back = straight_vertices[0].x

    for i in range(8):
        angle_v = angle_vertices[i]
        straight_v = straight_vertices[i]

        angle_front = max(angle_v.x, angle_front)
        angle_back = min(angle_v.x, angle_back)
        straight_front = max(straight_v.x, straight_front)
        straight_back = min(straight_v.x, straight_back)

    if angle_front - BUFFER < straight_back:
        return (False, False)


    #add some logic here about if lane change is allowed or not
    lane_change = params['lane_change']
    #
    #
    #

    #If the angle vehicle hit behind the straight, clearly it is at fault
    angle_loc = angle['tf'].location
    straight_loc = straight['tf'].location

    if straight_loc.x > angle_loc.x:
        return (True, ego_angle)
    
    angle_len = angle_front - angle_back

    overlap = max(straight_front - angle_back, 0)
    
    back_portion = (overlap / angle_len) <= (1/3)

    if back_portion:
        return (True, ego_straight)
    else:
        return (True, npc_straight)


#Rear-end collision type
#   visuals/rear_end

#Main idea: if the lead vehicle comes to a very abrupt stop, it should be at fault. otherwise, the trailing vehicle is at fault
#TO-DO: develop a more robust idea. this is only one stage better than original
#TO-DO: add more logic about the lead coming to an abrupt stop
def rear_end(params, ego_params, npc_params):

    ego_ahead = npc_params['tf'].location.x < ego_params['tf'].location.x

    lead = None
    trail = None

    if ego_ahead:
        lead = ego_params
        trail = npc_params
    else:
        lead = npc_params
        trail = ego_params

    lead_yaw = lead['tf'].rotation.yaw
    trail_yaw = trail['tf'].rotation.yaw

    lead_straight = tools.is_straight(lead_yaw)
    trail_straight = tools.is_straight(trail_yaw)

    #If the vehicles are not both straight, this isn't a rear-end
    #Consideration: As of writing this (9/10/2025), the allowed angles for rear_end and head_on are different. Should this not be the case?
    if not (lead_straight and trail_straight):
        return (False, False)
    
    lead_vertices = lead['box'].get_world_vertices(lead['tf'])
    trail_vertices = trail['box'].get_world_vertices(trail['tf'])

    lead_back = lead_vertices[0].x
    trail_front = trail_vertices[0].x

    lead_left = lead_vertices[0].y
    lead_right = lead_vertices[0].y

    trail_left = trail_vertices[0].y
    trail_right = trail_vertices[0].y

    for i in range(8):
        lead_v = lead_vertices[i]
        trail_v = trail_vertices[i]

        lead_back = min(lead_v.x, lead_back)
        trail_front = max(trail_v.x, trail_front)

        lead_left = min(lead_v.y, lead_left)
        lead_right = max(lead_v.y, lead_right)

        trail_left = min(trail_v.y, trail_left)
        trail_right = max(trail_v.y, trail_right)

    #Loose check for the trail actually being "behind" the lead
    """if trail_front - BUFFER > lead_back:
        return (False, False)"""

    """if lead_right - BUFFER < trail_left or trail_right - BUFFER < lead_left:
        return (False, False)"""

    lead_acc_avg = 0

    lead_history = list(lead['hist'])[-10::]

    for frame in lead_history:
        lead_acc_avg += frame['acc'].x
    lead_acc_avg /= 10

    if (lead_acc_avg <= -1.5 * lead['box'].extent.x):
        return (True, ego_ahead)

    return (True, not ego_ahead)


#Standard print messages for scenarios. Feel free to implement your own, more detailed, messages
def scenario_debug(ego, npc, fault, crash, detailed=True):
    print("--- CRASH LOG ---\n")

    print("Type: " + crash.upper() + "\n")
    print("--- MISC. DETAILS ---")
    f = ""
    if (fault):
        f = "Ego"
    else:
        f = "NPC"
    print(f + " fault\n")

    if detailed:
        print("--- EGO DETAILS ---")
        ego_trans = ego.get_transform()
        ego_box = ego.bounding_box
        ego_acc = ego.get_acceleration()
        ego_vel = ego.get_velocity()

        print(f"Location:\n    X: {ego_trans.location.x:.4f}\n    Y: {ego_trans.location.y:.4f}\n    Z: {ego_trans.location.z:.4f}")
        print(f"Rotation:\n    Pitch: {ego_trans.rotation.pitch:.4f}\n    Roll: {ego_trans.rotation.roll:.4f}\n    Yaw: {ego_trans.rotation.yaw:.4f}")   

        print(f"Box Dimensions:\n    X: {2 * ego_box.extent.x:.4f}\n    Y: {2*ego_box.extent.y:.4f}\n    Z: {2*ego_box.extent.z:.4f}")

        print(f"Velocity:\n    X: {ego_vel.x:.4f}\n    Y: {ego_vel.y:.4f}\n    Z: {ego_vel.z:.4f}")
        print(f"Acceleration:\n    X: {ego_acc.x:.4f}\n    Y: {ego_acc.y:.4f}\n     Z: {ego_acc.z:.4f}")

        print("--- NPC DETAILS ---")
        npc_trans = npc.get_transform()
        npc_box = npc.bounding_box
        npc_acc = npc.get_acceleration()
        npc_vel = npc.get_velocity()

        print(f"Location:\n    X: {npc_trans.location.x:.4f}\n    Y: {npc_trans.location.y:.4f}\n    Z: {npc_trans.location.z:.4f}")
        print(f"Rotation:\n    Pitch: {npc_trans.rotation.pitch:.4f}\n    Roll: {npc_trans.rotation.roll:.4f}\n    Yaw: {npc_trans.rotation.yaw:.4f}")

        print(f"Box Dimensions:\n    X: {2 * npc_box.extent.x:.4f}\n    Y: {2*npc_box.extent.y:.4f}\n    Z: {2*npc_box.extent.z:.4f}")

        print(f"Velocity:\n    X: {npc_vel.x:.4f}\n    Y: {npc_vel.y:.4f}\n    Z: {npc_vel.z:.4f}")
        print(f"Acceleration:\n    X: {npc_acc.x:.4f}\n    Y: {npc_acc.y:.4f}\n    Z: {npc_acc.z:.4f}")


    print("--- END CRASH ---\n\n")


#Fault determination

#Main idea: Collect data, run through each liability case in order
#TO-DO: figure out how adjusting works in junctions
def is_ego_fault(ego, ego_history, npc, npc_history, waypoint):
    
    if not npc:
        return True
    
    lane_tf = waypoint.transform
    lane_yaw = lane_tf.rotation.yaw % 360
    lane_id = waypoint.lane_id
    lane_change = waypoint.lane_change

    ego_box = ego.bounding_box
    ego_tf = ego.get_transform()

    #We adjust our frame of reference to the lane waypoint
    #So calculations can be simple
    adj_ego = tools.adjust_to_lane(lane_tf, ego_tf)

    ego_hist = []
    
    for e in list(ego_history):
        frame = {}
        frame['tf'] = tools.adjust_to_lane(lane_tf, e.transform)
        frame['vel'] = tools.rotate_vector(e.vector, lane_yaw)
        frame['acc'] = tools.rotate_vector(e.acceleration, lane_yaw)

        ego_hist.append(frame)

    npc_box = npc.bounding_box
    npc_tf = npc.get_transform()

    adj_npc = tools.adjust_to_lane(lane_tf, npc_tf)

    npc_hist = []
    
    for n in list(npc_history):
        frame = {}
        frame['tf'] = tools.adjust_to_lane(lane_tf, n.transform)
        frame['vel'] = tools.rotate_vector(n.vector, lane_yaw)
        frame['acc'] = tools.rotate_vector(n.acceleration, lane_yaw)

        npc_hist.append(frame)
        

    parameters = {
        "lane_id" : lane_id,
        "lane_change" : lane_change,
    }
    ego_parameters = {
        "box" : ego_box,
        "tf" : adj_ego,
        "hist" : ego_hist
    }

    npc_parameters = {
        "box" : npc_box,
        "tf" : adj_npc,
        "hist" : npc_history
    }

    cases = [head_on, sideswipe, rear_end]

    answer = False
    collision_case = "unknown"
    
    for i in range(len(cases)):
        result = cases[i](parameters, ego_parameters, npc_parameters)

        #3a. If the scenario applies, return its determination
        if (result[0]):        
            answer = result[1]
            collision_case = cases[i].__name__
            break
    
    #scenario_debug(ego, npc, answer, collision_case, True)
    return (answer, collision_case)

def is_ego_fault_test(ego, npc, waypoint):
    
    if not npc:
        return True
    
    lane_tf = waypoint['tf']
    lane_yaw = lane_tf.rotation.yaw % 360
    lane_id = waypoint['id']
    lane_change = waypoint['change']

    ego_box = ego['box']
    ego_tf = ego['tf']

    
    adj_ego = tools.adjust_to_lane(lane_tf, ego_tf)

    ego_hist = []
    
    for e in list(ego['hist']):
        frame = {}
        frame['tf'] = tools.adjust_to_lane(lane_tf, e.transform)
        frame['vel'] = tools.rotate_vector(e.vector, lane_yaw)
        frame['acc'] = tools.rotate_vector(e.acceleration, lane_yaw)

        ego_hist.append(frame)


    npc_box = npc['box']
    npc_tf = npc['tf']


    adj_npc = tools.adjust_to_lane(lane_tf, npc_tf)

    npc_hist = []

    for n in list(npc['hist']):
        frame = {}
        frame['tf'] = tools.adjust_to_lane(lane_tf, n.transform)
        frame['vel'] = tools.rotate_vector(n.vector, lane_yaw)
        frame['acc'] = tools.rotate_vector(n.acceleration, lane_yaw)

        npc_hist.append(frame)

    parameters = {
        "lane_id" : lane_id,
        "lane_change" : lane_change,
    }
    ego_parameters = {
        "box" : ego_box,
        "tf" : adj_ego,
        "hist" : ego_hist
    }

    npc_parameters = {
        "box" : npc_box,
        "tf" : adj_npc,
        "hist" : npc_hist
    }

    cases = [head_on, sideswipe, rear_end]

    answer = False
    collision_case = "unknown"
    
    for i in range(len(cases)):
        result = cases[i](parameters, ego_parameters, npc_parameters)

        #3a. If the scenario applies, return its determination
        if (result[0]):        
            answer = result[1]
            collision_case = cases[i].__name__
            break
    
    #scenario_debug(ego, npc, answer, collision_case, True)
    return (answer, collision_case)