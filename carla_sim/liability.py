import carla
import math

buffer = 0.40

def in_degree_range(degree, low, high):
    return (low <= degree) and (degree <= high)

def is_straight(yaw):
    return not in_degree_range(yaw, 15, 345)

#takes a carla.Vector3D and returns a unit vector pointing in the same direction
def unit_vector(vec):
    length = dist(carla.Vector3D(0.0, 0.0, 0.0), vec)

    new_x = vec.x / length
    new_y = vec.y / length 
    new_z = vec.z / length

    return carla.Vector3D(new_x, new_y, new_z)

#distance between two 3D points
def dist(start, end):
    x_comp = (end.x - start.x) ** 2
    y_comp = (end.y - start.y) ** 2
    z_comp = (end.z - start.z) ** 2

    return math.sqrt(x_comp + y_comp + z_comp)

#rotate a carla.Location around the origin
def rotate_location(loc, yaw):
    r = math.radians(-1 * yaw)

    x = loc.x * math.cos(r) - loc.y * math.sin(r)
    y = loc.x * math.sin(r) + loc.y * math.cos(r)

    return carla.Location(x, y, loc.z)

def rotate_vector(vec, yaw):
    #-1 to account for lefthandedness
    r = math.radians(-1 * yaw)

    x = vec.x * math.cos(r) - vec.y * math.sin(r)
    y = vec.x * math.sin(r) + vec.y * math.cos(r)

    return carla.Vector3D(x, y, vec.z)

def adjust_to_lane(lane_tf, vehicle_tf):
    adj_tf = carla.Transform()

    adj_rot = carla.Rotation()
    adj_rot.yaw = (vehicle_tf.rotation.yaw - lane_tf.rotation.yaw) % 360

    adj_loc = carla.Location()
    adj_loc.x = vehicle_tf.location.x - lane_tf.location.x
    adj_loc.y = vehicle_tf.location.y - lane_tf.location.y
    adj_loc.z = vehicle_tf.location.z - lane_tf.location.z

    adj_tf.location = rotate_location(adj_loc, lane_tf.rotation.yaw % 360)
    adj_tf.rotation = adj_rot
    return adj_tf

#no clue
def turn_into(params, ego_params, npc_params):
    return (False, False)

#no clue
def turn_across_opp(params, ego_params, npc_params):
    return (False, False)


#main idea: determine the direction of the lane, whoever doesn't match is at fault
def head_on(params, ego_params, npc_params):

    
    return (False, False)

#main idea: if the contact point is in the front 2/3rds of the angled vehicle, it is at fault. otherwise, the straight vehicle is at fault
#this stems from the idea that, if the straight vehicle hits the back 1/3rd of the angled vehicle, it should have yielded
#this proportion can and should be adjusted to reflect reality

#it may also be pertinent to determine if the angle vehicle was even allowed to change lanes
#how to determine? need to find the rules of the lane the angle was coming from
#waypoints do have get_left_lane and get_right_lane, but those themselves account for lane change rules
#i.e. if the left lane was allowed to merge (for whatever reason) into the right, but not vice versa
#just running waypoint.get_left_lane() to find out the rules, would not work
def sideswipe(params, ego_params, npc_params):
    angle = None
    straight = None

    #if both vehicles are straight, this is not a sideswipe
    ego_yaw = ego_params['tf'].rotation.yaw
    npc_yaw = npc_params['tf'].rotation.yaw

    ego_straight = is_straight(ego_yaw)
    npc_straight = is_straight(npc_yaw)

    ego_angle = not ego_straight #simple aliasing for easier understanding
    
    
    if (ego_straight == npc_straight):
        return (False, False)
    
    #decide who is angled and who is straight
    if ego_angle:
        angle = ego_params
        straight = npc_params
    else:
        angle = npc_params
        straight = ego_params
    

    #if the angled vehicle is at an extreme angle, this is not a sideswipe
    angle_yaw = angle['tf'].rotation.yaw

    direction = angle['tf'].location.y < straight['tf'].location.y

    if direction:
        if not in_degree_range(angle_yaw, 15, 75):
            return (False, False)
    else:
        if not in_degree_range(angle_yaw, 285, 345):
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

    if angle_front - buffer < straight_back:
        return (False, False)


    #add some logic here about if lane change is allowed or not
    lane_change = params['lane_change']

    #if the angle vehicle hit behind the straight, clearly it is at fault
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

#main idea: if the lead vehicle comes to a very abrupt stop, it should be at fault. otherwise, the trailing vehicle is at fault
#TO-DO: develop a more robust idea. this is only one stage better than original
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

    lead_straight = is_straight(lead_yaw)
    trail_straight = is_straight(trail_yaw)

    #if the vehicles are not both sufficiently straight
    if not (lead_straight and trail_straight):
        return (False, False)
    
    lead_vertices = lead['box'].get_world_vertices(lead['tf'])
    trail_vertices = trail['box'].get_world_vertices(trail['tf'])

    lead_back = lead_vertices[0].x
    trail_front = trail_vertices[0].x

    lead_y_range = [lead_vertices[0].y, lead_vertices[0].y]
    trail_y_range = [trail_vertices[0].y, trail_vertices[0].y]

    for i in range(8):
        lead_v = lead_vertices[i]
        trail_v = trail_vertices[i]

        lead_back = min(lead_v.x, lead_back)
        trail_front = max(trail_v.x, trail_front)

        lead_y_range[0] = min(lead_v.y, lead_y_range[0])
        lead_y_range[1] = max(lead_v.y, lead_y_range[1])
        
        trail_y_range[0] = min(trail_v.y, trail_y_range[0])
        trail_y_range[1] = max(trail_v.y, trail_y_range[1])

    if trail_front - buffer > lead_back:
        return (False, False)
    
    #add logic about the lead coming to an abrupt stop

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

def is_ego_fault(ego, npc, waypoint):
    
    if not npc:
        return True
    
    lane_tf = waypoint.transform
    lane_yaw = lane_tf.rotation.yaw % 360
    lane_id = waypoint.lane_id
    lane_change = waypoint.lane_change

    ego_box = ego.bounding_box
    ego_tf = ego.get_transform()
    ego_acc = ego.get_acceleration()
    ego_vel = ego.get_velocity()

    #We adjust our frame of reference to the lane waypoint
    #So calculations can be simple
    #TO-DO: figure out how this works in junctions
    
    adj_ego = adjust_to_lane(lane_tf, ego_tf)
    adj_ego_acc = rotate_vector(ego_acc, lane_yaw)
    adj_ego_vel = rotate_vector(ego_vel, lane_yaw)

    npc_box = npc.bounding_box
    npc_tf = npc.get_transform()
    npc_acc = npc.get_acceleration()
    npc_vel = npc.get_velocity()

    adj_npc = adjust_to_lane(lane_tf, npc_tf)
    adj_npc_acc = rotate_vector(npc_acc, lane_yaw)
    adj_npc_vel = rotate_vector(npc_vel, lane_yaw)

    

    parameters = {
        "lane_id" : lane_id,
        "lane_change" : lane_change,
    }
    ego_parameters = {
        "box" : ego_box,
        "tf" : adj_ego,
        "acc" : adj_ego_acc,
        "vel" : adj_ego_vel,
    }

    npc_parameters = {
        "box" : npc_box,
        "tf" : adj_npc,
        "acc" : adj_npc_acc,
        "vel" : adj_npc_vel
    }

    cases = [sideswipe, rear_end]

    answer = False
    collision_case = "Unknown"
    
    for i in range(len(cases)):
        result = cases[i](parameters, ego_parameters, npc_parameters)

        #3a. If the scenario applies, return its determination
        if (result[0]):        
            answer = result[1]
            collision_case = cases[i].__name__
            break
    
    #scenario_debug(ego, npc, answer, collision_case, True)
    return (answer, collision_case)