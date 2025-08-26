import carla
import math

def is_crossed_line(vehicle_loc):
    y = vehicle_loc.y
    return abs(y) < 6.5
def in_degree_range(degree, low, high):
    return (low < degree) and (degree < high)
def ranges(box, trans):

    corners = box.get_world_vertices(trans)

    x_min = min(corners, key= lambda item: item.x).x
    x_max = max(corners, key= lambda item: item.x).x
    y_min = min(corners, key= lambda item: item.y).y
    y_max = max(corners, key= lambda item: item.y).y
    z_min = min(corners, key= lambda item: item.z).z
    z_max = max(corners, key= lambda item: item.z).z

    return [[x_min, x_max], [y_min, y_max], [z_min, z_max]]
   
def bounding_intersect(ego_ranges, npc_ranges, crash_buffer=0.35):
    #vehicles overlap partially during collision, tends to max out at 0.35 units
    x_ego_low = not (npc_ranges[0][1] - crash_buffer > ego_ranges[0][0])
    x_ego_high = not (ego_ranges[0][1] - crash_buffer > npc_ranges[0][0])
    x_intersect = not (x_ego_low or x_ego_high)

    y_ego_low = not (npc_ranges[1][1] - crash_buffer > ego_ranges[1][0])
    y_ego_high = not (ego_ranges[1][1] - crash_buffer > npc_ranges[1][0])
    y_intersect = not (y_ego_low or y_ego_high)
    
    z_ego_low = not(npc_ranges[2][1] - crash_buffer > ego_ranges[2][0])
    z_ego_high = not(ego_ranges[2][1] - crash_buffer > npc_ranges[2][0])
    z_intersect = not (z_ego_low or z_ego_high)
    
    return [x_intersect, y_intersect, z_intersect]

             
"""
STRUCTURE OF LIABILITY SCENARIO:
def example(ego_trans, ego_box, npc_trans, npc_box, crossed):

    if scenario condition:
    
        calculations/logic
        return (True, liability determination)

    return (False, False)

Liability scenarios return a tuple of two elements:

return[0] is if the scenario applies (if not, go-to next scenario)
return[1] is if ego is liable
"""

def side(ego_trans, ego_box, npc_trans, npc_box, crossed):

    ego_loc = ego_trans.location
    ego_ranges = ranges(ego_box, ego_trans)

    npc_loc = npc_trans.location
    npc_ranges = ranges(npc_box, npc_trans)

    if not bounding_intersect(ego_ranges, npc_ranges)[0]:
        return (False, False)
    
    if not crossed:
        if (ego_loc.x > npc_loc.x):
            return (True, False)
        else:
            npc_x_length = npc_ranges[0][1] - npc_ranges[0][0]
            npc_x_min = npc_ranges[0][0]
            ego_x_max = ego_ranges[0][1]

            #If the ego hit a back portion of the NPC car, it should have noticed the crossing and slowed down
            #Set to 50% initially, can be fine tuned for fault definitions
            if ego_x_max - npc_x_min < 0.5 * npc_x_length:
                return (True, True)
            else:
                return (True, False)
    else:
        if (ego_loc.x < npc_loc.x):
            return (True, True)
        else:
            ego_x_length = ego_ranges[0][1] - ego_ranges[0][0]
            ego_x_min = ego_ranges[0][0]
            npc_x_max = npc_ranges[0][1]

            #See above
            if npc_x_max - ego_x_min < 0.5 * ego_x_length:
                return (True, False)
            else:
                return (True, True)

def rear(ego_trans, ego_box, npc_trans, npc_box, crossed):

    #Get the location, yaw, and ranges for the ego
    ego_loc = ego_trans.location
    ego_yaw = npc_trans.rotation.yaw % 360
    ego_ranges = ranges(ego_box, ego_trans)

    #Get the location, yaw, and ranges for the NPC
    npc_loc = npc_trans.location
    npc_yaw = npc_trans.rotation.yaw % 360
    npc_ranges = ranges(npc_box, npc_trans)

    #If the cars do not overlap on the y-axis, it cannot be a rear-end collision
    #Known issue: hardcoded, must update to determine programatically
    #Possibly determined based on lane direction
    if not bounding_intersect(ego_ranges, npc_ranges)[1]:
        return (False, False)

    if (ego_loc.x > npc_loc.x):
        
        #If the NPC is not moving straight, it isn't a rear-end
        if not in_degree_range(npc_yaw, -15, 15):
            return (False, False)
        
        #Very simple
        #Known issue: does not take velocity or acceleration into account
        return (True, False)
    else:

        #If the ego is not moving straight, it isn't a rear-end
        if not in_degree_range(ego_yaw, -15, 15):
            return (False, False)
        
        #Very simple
        #Known issue: does not take velocity or acceleration into account
        return (True, True)

#Standard print messages for scenarios. Feel free to implement your own, more detailed, messages
def scenario_debug(ego, npc, crossed, fault, crash, detailed=False):
    print("--- CRASH LOG ---\n")

    print("Type: " + crash.upper() + "\n")
    print("--- MISC. DETAILS ---")
    c = "Ego "
    if (crossed):
        c = c + "crossed"
    else:
        c = c + "stayed"

    f = ""
    if (fault):
        f = "Ego"
    else:
        f = "NPC"
    print(c + "\n" + f + " fault\n")

    if detailed:
        print("--- EGO DETAILS ---")
        ego_trans = ego.get_transform()
        ego_box = ego.bounding_box
        ego_ranges = ranges(ego_box, ego_trans)

        print(f"Location:\n    X: {ego_trans.location.x:.4f}\n    Y: {ego_trans.location.y:.4f}\n    Z: {ego_trans.location.z:.4f}")
        print(f"Rotation:\n    Pitch: {ego_trans.rotation.pitch:.4f}\n    Roll: {ego_trans.rotation.roll:.4f}\n    Yaw: {ego_trans.rotation.yaw:.4f}")   
        print(f"Box Dimensions:\n    X: {2 * ego_box.extent.x:.4f}\n    Y: {2*ego_box.extent.y:.4f}\n    Z: {2*ego_box.extent.z:.4f}")
        print(f"Extent:\n    X Range: ({ego_ranges[0][0]:.4f}, {ego_ranges[0][1]:.4f})")
        print(f"    Y Range: ({ego_ranges[1][0]:.4f}, {ego_ranges[1][1]:.4f})")
        print(f"    Z Range: ({ego_ranges[2][0]:.4f}, {ego_ranges[2][1]:.4f})\n")

        print("--- NPC DETAILS ---")
        npc_trans = npc.get_transform()
        npc_box = npc.bounding_box
        npc_ranges = ranges(npc_box, npc_trans)

        print(f"Location:\n    X: {npc_trans.location.x:.4f}\n    Y: {npc_trans.location.y:.4f}\n    Z: {npc_trans.location.z:.4f}")
        print(f"Rotation:\n    Pitch: {npc_trans.rotation.pitch:.4f}\n    Roll: {npc_trans.rotation.roll:.4f}\n    Yaw: {npc_trans.rotation.yaw:.4f}")
        print(f"Box Dimensions:\n    X: {2 * npc_box.extent.x:.4f}\n    Y: {2*npc_box.extent.y:.4f}\n    Z: {2*npc_box.extent.z:.4f}")
        print(f"Extent:\n    X Range: ({npc_ranges[0][0]:.4f}, {npc_ranges[0][1]:.4f})")
        print(f"    Y Range: ({npc_ranges[1][0]:.4f}, {npc_ranges[1][1]:.4f})")
        print(f"    Z Range: ({npc_ranges[2][0]:.4f}, {npc_ranges[2][1]:.4f})")
    print("--- END CRASH ---\n\n\n")

def is_ego_fault(ego, npc):
    
    if not npc:
        return True

    #1. Calculate all the parameters needed for full liability determination

    ego_box = ego.bounding_box
    ego_trans = ego.get_transform()

    npc_box = npc.bounding_box
    npc_trans = npc.get_transform()

    crossed = is_crossed_line(ego_trans.location)


    #2. Add liability scenarios as first-class references to functions
    #Order is important, and determines the precedence of scenarios
    #In general, it's advisable to put more specific scenarios earlier in the list.
    #side and rear are the base cases, and should be put last (in that order)
    cases = [side, rear]

    answer = False
    collision_case = "Unknown"

    #3. Call each function in order
    for i in range(len(cases)):
        result = cases[i](ego_trans, ego_box, npc_trans, npc_box, crossed)

        #3a. If the scenario applies, return its determination
        if (result[0]):        
            answer = result[1]
            collision_case = cases[i].__name__
            break
    
    scenario_debug(ego, npc, crossed, answer, collision_case, True)
    return answer