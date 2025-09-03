# Turn Across Path: Initial Opposite Directions

## Description
A crash type where a vehicle attempts to turn across a lane of traffic and enters a collision with a vehicle traveling the opposite direction

## Category
Category 4: Change Trafficway Vehicle Turning

## ACC_CONFIG
- 401: Trafficway Vehicle Turning-Turn Across Path, Initial Opposite Directions [Left / Right] 
- 402: Trafficway Vehicle Turning-Turn Across Path, Initial Opposite Directions [Going Straight]
- 407: Trafficway Vehicle Turning-Turn Across Path, Other or Unknown

## Graphics
- turn_before: The ego vehicle (EV) begins to make a left turn across a lane of traffic flowing in the opposite direction
- turn_crash: The EV has entered a crash with the target vehicle (TV)

## Scenarios
- scenario_1
    - Map: Town01
    - Description: An NPC runs a red light and enters a crash where the ego vehicle was turning across the lane
    - Fault: NPC
    - ACC_CONFIG
        - ego: 401
        - NPC: 402
- scenario_2
    - Map: Town02
    - Description: The ego vehicle fails to yield for an NPC vehicle in an intersection
    - Fault: ego
    - ACC_CONFIG
        - ego: 401
        - NPC: 402
- scenario_3
    - Map: Town03
    - Description: An NPC fails to properly slow down and hits the ego
    - Fault: NPC
    - ACC_CONFIG
        - ego: 401
        - NPC: 402

## Simulated

Images of a rear end crash simulated in CARLA, spawned from scenarios/scenario_3.yaml.

- collision_angle: An angled view of the crash
- collision_bird: A bird's-eye view of the crash, mirroring what is seen in the graphics directory
- collision_high: A higher-up view of the crash, similar to what is seen when using the AV-Fuzzer tool (albeit more zoomed in)