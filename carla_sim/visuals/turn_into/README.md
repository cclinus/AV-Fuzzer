# Turn Into Path: Turn Into Opposite Directions

## Description
A crash type where a vehicle attempts to turn into a lane of traffic and enters a collision with a vehicle traveling the opposite direction

## Category
Category 4: Change Trafficway Vehicle Turning

## ACC_CONFIG
- 412: Trafficway Vehicle Turning-Turn Into Path, Turn into Opposite Directions [Turning Right] 
- 413: Trafficway Vehicle Turning-Turn Into Path, Turn into Opposite Directions [Going Straight, Other Vehicle Turning Right] 
- 414: Trafficway Vehicle Turning-Turn Into Path, Turn into Opposite Directions [Turning Left]
- 415: Trafficway Vehicle Turning-Turn Into Path, Turn into Opposite Directions [Going Straight, Other Vehicle Turning Left]
- 416: Trafficway Vehicle Turning-Turn into Path, Other or Unknown 

## Graphics
- turn_before: The ego vehicle (EV) begins to make a left turn into a lane of traffic
- turn_crash: The EV has entered a crash with the target vehicle (TV)

## Scenarios
- scenario_1:
    - Map: Town01
    - Description: The ego runs a red light and gets hit by an NPC
    - Fault: ego
    - ACC_CONFIG
        - ego: 414
        - NPC: 415
- scenario_2:
    - Map: Town02
    - Description: The ego fails to yield during a right turn and hits an NPC
    - Fault: ego
    - ACC_CONFIG
        - ego: 412
        - NPC: 413
- scenario_3:
    - Map: Town03
    - Description: The ego fails to yield during a left turn and gets hit by an NPC
    - Fault: ego
    - ACC_CONFIG
        - ego: 414
        - NPC: 415

## Simulated

Images of a rear end crash simulated in CARLA, spawned from scenarios/scenario_2.yaml.

- collision_angle: An angled view of the crash
- collision_bird: A bird's-eye view of the crash, mirroring what is seen in the graphics directory
- collision_high: A higher-up view of the crash, similar to what is seen when using the AV-Fuzzer tool (albeit more zoomed in)