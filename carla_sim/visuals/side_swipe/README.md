# Sideswipe

## Description
A crash type where two vehicles traveling in the same direction collide in a sideswipe

## Category
Category 2: Same Trafficway, Same Direction

## ACC_CONFIG
- 207: Same Trafficway, Same Direction-Sideswipe, Angle, Vehicle on Left 
- 208: Same Trafficway, Same Direction-Sideswipe, Angle, Vehicle on Right
- 209: Same Trafficway, Same Direction-Sideswipe, Angle, Other or Unknown

## Graphics
- sideswipe: The ego vehicle (EV) performs a lane-change and enters a sideswipe collision with the target vehicle (TV)

## Scenarios
- scenario_1:
    - Map: Town01
    - Description: An NPC fails to stop and collides with the ego's side
    - Fault: NPC
    - ACC_CONFIG
        - ego: 208
        - NPC: 207
- scenario_2:
    - Map: Town02
    - Description: An NPC fails to stop and sideswipes the ego
    - Fault: NPC
    - ACC_CONFIG
        - ego: 208
        - NPC: 207
- scenario_3:
    - Map: Town03
    - Description: The ego fails to stop and collides with an NPC's side
    - Fault: ego
    - ACC_CONFIG
        - ego: 207
        - NPC: 208

## Simulated

Images of a rear end crash simulated in CARLA, spawned from scenarios/scenario_3.yaml.

- collision_angle: An angled view of the crash
- collision_bird: A bird's-eye view of the crash, mirroring what is seen in the graphics directory
- collision_high: A higher-up view of the crash, similar to what is seen when using the AV-Fuzzer tool (albeit more zoomed in)