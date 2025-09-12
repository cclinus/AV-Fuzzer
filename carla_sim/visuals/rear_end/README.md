# Rear End

## Description
A crash type where two vehicles moving the same direction collide front-to-rear

## Category
Category 2: Same Trafficway, Same Direction

## ACC_CONFIG
- 201: Same Trafficway, Same Direction-Rear End, Trailing Vehicle
- 202: Same Trafficway, Same Direction-Rear End, Lead Vehicle 
- 203: Same Trafficway, Same Direction-Rear End, Other or Unknown

## Graphics
- rear: The ego vehicle (EV) travels down the road and enters a front-to-rear collision with the target vehicle (TV)

## Scenarios
- scenario_1.yaml:
    - Map: Town01
    - Description: The ego fails to stop for a halted NPC
    - Fault: EGO
    - ACC_CONFIG:
        - ego: 201
        - NPC: 202
- scenario_2.yaml:
    - Map: Town02
    - Description: An NPC fails to stop for the ego, which has halted for another NPC
    - Fault: NPC
    - ACC_CONFIG:
        - ego: 202
        - NPC: 201
- scenario_3.yaml:
    - Map: Town03
    - Description: An NPC fails to stop for the ego, which has halted for another NPC
    - Fault: NPC
    - ACC_CONFIG
        - ego: 202
        - NPC: 201

## Simulated

Images of a rear end crash simulated in CARLA, spawned from scenarios/scenario_3.yaml.

- collision_angle: An angled view of the crash
- collision_bird: A bird's-eye view of the crash, mirroring what is seen in the graphics directory
- collision_high: A higher-up view of the crash, similar to what is seen when using the AV-Fuzzer tool (albeit more zoomed in)