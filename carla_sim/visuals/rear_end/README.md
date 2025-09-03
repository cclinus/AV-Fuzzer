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
    - Description: A crash on a residential two-way street where the ego vehicle (blue) has stopped for an NPC (red). The NPC then backs up and enters a rear-to-front collision with the ego vehicle
    - Fault: NPC
- scenario_2.yaml:
    - Map: Town02
    - Description: A crash on a residential two-way street where the ego vehicle (blue) stops for a halted NPC (red). A high-speed NPC (red) then enters a front-to-rear collision with the ego vehicle
    - Fault: NPC
- scenario_3.yaml:
    - Map: Town03
    - Description: A crash on an urban one-way street where the ego vehicle (blue) stops for a halted NPC (red). A medium-speed NPC (red) then enters a front-to-rear collision with the ego vehicle
    - Fault: NPC

Note: Due to the nature of the ADS (Carla ADS) used for simulating, it is difficult to obtain simulations of a rear end crash where the ego vehicle is at fault. For all intents and purposes, a scenario where the ego is at fault would look similar to scenario_1, but would have the ego move into the NPC, rather than the NPC back into the ego.

## Simulated

Images of a simulated rear end crash, spawned from scenarios/scenario_3.yaml.

- collision_angle: An angled view of the crash
- collision_bird: A bird's-eye view of the crash, mirroring what is seen in the graphics directory
- collision_high: A higher-up view of the crash, similar to what is seen when using the AV-Fuzzer tool (albeit more zoomed in)