# Head-On

## Description
A crash type where two vehicles moving opposite directions collide front-to-front

## Category
Category 3: Same Trafficway, Opposite Directions

## ACC_CONFIG
- 301: Same Trafficway, Opposite Direction-Lateral Move [Left / Right], Head-On, Sideswipe, or Angle
- 302: Same Trafficway, Opposite Direction-Lateral Move [Going Straight], Head-On, Sideswipe, or Angle
- 303: Same Trafficway, Opposite Direction-Lateral Move, Other or Unknown

## Graphics
- head_left: The ego vehicle (EV) performs a lateral move (left) and enters a front-to-front collision with the target vehicle (TV). In this case, the ego is at fault.
- head_straight: The ego vehicle (EV) perofrms a lateral move (straight) and enters a front-to-front collision with the target vehicle (TV). In this case, the ego is at fault.

## Scenarios
- scenario_1:
    - Map: Town01
    - Description: An NPC enters a head-on collision with the ego by driving into the flow of traffic
    - Fault: NPC
    - ACC_CONFIG
        - ego: 302
        - NPC: 302
- scenario_2:
    - Map: Town02
    - Description: The ego enters a head-on collision with an NPC by driving into the flow of traffic
    - Fault: ego
    - ACC_CONFIG
        - ego: 302
        - NPC: 302
- scenario_3:
    - Map: Town03
    - Description: An NPC enters a head-on collision with the ego by driving into the flow of traffic at an angle
    - Fault: NPC
    - ACC_CONFIG
        - ego: 302
        - NPC: 301

## Simulated

Images of a rear end crash simulated in CARLA, spawned from scenarios/scenario_1.yaml.

- collision_angle: An angled view of the crash
- collision_bird: A bird's-eye view of the crash, mirroring what is seen in the graphics directory
- collision_high: A higher-up view of the crash, similar to what is seen when using the AV-Fuzzer tool (albeit more zoomed in)