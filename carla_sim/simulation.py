import carla
import time
import math
import sys
import os
from datetime import datetime
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../PythonAPI')))
from agents.navigation.behavior_agent import BehaviorAgent
import liability
import tools
from collections import namedtuple


SimulationResult = namedtuple('SimulationResult', [
    'deltaDlist',  # List of per-NPC lists of deltaD over time
    'dList',       # List of per-NPC lists of raw distances over time
    'isHit',       # Whether a collision occurred
    'isEgoFault',  # If collision, whether ego vehicle was at fault
    'hitTime'      # Frame index when collision happened
])

def run_simulation(spawn_config, weather_params,
                   npc1_behaviors, npc2_behaviors,
                   tick_interval=0.05,
                   max_frames=500):


    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)
    client.load_world('Town03')
    world = client.get_world()
    tools.set_weather(world, weather_params)

    spectator = world.get_spectator()
    spectator.set_transform(carla.Transform(
        carla.Location(x=-50, y=0, z=100),
        carla.Rotation(pitch=-35, yaw=0, roll=0)))


    for actor in world.get_actors().filter('vehicle.*'):
        actor.destroy()
    for actor in world.get_actors().filter('walker.*'):
        actor.destroy()


    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter('vehicle.tesla.model3')[0]
    ev_tf   = tools.list_to_transform(spawn_config['ev']['start'])
    ev_end  = tools.list_to_transform(spawn_config['ev']['end']).location

    vehicle = world.spawn_actor(vehicle_bp, ev_tf)
    agent = BehaviorAgent(vehicle, behavior='normal')
    agent.set_destination(ev_end)


    npc1 = tools.spawn_npc(world, blueprint_library, tools.list_to_transform(spawn_config['npc1']['start']))
    npc2 = tools.spawn_npc(world, blueprint_library, tools.list_to_transform(spawn_config['npc2']['start']))

    walker1_bp = blueprint_library.find('walker.pedestrian.0001')
    walker2_bp = blueprint_library.find('walker.pedestrian.0002')

    ped1_start = tools.list_to_transform(spawn_config['pedestrian1']['start'])
    ped1_end = tools.list_to_location(spawn_config['pedestrian1']['end'])

    ped2_start = tools.list_to_transform(spawn_config['pedestrian2']['start'])
    ped2_end = tools.list_to_location(spawn_config['pedestrian2']['end'])

    walker1 = world.spawn_actor(walker1_bp, ped1_start)
    walker2 = world.spawn_actor(walker2_bp, ped2_start)

    controller_bp = blueprint_library.find('controller.ai.walker')
    controller1 = world.spawn_actor(controller_bp, carla.Transform(), attach_to=walker1)
    controller2 = world.spawn_actor(controller_bp, carla.Transform(), attach_to=walker2)

    controller1.start()
    controller1.go_to_location(ped1_end)
    controller2.start()
    controller2.go_to_location(ped2_end)

    collision_bp = blueprint_library.find('sensor.other.collision')
    collision_sensor = world.spawn_actor(collision_bp, carla.Transform(), attach_to=vehicle)
    isHit = False; isEgoFault = False; hitTime = None
    def on_collision(event):
        nonlocal isHit, isEgoFault, hitTime
        if isHit:
            return
        other = event.other_actor
        if ('vehicle' in event.actor.type_id and
            'vehicle' in other.type_id and
            (event.actor.id == vehicle.id or other.id == vehicle.id)):
            isHit = True
            hitTime = event.frame
            ev, npc = (event.actor, other) if event.actor.id == vehicle.id else (other, event.actor)
            isEgoFault = liability.is_ego_fault(ev, npc)
    collision_sensor.listen(on_collision)


    npc1.apply_control(npc1_behaviors[0])
    npc2.apply_control(npc2_behaviors[0])


    world.tick()
    time.sleep(tick_interval)
    _ = agent.run_step()

    deltaDlist, dList = [[],[]], [[],[]]
    lane_change_interval = int(1.0 / tick_interval)
    frame_count = 0

    while frame_count < max_frames:



        world.tick()

        if isHit or agent.done():
            break



        control = agent.run_step()
        vehicle.apply_control(control)


        if frame_count % lane_change_interval == 0:
            idx = frame_count // lane_change_interval
            if idx < len(npc1_behaviors):
                npc1.apply_control(npc1_behaviors[idx])
                npc2.apply_control(npc2_behaviors[idx])


        ego_loc = vehicle.get_location()
        ego_vel = vehicle.get_velocity()
        ego_speed = math.sqrt(ego_vel.x**2 + ego_vel.y**2 + ego_vel.z**2)
        brake_dist = max(0.0, 0.0467*ego_speed**2 + 0.4116*ego_speed - 1.9913 + 0.5)

        for i,npc in enumerate((npc1, npc2)):
            loc = npc.get_location()
            d = math.hypot(ego_loc.x-loc.x, ego_loc.y-loc.y)
            delta = d - 4.6 - brake_dist
            dList[i].append(d)
            deltaDlist[i].append(delta)

        frame_count += 1
        time.sleep(tick_interval)
    for actor in (vehicle, npc1, npc2):
        if actor: actor.destroy()
    if collision_sensor:
        collision_sensor.destroy()
    #print(frame_count)

    return SimulationResult(deltaDlist, dList, isHit, isEgoFault, hitTime)




def evaluate_individual(spawn_config, weather_params, individual,
                        tick_interval=0.05, max_frames=500):

    npc1_behaviors, npc2_behaviors = individual

    #start_time = time.time()


    result = run_simulation(
        spawn_config,
        weather_params,
        npc1_behaviors,
        npc2_behaviors,
        tick_interval=tick_interval,
        max_frames=max_frames
    )


    fitness = tools.find_fitness(
        result.deltaDlist,
        result.dList,
        result.isEgoFault,
        result.isHit,
        result.hitTime
    )

    #elapsed_time = time.time() - start_time
    #print(f" evaluate_individual {elapsed_time:.2f}")

    return fitness, result


