import carla
import random
import time
import math
import sys
import os
import yaml
from datetime import datetime

def get_speed(actor):
    v = actor.get_velocity()
    return math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)

def spawn_npc(world, blueprint_library, spawn_point):
    npc_bp = blueprint_library.filter('vehicle.audi.tt')[0]
    npc_vehicle = world.try_spawn_actor(npc_bp, spawn_point)
    return npc_vehicle

def set_weather(world, weather_params):
    weather = carla.WeatherParameters(
        cloudiness=weather_params.get('cloudiness', 0.0),
        precipitation=weather_params.get('precipitation', 0.0),
        precipitation_deposits=weather_params.get('wetness', 0.0),
        sun_altitude_angle=weather_params.get('sun_altitude_angle', 70.0),
        wind_intensity=weather_params.get('wind_intensity', 0.0),
        fog_density=weather_params.get('fog_density', 0.0),
        fog_distance=weather_params.get('fog_distance', 0.0),
        fog_falloff=weather_params.get('fog_falloff', 0.1),
    )
    world.set_weather(weather)

def load_spawn_yaml(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)
def load_weather_yaml(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)
def list_to_location(lst):
    return carla.Location(x=lst[0], y=lst[1], z=lst[2])

def list_to_transform(lst):
    return carla.Transform(list_to_location(lst), carla.Rotation())

def load_ga_config(path):
    if not os.path.exists(path):
        raise FileNotFoundError(f"GA config not found: {path}")
    with open(path, 'r') as f:
        cfg = yaml.safe_load(f)

    return cfg


def spawn_pedestrian(world, blueprint_library, spawn_point, destination_location):
    walker_bp = random.choice(blueprint_library.filter('walker.pedestrian.*'))
    controller_bp = blueprint_library.find('controller.ai.walker')

    walker = world.try_spawn_actor(walker_bp, spawn_point)
    if not walker:
        print("Failed to spawn pedestrian.")
        return None, None

    controller = world.spawn_actor(controller_bp, carla.Transform(), attach_to=walker)
    controller.start()
    controller.go_to_location(destination_location)
    controller.set_max_speed(3 + random.random())


    return walker, controller

def brake_distance(speed_mps):

    d_brake = 0.0467 * speed_mps**2 + 0.4116 * speed_mps - 1.9913 + 0.5
    return max(d_brake, 0.0)


def euclidean_distance(loc1, loc2):
    dx = loc1.x - loc2.x
    dy = loc1.y - loc2.y
    dz = loc1.z - loc2.z
    return math.sqrt(dx**2 + dy**2 + dz**2)


def generate_npc_behaviors(npc_cfg, num_intervals, extra_steer_perturb=False):

    behaviors = []
    for _ in range(num_intervals):
        ctrl = carla.VehicleControl()

        ctrl.throttle = random.uniform(*npc_cfg['throttle_range'])
        ctrl.brake    = random.uniform(*npc_cfg['brake_range']) \
                        if random.random() < npc_cfg.get('brake_chance', 0) else 0.0
        ctrl.steer    = random.choice([-1, 1]) * random.uniform(*npc_cfg['steer_range'])

        if extra_steer_perturb:
            perturb = random.choice([-1, 1]) * random.uniform(0.0, 0.01)
            ctrl.steer = max(-1.0, min(1.0, ctrl.steer + perturb))
        behaviors.append(ctrl)
    return behaviors


def save_behaviors(filepath, npc1_behaviors, npc2_behaviors):
    os.makedirs(os.path.dirname(filepath), exist_ok=True)
    data = {
        'npc1': [
            {'throttle': b.throttle, 'brake': b.brake, 'steer': b.steer}
            for b in npc1_behaviors
        ],
        'npc2': [
            {'throttle': b.throttle, 'brake': b.brake, 'steer': b.steer}
            for b in npc2_behaviors
        ]
    }
    with open(filepath, 'w') as f:
        yaml.safe_dump(data, f)

def load_behaviors(filepath):
    with open(filepath) as f:
        data = yaml.safe_load(f)
    def dict_to_ctrl(d):
        ctrl = carla.VehicleControl()
        ctrl.throttle = d['throttle']
        ctrl.brake    = d['brake']
        ctrl.steer    = d['steer']
        return ctrl
    npc1 = [dict_to_ctrl(d) for d in data['npc1']]
    npc2 = [dict_to_ctrl(d) for d in data['npc2']]
    return npc1, npc2


def find_fitness(deltaDlist, dList, isEgoFault, isHit, hitTime,
                 weight_d=0.5, weight_deltaD=0.5, C=1.0):
    if isHit:
        return C * 2.0 if isEgoFault else 0.0


    end = hitTime if (isHit and hitTime is not None) else None


    min_deltaD = min(
        min(npc[:end]) for npc in deltaDlist if npc[:end]
    )
    min_d = min(
        min(npc[:end]) for npc in dList if npc[:end]
    )
    score = C - (weight_d * min_d + weight_deltaD * min_deltaD)
    return score

