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

def main(spawn_config, weather_params):

    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)
    client.load_world('Town03')
    world = client.get_world()
    tools.set_weather(world, weather_params)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_filename = f"./logs/carla_log_{timestamp}.csv"
    log_file = open(log_filename, mode='w', newline='')


    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter('vehicle.tesla.model3')[0]

    ev_start = tools.list_to_transform(spawn_config['ev']['start'])
    ev_end = tools.list_to_transform(spawn_config['ev']['end'])

    npc1_start = tools.list_to_transform(spawn_config['npc1']['start'])
    npc2_start = tools.list_to_transform(spawn_config['npc2']['start'])

    pd1_start = tools.list_to_transform(spawn_config['pedestrian1']['start'])
    pd1_end = tools.list_to_location(spawn_config['pedestrian1']['end'])

    pd2_start = tools.list_to_transform(spawn_config['pedestrian2']['start'])
    pd2_end = tools.list_to_location(spawn_config['pedestrian2']['end'])


    npc1_cfg = spawn_config['npc1']
    npc2_cfg = spawn_config['npc2']


    for actor in world.get_actors().filter('vehicle.*'):
        actor.destroy()
    for actor in world.get_actors().filter('walker.*'):
        actor.destroy()

    max_frames = 500
    tick_interval = 0.05
    lane_change_interval = int(1.0 / tick_interval)
    num_intervals = max_frames // lane_change_interval + 1


    for trial_idx in range(500):

        npc1_behaviors = tools.generate_npc_behaviors(npc1_cfg, num_intervals, extra_steer_perturb=False)
        npc2_behaviors = tools.generate_npc_behaviors(npc2_cfg, num_intervals, extra_steer_perturb=False)

        behaviors_dir = './behaviors'
        behaviors_file = os.path.join(
            behaviors_dir,
            f"trial{trial_idx + 1}_{timestamp}.yaml"
        )
        # 只保存，不加载
        tools.save_behaviors(behaviors_file, npc1_behaviors, npc2_behaviors)
        log_file.write(f"Also saved behaviors to {behaviors_file}\n")

        deltaD_tracker = {
            "min_deltaD": float('inf'),
            "min_deltaD_frame": -1,
        }

        log_file.write("\n")
        log_file.write(f"Trial Index: {trial_idx + 1}\n")
        log_file.write(f"Start Time: {timestamp}\n")
        print(f"\n Trial {trial_idx + 1}")

        log_file.write(f"Trial {trial_idx + 1} — Pre-generated NPC behaviors\n")
        log_file.write("NPC1 behaviors:\n")
        for idx, ctrl in enumerate(npc1_behaviors):
            log_file.write(
                f"  Interval {idx}: throttle={ctrl.throttle:.2f}, "
                f"brake={ctrl.brake:.2f}, steer={ctrl.steer:.2f}\n"
            )
        log_file.write("NPC2 behaviors:\n")
        for idx, ctrl in enumerate(npc2_behaviors):
            log_file.write(
                f"  Interval {idx}: throttle={ctrl.throttle:.2f}, "
                f"brake={ctrl.brake:.2f}, steer={ctrl.steer:.2f}\n"
            )
        log_file.write("\n")

        collision_occurred = False
        client.reload_world()
        world = client.get_world()
        blueprint_library = world.get_blueprint_library()

        spectator = world.get_spectator()
        spectator.set_transform(carla.Transform(
            carla.Location(x=-50, y=0, z=50),
            carla.Rotation(pitch=-35, yaw=0, roll=0)))

        vehicle = world.spawn_actor(vehicle_bp, ev_start)
        agent = BehaviorAgent(vehicle, behavior='normal')
        agent.set_destination(ev_end.location)

        npc1 = tools.spawn_npc(world, blueprint_library, npc1_start)
        npc2 = tools.spawn_npc(world, blueprint_library, npc2_start)

        walker1, controller1 = tools.spawn_pedestrian(world, blueprint_library, pd1_start, pd1_end)
        walker2, controller2 = tools.spawn_pedestrian(world, blueprint_library, pd2_start, pd2_end)

        def on_collision(event):
            nonlocal collision_occurred
            other = event.other_actor

            print(f" Collision: {event.actor.type_id} hit {other.type_id} at frame {event.frame}")

            # 确保至少有一个是 EV
            if "vehicle" in event.actor.type_id and "vehicle" in other.type_id:
                # 判断哪个是 EV
                if event.actor.id == vehicle.id:
                    ev = event.actor
                    npc = other
                elif other.id == vehicle.id:
                    ev = other
                    npc = event.actor
                else:
                    print(" Collision does not involve ego vehicle. Ignored.")
                    return

                # 获取位置、速度和朝向
                ev_loc = ev.get_location()
                ev_rot = ev.get_transform().rotation
                ev_vel = ev.get_velocity()
                ev_speed = math.sqrt(ev_vel.x ** 2 + ev_vel.y ** 2 + ev_vel.z ** 2)

                npc_loc = npc.get_location()
                npc_rot = npc.get_transform().rotation
                npc_vel = npc.get_velocity()
                npc_speed = math.sqrt(npc_vel.x ** 2 + npc_vel.y ** 2 + npc_vel.z ** 2)

                # 判断责任
                fault = liability.is_ego_fault(ev, npc)
                collision_occurred = True

                print(f" EV at ({ev_loc.x:.2f}, {ev_loc.y:.2f}) yaw={ev_rot.yaw:.1f} speed={ev_speed:.2f} m/s")
                print(f" NPC at ({npc_loc.x:.2f}, {npc_loc.y:.2f}) yaw={npc_rot.yaw:.1f} speed={npc_speed:.2f} m/s")
                print(f" Liability: {'EGO FAULT' if fault else 'NPC FAULT'}")

                # 写入日志
                log_file.write(f"[Frame {event.frame}] Collision: {ev.type_id} vs {npc.type_id}\n")
                log_file.write(
                    f"EV   Pos=({ev_loc.x:.2f}, {ev_loc.y:.2f}), Yaw={ev_rot.yaw:.1f}, Speed={ev_speed:.2f} m/s\n")
                log_file.write(
                    f"NPC  Pos=({npc_loc.x:.2f}, {npc_loc.y:.2f}), Yaw={npc_rot.yaw:.1f}, Speed={npc_speed:.2f} m/s\n")
                log_file.write(f"Liability: {'EGO' if fault else 'NPC'}\n\n")

                return

            else:
                # 如果是 pedestrian / static object
                collision_occurred = True
                print(" Undecidable collision (e.g., pedestrian or static object).")
                log_file.write(f"[Frame {event.frame}] Collision: {event.actor.type_id} vs {other.type_id}\n")
                log_file.write("Liability: UNDECIDED (non-vehicle)\n\n")

                return

        collision_sensor_bp = blueprint_library.find('sensor.other.collision')
        collision_sensor = world.spawn_actor(collision_sensor_bp, carla.Transform(), attach_to=vehicle)
        collision_sensor.listen(lambda e: on_collision(e))
        npc1.apply_control(npc1_behaviors[0])
        npc2.apply_control(npc2_behaviors[0])
        frame_count = 0

        while True:
            if collision_occurred:
                print(f"Collision occurred. Ending Trial {trial_idx + 1}.")
                log_file.write(f"Collision occurred at frame {frame_count}.\n")
                break

            if agent.done():
                print(f"Run {trial_idx + 1} complete.")
                break

            control = agent.run_step()
            vehicle.apply_control(control)
            world.tick()
            time.sleep(tick_interval)

            ego_loc = vehicle.get_location()
            ego_vel = vehicle.get_velocity()
            ego_speed = math.sqrt(ego_vel.x ** 2 + ego_vel.y ** 2 + ego_vel.z ** 2)
            brake_distance = 0.0467 * ego_speed ** 2 + 0.4116 * ego_speed - 1.9913 + 0.5
            brake_distance = max(brake_distance, 0)

            def compute_deltaD(npc):
                npc_loc = npc.get_location()
                dx = ego_loc.x - npc_loc.x
                dy = ego_loc.y - npc_loc.y
                distance = math.sqrt(dx ** 2 + dy ** 2) - 4.6
                return distance - brake_distance

            deltaD_1 = compute_deltaD(npc1)
            deltaD_2 = compute_deltaD(npc2)
            current_min = min(deltaD_1, deltaD_2)

            if current_min < deltaD_tracker["min_deltaD"]:
                deltaD_tracker["min_deltaD"] = current_min
                deltaD_tracker["min_deltaD_frame"] = frame_count
            frame_count += 1



            if frame_count % lane_change_interval == 0 or frame_count == 0:
                #print(frame_count)

                idx = frame_count // lane_change_interval
                # 防越界检查
                if idx < len(npc1_behaviors):
                    npc1.apply_control(npc1_behaviors[idx])
                    npc2.apply_control(npc2_behaviors[idx])

                """
                npc_control1.throttle = random.uniform(*npc1_cfg['throttle_range'])
                npc_control1.brake = random.uniform(*npc1_cfg['brake_range']) if random.random() < npc1_cfg[
                    'brake_chance'] else 0.0
                npc_control1.steer = random.choice([-1, 1]) * random.uniform(*npc1_cfg['steer_range'])
                npc1.apply_control(npc_control1)

                npc_control1.steer += random.choice([-1, 1]) * random.uniform(0.1, 0.3)
                npc_control1.steer = max(-1.0, min(1.0, npc_control1.steer))
                npc1.apply_control(npc_control1)

                npc_control2.throttle = random.uniform(*npc2_cfg['throttle_range'])
                npc_control2.brake = random.uniform(*npc2_cfg['brake_range']) if random.random() < npc2_cfg[
                    'brake_chance'] else 0.0
                npc_control2.steer = random.choice([-1, 1]) * random.uniform(*npc2_cfg['steer_range'])
                npc2.apply_control(npc_control2)
                """

            """
            if frame_count % log_frequency == 0:
                # EV
                ev_loc = vehicle.get_location()
                ev_vel = vehicle.get_velocity()
                ev_speed = math.sqrt(ev_vel.x ** 2 + ev_vel.y ** 2 + ev_vel.z ** 2)

                # NPC1
                npc1_loc = npc1.get_location()
                npc1_vel = npc1.get_velocity()
                npc1_speed = math.sqrt(npc1_vel.x ** 2 + npc1_vel.y ** 2 + npc1_vel.z ** 2)

                # NPC2
                npc2_loc = npc2.get_location()
                npc2_vel = npc2.get_velocity()
                npc2_speed = math.sqrt(npc2_vel.x ** 2 + npc2_vel.y ** 2 + npc2_vel.z ** 2)

                # Pedestrian 1
                ped1_loc = walker1.get_location()
                ped1_vel = walker1.get_velocity()
                ped1_speed = math.sqrt(ped1_vel.x ** 2 + ped1_vel.y ** 2 + ped1_vel.z ** 2)

                # Pedestrian 2
                ped2_loc = walker2.get_location()
                ped2_vel = walker2.get_velocity()
                ped2_speed = math.sqrt(ped2_vel.x ** 2 + ped2_vel.y ** 2 + ped2_vel.z ** 2)

                collision_flag = "Yes" if collision_occurred else "No"

                log_file.write(
                    f"{frame_count}\t"
                    f"({ev_loc.x:.1f},{ev_loc.y:.1f})\t{ev_speed:.2f}\t{control.steer:.2f}\t{control.throttle:.2f}\t{control.brake:.2f}\t"
                    f"({npc1_loc.x:.1f},{npc1_loc.y:.1f})\t{npc1_speed:.2f}\t{npc_control1.steer:.2f}\t{npc_control1.throttle:.2f}\t{npc_control1.brake:.2f}\t"
                    f"({npc2_loc.x:.1f},{npc2_loc.y:.1f})\t{npc2_speed:.2f}\t{npc_control2.steer:.2f}\t{npc_control2.throttle:.2f}\t{npc_control2.brake:.2f}\t"
                    f"({ped1_loc.x:.1f},{ped1_loc.y:.1f})\t{ped1_speed:.2f}\t"
                    f"({ped2_loc.x:.1f},{ped2_loc.y:.1f})\t{ped2_speed:.2f}\t"
                    f"{collision_flag}\n"
                )
                """
        log_file.write(
            f"Minimum deltaD during trial: {deltaD_tracker['min_deltaD']:.2f} "
            f"at frame {deltaD_tracker['min_deltaD_frame']}\n"
        )
        for actor in [vehicle, npc1, npc2, collision_sensor]:
            if actor:
                actor.destroy()
        for controller, walker in [(controller1, walker1), (controller2, walker2)]:
            if controller:
                controller.stop()
                controller.destroy()
            if walker:
                walker.destroy()



if __name__ == '__main__':
    weather_yaml_path = './parameters/weather.yaml'  # Change if needed
    spawn_yaml_path = './parameters/spawn.yaml'

    if not os.path.exists(weather_yaml_path) or not os.path.exists(spawn_yaml_path):
        print("Missing weather or spawn config file.")
        sys.exit(1)

    weather_config = tools.load_weather_yaml(weather_yaml_path)
    spawn_config = tools.load_spawn_yaml(spawn_yaml_path)

    main(spawn_config,weather_config)
