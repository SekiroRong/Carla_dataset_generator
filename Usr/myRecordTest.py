# -*- coding = utf-8 -*-
# @Time : 2021/11/21 9:59
# @Author : 戎昱
# @File : myRecordTest.py
# @Software : PyCharm
# @Contact : sekirorong@gmail.com
# @github : https://github.com/SekiroRong
import glob
import os
import sys
import time

save_path = r"G:\Carla_Recorder\Cam_Recorder\test"

cam_size_x = 1200
cam_size_y = 800
cam_fov = 90

set_autopilot = False    # 自动巡航
set_cam = True
set_collision_sensor = True
set_Lane_invasion_sensor = True
set_Obstacle_sensor = True


try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

import argparse
import logging
import random

try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_q
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError(
        'cannot import numpy, make sure numpy package is installed')

try:
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '/carla')
except IndexError:
    pass

from agents.navigation.behavior_agent import BehaviorAgent  # pylint: disable=import-error
from agents.navigation.basic_agent import BasicAgent  # pylint: disable=import-error

class World(object):
    """ Class representing the surrounding environment """

    def __init__(self, carla_world, hud, args):
        """Constructor method"""
        self._args = args
        self.world = carla_world
        try:
            self.map = self.world.get_map()
        except RuntimeError as error:
            print('RuntimeError: {}'.format(error))
            print('  The server could not send the OpenDRIVE (.xodr) file:')
            print('  Make sure it exists, has the same name of your town, and is correct.')
            sys.exit(1)
        self.hud = hud
        self.player = None

        # Get a random blueprint.
        blueprint = random.choice(self.world.get_blueprint_library().filter('vehicle.*'))
        blueprint.set_attribute('role_name', 'hero')
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        if self.player is not None:
            spawn_point = self.player.get_transform()
            spawn_point.location.z += 2.0
            spawn_point.rotation.roll = 0.0
            spawn_point.rotation.pitch = 0.0
            self.destroy()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            self.modify_vehicle_physics(self.player)
        while self.player is None:
            if not self.map.get_spawn_points():
                print('There are no spawn points available in your map/town.')
                print('Please add some Vehicle Spawn Point to your UE4 scene.')
                sys.exit(1)
            spawn_points = self.map.get_spawn_points()
            spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            self.modify_vehicle_physics(self.player)
        # self.collision_sensor = None
        # self.lane_invasion_sensor = None
        # self.gnss_sensor = None
        # self.camera_manager = None
        # self._weather_presets = find_weather_presets()
        # self._weather_index = 0
        # self._actor_filter = args.filter
        # self.restart(args)
        # self.world.on_tick(hud.on_world_tick)
        # self.recording_enabled = False
        # self.recording_start = 0

    def restart(self, args):
        """Restart the world"""
        # Keep same camera config if the camera manager exists.
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_id = self.camera_manager.transform_index if self.camera_manager is not None else 0

        # Get a random blueprint.
        blueprint = random.choice(self.world.get_blueprint_library().filter(self._actor_filter))
        blueprint.set_attribute('role_name', 'hero')
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)

        # Spawn the player.
        if self.player is not None:
            spawn_point = self.player.get_transform()
            spawn_point.location.z += 2.0
            spawn_point.rotation.roll = 0.0
            spawn_point.rotation.pitch = 0.0
            self.destroy()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            self.modify_vehicle_physics(self.player)
        while self.player is None:
            if not self.map.get_spawn_points():
                print('There are no spawn points available in your map/town.')
                print('Please add some Vehicle Spawn Point to your UE4 scene.')
                sys.exit(1)
            spawn_points = self.map.get_spawn_points()
            spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            self.modify_vehicle_physics(self.player)

        if self._args.sync:
            self.world.tick()
        else:
            self.world.wait_for_tick()

        # Set up the sensors.
        # self.collision_sensor = CollisionSensor(self.player, self.hud)
        # self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
        # self.gnss_sensor = GnssSensor(self.player)
        # self.camera_manager = CameraManager(self.player, self.hud)
        # self.camera_manager.transform_index = cam_pos_id
        # self.camera_manager.set_sensor(cam_index, notify=False)
        # actor_type = get_actor_display_name(self.player)
        # self.hud.notification(actor_type)

    def next_weather(self, reverse=False):
        """Get next weather setting"""
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.hud.notification('Weather: %s' % preset[1])
        self.player.get_world().set_weather(preset[0])

    def modify_vehicle_physics(self, actor):
        #If actor is not a vehicle, we cannot use the physics control
        try:
            physics_control = actor.get_physics_control()
            physics_control.use_sweep_wheel_collision = True
            actor.apply_physics_control(physics_control)
        except Exception:
            pass

    def tick(self, clock):
        """Method for every tick"""
        self.hud.tick(self, clock)

    def render(self, display):
        """Render world"""
        self.camera_manager.render(display)
        self.hud.render(display)

    def destroy_sensors(self):
        """Destroy sensors"""
        self.camera_manager.sensor.destroy()
        self.camera_manager.sensor = None
        self.camera_manager.index = None

    def destroy(self):
        """Destroys all actors"""
        actors = [
            self.camera_manager.sensor,
            self.collision_sensor.sensor,
            self.lane_invasion_sensor.sensor,
            self.gnss_sensor.sensor,
            self.player]
        for actor in actors:
            if actor is not None:
                actor.destroy()


def main():
    pygame.init()
    pygame.font.init()
    print('\nPygame is set')

    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    args = argparser.parse_args()

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)

    try:
        traffic_manager = client.get_trafficmanager()  #待仔细研究这个是啥
        world = World(client.get_world(), None, args)
        settings = world.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05   #设定仿真步长为固定0.05s
        world.world.apply_settings(settings)

        ego_vehicle = None
        ego_cam = None
        ego_col = None
        ego_lane = None
        ego_obs = None
        ego_gnss = None
        ego_imu = None

        # --------------
        # Start recording
        # --------------

        client.start_recorder('~/tutorial/recorder/recording01.log', True)


        # --------------
        # Spawn ego vehicle
        # --------------

        ego_bp = world.world.get_blueprint_library().find('vehicle.tesla.model3')
        ego_bp.set_attribute('role_name','ego')
        print('\nEgo role_name is set')
        ego_color = random.choice(ego_bp.get_attribute('color').recommended_values)
        ego_bp.set_attribute('color',ego_color)
        print('\nEgo color is set')

        spawn_points = world.world.get_map().get_spawn_points()
        number_of_spawn_points = len(spawn_points)

        if 0 < number_of_spawn_points:
            random.shuffle(spawn_points)
            ego_transform = spawn_points[0]
            ego_vehicle = world.world.spawn_actor(ego_bp,ego_transform)
            print('\nEgo is spawned')
        else: 
            logging.warning('Could not found any spawn points')

        # if args.agent == "Basic":    # 这个agent也还没研究
        #     agent = BasicAgent(world.player)
        # else:
        #     agent = BehaviorAgent(world.player, behavior=args.behavior)
        agent = BehaviorAgent(world.player, behavior='normal')   #这个behavior也还没研究

        destination = random.choice(spawn_points).location  #设定仿真目的地
        agent.set_destination(destination)
        print('\nDestination is set')


        # --------------
        # Add a RGB camera sensor to ego vehicle.
        # --------------
        if set_cam:
            cam_bp = None
            cam_bp = world.world.get_blueprint_library().find('sensor.camera.rgb')
            cam_bp.set_attribute("image_size_x",str(cam_size_x))
            cam_bp.set_attribute("image_size_y",str(cam_size_y))
            cam_bp.set_attribute("fov",str(cam_fov))
            cam_location = carla.Location(2,0,1)   #cam的位置（待研究）
            cam_rotation = carla.Rotation(0,0,0)
            cam_transform = carla.Transform(cam_location,cam_rotation)
            ego_cam = world.world.spawn_actor(cam_bp,cam_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
            ego_cam.listen(lambda image: image.save_to_disk(save_path + '/%.6d.jpg' % image.frame))


        # --------------
        # Add collision sensor to ego vehicle.
        # --------------
        # """
        if set_collision_sensor:
            col_bp = world.world.get_blueprint_library().find('sensor.other.collision')
            col_location = carla.Location(0,0,0)
            col_rotation = carla.Rotation(0,0,0)
            col_transform = carla.Transform(col_location,col_rotation)
            ego_col = world.world.spawn_actor(col_bp,col_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
            def col_callback(colli):
                print("Collision detected:\n"+str(colli)+'\n')
            ego_col.listen(lambda colli: col_callback(colli))
        # """

        # --------------
        # Add Lane invasion sensor to ego vehicle.
        # --------------
        # """
        if set_Lane_invasion_sensor:
            lane_bp = world.world.get_blueprint_library().find('sensor.other.lane_invasion')
            lane_location = carla.Location(0,0,0)
            lane_rotation = carla.Rotation(0,0,0)
            lane_transform = carla.Transform(lane_location,lane_rotation)
            ego_lane = world.world.spawn_actor(lane_bp,lane_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
            def lane_callback(lane):
                print("Lane invasion detected:\n"+str(lane)+'\n')
            ego_lane.listen(lambda lane: lane_callback(lane))
        # """

        # --------------
        # Add Obstacle sensor to ego vehicle.
        # --------------
        if set_Obstacle_sensor:
            obs_bp = world.world.get_blueprint_library().find('sensor.other.obstacle')
            obs_bp.set_attribute("only_dynamics",str(True))
            obs_location = carla.Location(0,0,0)
            obs_rotation = carla.Rotation(0,0,0)
            obs_transform = carla.Transform(obs_location,obs_rotation)
            ego_obs = world.world.spawn_actor(obs_bp,obs_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
            def obs_callback(obs):
                print("Obstacle detected:\n"+str(obs)+'\n')
            ego_obs.listen(lambda obs: obs_callback(obs))


        # --------------
        # Add GNSS sensor to ego vehicle.
        # --------------
        """
        gnss_bp = world.get_blueprint_library().find('sensor.other.gnss')
        gnss_location = carla.Location(0,0,0)
        gnss_rotation = carla.Rotation(0,0,0)
        gnss_transform = carla.Transform(gnss_location,gnss_rotation)
        gnss_bp.set_attribute("sensor_tick",str(3.0))
        ego_gnss = world.spawn_actor(gnss_bp,gnss_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
        def gnss_callback(gnss):
            print("GNSS measure:\n"+str(gnss)+'\n')
        ego_gnss.listen(lambda gnss: gnss_callback(gnss))
        """

        # --------------
        # Add IMU sensor to ego vehicle.
        # --------------
        """
        imu_bp = world.get_blueprint_library().find('sensor.other.imu')
        imu_location = carla.Location(0,0,0)
        imu_rotation = carla.Rotation(0,0,0)
        imu_transform = carla.Transform(imu_location,imu_rotation)
        imu_bp.set_attribute("sensor_tick",str(3.0))
        ego_imu = world.spawn_actor(imu_bp,imu_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
        def imu_callback(imu):
            print("IMU measure:\n"+str(imu)+'\n')
        ego_imu.listen(lambda imu: imu_callback(imu))
        """

        # --------------
        # Place spectator on ego spawning
        # --------------
        """
        spectator = world.get_spectator()
        world_snapshot = world.wait_for_tick() 
        spectator.set_transform(ego_vehicle.get_transform())
        """
        clock = pygame.time.Clock()

        # --------------
        # Enable autopilot for ego vehicle
        # --------------
        # if set_autopilot:
        #     ego_vehicle.set_autopilot(True)


        # --------------
        # Game loop. Prevents the script from finishing.
        # --------------
        print('\nStart simulation!')
        while True:
            clock.tick()
            world.world.tick()    # 这个是同步模式
            # world_snapshot = world.wait_for_tick() # 这个是非同步模式的
            # world.tick(clock)   # 不太懂为啥要再tick一遍 应该不用 貌似是和HUD同步
            if agent.done():
                print("The target has been reached, stopping the simulation")
                break

            control = agent.run_step()
            print('\nStep')
            control.manual_gear_shift = False
            world.player.apply_control(control)

    finally:
        # --------------
        # Stop recording and destroy actors
        # --------------
        client.stop_recorder()
        if ego_vehicle is not None:
            if ego_cam is not None:
                ego_cam.stop()
                ego_cam.destroy()
            if ego_col is not None:
                ego_col.stop()
                ego_col.destroy()
            if ego_lane is not None:
                ego_lane.stop()
                ego_lane.destroy()
            if ego_obs is not None:
                ego_obs.stop()
                ego_obs.destroy()
            if ego_gnss is not None:
                ego_gnss.stop()
                ego_gnss.destroy()
            if ego_imu is not None:
                ego_imu.stop()
                ego_imu.destroy()
            ego_vehicle.destroy()

        if world is not None:
            settings = world.world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            world.world.apply_settings(settings)
            traffic_manager.set_synchronous_mode(True)

        pygame.quit()

if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\nDone with tutorial_ego.')