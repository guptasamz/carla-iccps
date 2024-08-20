""" Script to create a single vehicle and play with its dynamics"""
import glob
import os
import sys
import time

try:
    sys.path.append(glob.glob('../../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

from carla import VehicleLightState as vls

import argparse
import logging
from numpy import random
import pathlib

SpawnActor = carla.command.SpawnActor
SetAutopilot = carla.command.SetAutopilot
SetVehicleLightState = carla.command.SetVehicleLightState
FutureActor = carla.command.FutureActor
pathlib.Path('./vehicle_status/').mkdir(parents=True, exist_ok=True)
logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO, filename='./logs/egovehicle.log', filemode='w')

class EgoVehicle:
    # Need the set of parameters for my ego vehicle
    def __init__(self, carla_env, host, port, filterv, car_lights_on):
        
        
        self.vehicle_blueprint = self.get_vehicle_blueprint(carla_env.blueprints, filterv)
        # prepare the light state of the cars to spawn
        self.light_state = vls.NONE
        if car_lights_on:
            self.light_state = vls.Position | vls.LowBeam | vls.LowBeam

        # Getting spawn locations
        spawn_points = carla_env.world.get_map().get_spawn_points()
        self.spawn_point = spawn_points[0]

        batch = []
        self.batch_list = []
        # Spawn the ego vehicle
        batch.append(SpawnActor(self.vehicle_blueprint, self.spawn_point))
        for response in carla_env.client.apply_batch_sync(batch, carla_env.synchronous_master):
            if response.error:
                logging.error(response.error)
            else:
                self.batch_list.append(response.actor_id)

        # Getting the client and world again. This is to get the vehicle object
        carla_env.client = carla.Client(host, port)
        carla_env.world = carla_env.client.get_world()
        self.vehicles_list = []
        for actor in carla_env.world.get_actors().filter('vehicle.*'):
            print(actor)
            self.vehicles_list.append(actor)

    def get_vehicle_blueprint(self, blueprints, filterv):
        blueprint =  [x for x in blueprints if x.id.endswith(filterv)][0]
        # Randomly set the color of the vehicle
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        if blueprint.has_attribute('driver_id'):
            driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
            blueprint.set_attribute('driver_id', driver_id)
        blueprint.set_attribute('role_name', 'autopilot')

        return blueprint

    def autopilot_vehicle(self):
        for vehicle in self.vehicles_list:
            vehicle.set_autopilot(True)

        print('Set %d vehicles, press Ctrl+C to exit.' % len(self.vehicles_list))

    def remove_autopilot(self):
        for vehicle in self.vehicles_list:
            vehicle.set_autopilot(False)
        
    def destroy_vehicles(self, carla_env):
        carla_env.client.apply_batch([carla.command.DestroyActor(x) for x in self.batch_list])


    def print_vehicle_state(self):

        for vehicle in self.vehicles_list:
            with open(f'./vehicle_status/vehicle_state_{vehicle.id}.txt', 'a') as f:
                f.write(f"""
                        Vehicle ID: {vehicle.id}\n
                        Vehicle Physics: {vehicle.get_physics_control()}\n
                        \n\n\n
                        """)

    def set_vehicle_dynamics(self):
        for vehicle in self.vehicles_list:
            # Create Wheels Physics Control
            front_left_wheel  = carla.WheelPhysicsControl(tire_friction=0.1, damping_rate=1.5, max_steer_angle=70.0, radius=30.0,max_brake_torque=1500.0,max_handbrake_torque=3000.0)
            front_right_wheel = carla.WheelPhysicsControl(tire_friction=0.1, damping_rate=1.5, max_steer_angle=70.0, radius=30.0,max_brake_torque=1500.0,max_handbrake_torque=3000.0)
            rear_left_wheel   = carla.WheelPhysicsControl(tire_friction=3.5, damping_rate=1.5, max_steer_angle=0.0,  radius=30.0,max_brake_torque=1500.0,max_handbrake_torque=3000.0)
            rear_right_wheel  = carla.WheelPhysicsControl(tire_friction=3.5, damping_rate=1.5, max_steer_angle=0.0,  radius=30.0,max_brake_torque=1500.0,max_handbrake_torque=3000.0)

            wheels = [front_left_wheel, front_right_wheel, rear_left_wheel, rear_right_wheel]

            physics_control = vehicle.get_physics_control()
            # print("Vehicle physics before changing: ", physics_control)
            # print("\n\n\n")
            physics_control.wheels = wheels

            # Apply Vehicle Physics Control for the vehicle
            vehicle.apply_physics_control(physics_control)
            print(physics_control)
            print("Vehicle dynamics set for vehicle: ", vehicle.id)


    def set_vehicle_dynamic_for_weather(self, actor_list):
        pass
