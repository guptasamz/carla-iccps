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
import logging

import CarlaEnvironment as CarlaEnv
import EgoVehicle as EV
import traceback

# Function to check if a vehicle has crossed a certain location
def check_vehicle_crossed_location(vehicles, world, target_index=78, threshold=0.5):
    # Get the target location from spawn points
    spawn_points = world.get_map().get_spawn_points()
    target_location = spawn_points[target_index].location

    for vehicle in vehicles:
        # Get the vehicle location
        vehicle_location = vehicle.get_location()

        # Calculate the distance between the vehicle and the target location
        distance = vehicle_location.distance(target_location)

        if distance < threshold:
            return True
            # You can add additional logic here if needed
        else:
            return False


def main():
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
    argparser.add_argument(
        '--safe',
        action='store_true',
        help='avoid spawning vehicles prone to accidents')
    argparser.add_argument(
        '--filterv',
        metavar='PATTERN',
        default='model3',
        # help='vehicles filter (default: "vehicle.*")')
        help='vehicles filter (default: "vehicle.model3")')

    argparser.add_argument(
        '--tm-port',
        metavar='P',
        default=8000,
        type=int,
        help='port to communicate with TM (default: 8000)')
    argparser.add_argument(
        '--sync',
        action='store_true',
        help='Synchronous mode execution')
    argparser.add_argument(
        '--hybrid',
        action='store_true',
        help='Enanble')
    argparser.add_argument(
        '-s', '--seed',
        metavar='S',
        type=int,
        help='Random device seed')
    argparser.add_argument(
        '--car-lights-on',
        action='store_true',
        default=False,
        help='Enanble car lights')
    argparser.add_argument(
        '--map',
        metavar='STRING',
        default='Town02',
        help='Map Filter (default: "Town02")')

    args = argparser.parse_args()

    try: 

        # Create a CarlaEnvironment object
        carla_env = CarlaEnv.CarlaEnvironment(args.host, args.port, args.tm_port, args.map, args.hybrid, args.seed, args.sync, args.filterv)
        # Set the spectator to the ego vehicle
        carla_env.set_location_to_actor_spawn()
        # Set all traffic lights to green
        carla_env.set_all_traffic_lights_green()
        print('All traffic lights set to green')


        # Creating the ego vehicle
        ego_vehicle = EV.EgoVehicle(carla_env, args.host, args.port, args.filterv, args.car_lights_on)
        # Set the autopilot for the ego vehicle
        ego_vehicle.autopilot_vehicle()


        crossed_point = False
        is_parameter_set = False
        start_time = time.time()
        # Letting the simulation run
        while True:
            if args.sync and carla_env.synchronous_master:
                carla_env.world.tick()
            else:

                if not crossed_point:
                    crossed_point = check_vehicle_crossed_location(ego_vehicle.vehicles_list, carla_env.world)
                
                if not is_parameter_set and crossed_point:
                    carla_env.set_weather()
                    ego_vehicle.set_vehicle_dynamics()
                    is_parameter_set = True

                # TODO Step1: Write logic for the carla to detect the change in the weather conditions.
                # For now I think we can just get this from CARLA directly not need of any model or anything. 
                
                # TODO Step2: Once I have the weather conditions, reduce the speed of the vehicle if the amount of rain is 
                # more than a certain threshold.


                # printing the state of the vehicle every 10 seconds
                if time.time() - start_time > 10:
                    ego_vehicle.print_vehicle_state()
                    start_time = time.time()

                carla_env.world.wait_for_tick()


    except Exception as e:
        print(f"Error: {e}")
        ex_type, ex, tb = sys.exc_info()
        print(f"Traceback: {traceback.print_tb(tb)}")
        print('Error with setting for all actors')

    finally:
        if args.sync and carla_env.synchronous_master:
            settings = carla_env.world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            carla_env.world.apply_settings(settings)
            
        print('\nDestroying %d vehicles')
        ego_vehicle.destroy_vehicles(carla_env)

        print('\nResetting the weather to clear noon')
        carla_env.set_clear_noon_weather()

        time.sleep(0.5)


if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')




    