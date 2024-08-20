
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
import logging
logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO, filename='./logs/environment.log', filemode='w')


class CarlaEnvironment: 
    def __init__(self, host, port, tm_port, map, hydrid, seed, sync, filterv):
        self.client = carla.Client(host, port)
        
        self.client.set_timeout(10.0)
        self.world = self.client.load_world(map)

        # Setting up the traffic manager
        self.traffic_manager = self.client.get_trafficmanager(tm_port)
        if hydrid:
            self.traffic_manager.set_hybrid_physics_mode(True)
        if seed is not None:
            self.traffic_manager.set_random_device_seed(seed)
        self.settings = self.world.get_settings()

        ### SOME TRAFFIC MANAGER ATTRIBUTES - Do not understand them yet ############################
        self.synchronous_master = False
        if sync:
            self.traffic_manager.set_synchronous_mode(True)
            if not self.settings.synchronous_mode:
                self.synchronous_master = True
                self.settings.synchronous_mode = True
                self.settings.fixed_delta_seconds = 0.05
                self.world.apply_settings(self.settings)
            else:
                self.synchronous_master = False
        # Getting the blueprints
        self.blueprints = self.world.get_blueprint_library()
        self.blueprints = sorted(self.blueprints, key=lambda bp: bp.id)
    

        logging.info('Successfully listening to server %s:%s and TM port %s' % (host, port, tm_port))

    def set_all_traffic_lights_green(self):
        # Get all the traffic lights in the world
        traffic_lights = self.world.get_actors().filter('traffic.traffic_light')

        for traffic_light in traffic_lights:
            # Set the traffic light state to green
            traffic_light.set_state(carla.TrafficLightState.Green)

            # Optionally, you can also set the traffic light to always stay green
            traffic_light.set_green_time(999999)
            traffic_light.set_red_time(0)
            traffic_light.set_yellow_time(0)

            print(f"Traffic light {traffic_light.id} set to green.")


    def set_weather(self):
        # Define a rainy weather scenario
        rainy_weather = carla.WeatherParameters(
            cloudiness=100.0,
            precipitation=100.0,
            precipitation_deposits=100.0,
            wind_intensity=100.0,
            sun_altitude_angle=45.0,
            wetness=100.0,  # High wetness indicates wet roads
            fog_density=0.0
        )

        # Apply the weather to the world
        self.world.set_weather(rainy_weather)

    def set_clear_noon_weather(self):
        # Define a clear noon weather scenario
        clear_noon_weather = carla.WeatherParameters(
            cloudiness=0.0,
            precipitation=0.0,
            precipitation_deposits=0.0,
            wind_intensity=0.0,
            sun_altitude_angle=60.0,  # High sun altitude for noon
            wetness=0.0,  # No wetness indicates dry roads
            fog_density=0.0
        )

        # Apply the clear weather to the world
        self.world.set_weather(clear_noon_weather)
        

    def set_location_to_actor_spawn(self):
        # Get the spectator (camera) object
        spectator = self.world.get_spectator()
        # Getting spawn locations
        spawn_points = self.world.get_map().get_spawn_points()
        # Get the first spawn point
        transform = spawn_points[0]
        # Set the spectator's transform to the new location
        spectator.set_transform(transform)
