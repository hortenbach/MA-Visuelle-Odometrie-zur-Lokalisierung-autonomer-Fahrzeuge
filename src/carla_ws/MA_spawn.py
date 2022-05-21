#!/bin/python3
import glob
import os
import sys
import logging

try:
    sys.path.append(
        glob.glob(
            "../carla/dist/carla-*%d.%d-%s.egg"
            % (
                sys.version_info.major,
                sys.version_info.minor,
                "win-amd64" if os.name == "nt" else "linux-x86_64",
            )
        )[0]
    )
except IndexError:
    pass

import carla
import re
import time


if __name__ == "__main__":

    # Setup CARLA Server and connect
    host = "127.0.0.1"
    port = 2000
    actorname = "ego_vehicle"

    client = carla.Client(host, port)
    client.set_timeout(20.0)

    ###################### LOGGING
    # Setup logging
    log_level = logging.INFO
    logging.basicConfig(format="%(levelname)s: %(message)s", level=log_level)

    # write log file at script directory root path
    DIR = os.path.dirname(os.path.realpath(__file__))
    # print(__doc__)
    logging.basicConfig(filename=f"{DIR}/info.log", level=logging.INFO, force=True)

    logging.info("listening to server %s:%s", host, port)

    ###################### WORLD SETTINGS
    world = client.get_world()
    settings = world.get_settings()
    # change settings
    # settings.XY()
    # world.apply_settings(settings)

    ###################### SPAWN VEHICLE
    map = world.get_map()
    spawn_points = map.get_spawn_points()
    print(spawn_points[3])

    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.find("vehicle.ford.mustang")
    vehicle_bp.set_attribute("role_name", "ego_vehicle")

    # transform = Transform(Location(x=230, y=195, z=40), Rotation(yaw=180))
    # actor = world.spawn_actor(vehicle_bp, transform)
    actor_queue = []
    try:
        actor = world.spawn_actor(vehicle_bp, spawn_points[3])
        actor_queue.append(actor)
        while True:
            pass
    except KeyboardInterrupt:
        print("\nDestroy actor.")
        actor_queue.pop().destroy()
