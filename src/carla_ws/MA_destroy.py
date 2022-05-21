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

    ###################### DESTROY VEHICLE
    if len(sys.argv) != 2:
        print("Usage: ./MA_destroy.py <actor_id>")
    else:
        try:
            actor = world.get_actors().find(id=int(sys.argv[1]))
            actor.destroy()
        except:
            print("ERROR: Couldnt destroy actor with id: ", int(sys.argv[1]))
