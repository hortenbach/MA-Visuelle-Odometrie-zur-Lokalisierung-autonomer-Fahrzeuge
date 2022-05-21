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


def find_weather_presets():
    rgx = re.compile(".+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)")
    name = lambda x: " ".join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match("[A-Z].+", x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]


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

    world = client.get_world()
    settings = world.get_settings()

    ###################### TRAFFIC MANAGER
    if len(sys.argv) != 2:
        print("Usage: ./MA_destroy.py <actor_id>")
    else:
        try:
            actor = world.get_actors().find(id=int(sys.argv[1]))
            try:
                tm = client.get_trafficmanager(port)
                tm_port = tm.get_port()
                # actor.set_autopilot(True, tm_port)
                tm.ignore_lights_percentage(actor, 100)
                # tm.distance_to_leading_vehicle(danger_car,0)
                tm.vehicle_percentage_speed_difference(actor, -20)
            except:
                print("Cant get tm")
        except:
            print("ERROR: Couldnt find actor with id: ", int(sys.argv[1]))
