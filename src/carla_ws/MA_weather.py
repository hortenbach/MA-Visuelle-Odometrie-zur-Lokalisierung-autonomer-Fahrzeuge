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
    
#    world = client.load_world('Town03')

    ###################### WEATHER
    weather_presets = find_weather_presets()
    cnt = 0
    print("Weather Options: ")
    for p in weather_presets:
        print(f"Option[{cnt}]: {p[1]}")
        cnt += 1

    if len(sys.argv) == 2:
        weather = int(sys.argv[1])
    else:
        weather = 6
    print("--------------------")
    print("Set weather to ", weather_presets[weather][1])
    world.set_weather(weather_presets[weather][0])
