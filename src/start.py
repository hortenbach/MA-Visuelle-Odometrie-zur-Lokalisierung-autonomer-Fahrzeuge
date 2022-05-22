#!/bin/python3
from multiprocessing import Process
import time
import os

#CARLAPATH = "/home/hortenbach/carla/CARLA_0.9.12/CarlaUE4.sh"
CARLAPATH = "/path/to/carla//CarlaUE4.sh"

def carla_server():
    os.system(CARLAPATH)

def rosbridge():
    time.sleep(15)
    os.system("ros2 launch carla_ros_bridge carla_ros_bridge.launch.py timeout:=10 town:=Town03")

def carlaspawn():
    time.sleep(20)
    os.system("ros2 launch carla_spawn_objects carla_spawn_objects.launch.py objects_definition_file:=/path/to/src/carla_vo/config/objects.json town:=Town03 timeout:=10")

def manual():
    time.sleep(25)
    os.system("ros2 launch carla_manual_control carla_manual_control.launch.py")

def sync2():
    time.sleep(20)
    os.system("ros2 run carla_vo stereo_synchronizer")

def runInParallel(*fns):
    global proc
    for fn in fns:
        p = Process(target=fn)
        p.start()
        proc.append(p)
    for p in proc:
        p.join()

proc = []

if __name__ == '__main__':
    try:
        runInParallel(carla_server, rosbridge, carlaspawn, manual, sync2)
    except KeyboardInterrupt:
        for p in proc:
            p.terminate()
            time.sleep(3)
