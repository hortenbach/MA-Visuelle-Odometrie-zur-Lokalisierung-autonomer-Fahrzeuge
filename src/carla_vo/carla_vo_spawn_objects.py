#!/usr/bin/env python3
#
# Modified version os carla_spawn_objects
# author: Gina Hortenbach, year: 2022
#
# Copyright (c) 2019-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
base class for spawning objects (carla actors and pseudo_actors) in ROS

Gets config file from ros parameter ~objects_definition_file and spawns corresponding objects
through ROS service /carla/spawn_object.

Looks for an initial spawn point first in the launchfile, then in the config file, and
finally ask for a random one to the spawn service.

"""

import json
from logging import raiseExceptions
import math
import os
from signal import raise_signal

from transforms3d.euler import euler2quat

import ros_compatibility as roscomp
from ros_compatibility.exceptions import *
from ros_compatibility.node import CompatibleNode

from carla_msgs.msg import CarlaActorList
from carla_msgs.srv import SpawnObject, DestroyObject
from diagnostic_msgs.msg import KeyValue
from geometry_msgs.msg import Pose


cam_setup = {
    "objects": [
        {
            "type": "sensor.camera.rgb",
            "id": "rgb_left",
            "spawn_point": {
                "x": 2.0,
                "y": 0.25,
                "z": 1.0,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
            },
            "image_size_x": 800,
            "image_size_y": 600,
            "fov": 90.0,
        },
        {
            "type": "sensor.camera.rgb",
            "id": "rgb_right",
            "spawn_point": {
                "x": 2.0,
                "y": -0.25,
                "z": 1.0,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
            },
            "image_size_x": 800,
            "image_size_y": 600,
            "fov": 90.0,
        },
        {
            "type": "sensor.camera.depth",
            "id": "depth_front",
            "spawn_point": {
                "x": 2.0,
                "y": 0.0,
                "z": 1.0,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
            },
            "image_size_x": 800,
            "image_size_y": 600,
            "fov": 90.0,
        },
        {"type": "sensor.pseudo.odom", "id": "odometry"},
    ]
}


# ==============================================================================
# -- CarlaSpawnObjects ------------------------------------------------------------
# ==============================================================================


class CarlaSpawnObjects(CompatibleNode):

    """
    Handles the spawning of the ego vehicle and its sensors

    Derive from this class and implement method sensors()
    """

    def __init__(self):
        super(CarlaSpawnObjects, self).__init__("carla_spawn_objects")

        # self.objects_definition_file = self.get_param("objects_definition_file", "")
        # curr = os.path.dirname(os.path.realpath(__file__))
        # self.objects_definition_file = curr + "/config/vo_objects.json"
        self.objects_definition_file = json.loads(json.dumps(cam_setup))

        self.players = []
        self.vehicles_sensors = []
        self.global_sensors = []

        self.spawn_object_service = self.new_client(SpawnObject, "/carla/spawn_object")
        self.destroy_object_service = self.new_client(
            DestroyObject, "/carla/destroy_object"
        )

    def spawn_object(self, spawn_object_request):
        response_id = -1
        response = self.call_service(
            self.spawn_object_service,
            spawn_object_request,
            spin_until_response_received=True,
        )
        response_id = response.id
        if response_id != -1:
            self.loginfo(
                "Object (type='{}', id='{}') spawned successfully as {}.".format(
                    spawn_object_request.type, spawn_object_request.id, response_id
                )
            )
        else:
            self.logwarn(
                "Error while spawning object (type='{}', id='{}').".format(
                    spawn_object_request.type, spawn_object_request.id
                )
            )
            raise RuntimeError(response.error_string)
        return response_id

    def spawn_objects(self):
        # Read sensors from file
        if not self.objects_definition_file or not os.path.exists(
            self.objects_definition_file
        ):
            raise RuntimeError(
                "Could not read object definitions from {}".format(
                    self.objects_definition_file
                )
            )
        with open(self.objects_definition_file) as handle:
            json_actors = json.loads(handle.read())

        global_sensors = []
        ego_id = 0
        found_sensor_actor_list = False

        print("+++++ Debug")

        for actor in json_actors["objects"]:
            actor_type = actor["type"].split(".")[0]
            if actor_type == "sensor":
                global_sensors.append(actor)
            else:
                self.logwarn(
                    "Object with type {} is not a sensor, ignoring".format(
                        actor["type"]
                    )
                )
        # Debug
        print(global_sensors)

        # find ego_vehicle id
        actor_info_list = self.wait_for_message("/carla/actor_list", CarlaActorList)
        # print(actor_info_list)  # Debug

        for actor in actor_info_list.actors:
            if actor.rolename == "ego_vehicle":
                ego_id = actor.id
                # debug
                print("++++ ego_vehicle id found: ", ego_id)

        self.setup_sensors(global_sensors, ego_id)

    def setup_sensors(self, sensors, attached_vehicle_id):
        """
        Create the sensors defined by the user and attach them to the vehicle
        (or not if global sensor)
        :param sensors: list of sensors
        :param attached_vehicle_id: id of vehicle to attach the sensors to
        :return actors: list of ids of objects created
        """
        sensor_names = []
        for sensor_spec in sensors:
            if not roscomp.ok():
                break
            try:
                sensor_type = str(sensor_spec.pop("type"))
                sensor_id = str(sensor_spec.pop("id"))

                sensor_name = sensor_type + "/" + sensor_id
                if sensor_name in sensor_names:
                    raise NameError
                sensor_names.append(sensor_name)

                # if sensor attached to a vehicle, or is a 'pseudo_actor', allow default pose
                spawn_point = sensor_spec.pop("spawn_point", 0)
                if spawn_point == 0:
                    sensor_transform = self.create_spawn_point(
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0
                    )
                else:
                    sensor_transform = self.create_spawn_point(
                        spawn_point.pop("x", 0.0),
                        spawn_point.pop("y", 0.0),
                        spawn_point.pop("z", 0.0),
                        spawn_point.pop("roll", 0.0),
                        spawn_point.pop("pitch", 0.0),
                        spawn_point.pop("yaw", 0.0),
                    )

                spawn_object_request = roscomp.get_service_request(SpawnObject)
                spawn_object_request.type = sensor_type
                spawn_object_request.id = sensor_id
                spawn_object_request.attach_to = (
                    attached_vehicle_id if attached_vehicle_id is not None else 0
                )
                spawn_object_request.transform = sensor_transform
                spawn_object_request.random_pose = (
                    False  # never set a random pose for a sensor
                )

                attached_objects = []
                for attribute, value in sensor_spec.items():
                    if attribute == "attached_objects":
                        for attached_object in sensor_spec["attached_objects"]:
                            attached_objects.append(attached_object)
                        continue
                    spawn_object_request.attributes.append(
                        KeyValue(key=str(attribute), value=str(value))
                    )

                response_id = self.spawn_object(spawn_object_request)

                if response_id == -1:
                    raise RuntimeError(response_id.error_string)

                if attached_objects:
                    # spawn the attached objects
                    self.setup_sensors(attached_objects, response_id)

                self.vehicles_sensors.append(response_id)

            except KeyError as e:
                self.logerr(
                    "Sensor {} will not be spawned, the mandatory attribute {} is missing".format(
                        sensor_name, e
                    )
                )
                continue

            except RuntimeError as e:
                self.logerr("Sensor {} will not be spawned: {}".format(sensor_name, e))
                continue

            except NameError:
                self.logerr(
                    "Sensor rolename '{}' is only allowed to be used once. The second one will be ignored.".format(
                        sensor_id
                    )
                )
                continue

    def create_spawn_point(self, x, y, z, roll, pitch, yaw):
        spawn_point = Pose()
        spawn_point.position.x = x
        spawn_point.position.y = y
        spawn_point.position.z = z
        quat = euler2quat(math.radians(roll), math.radians(pitch), math.radians(yaw))

        spawn_point.orientation.w = quat[0]
        spawn_point.orientation.x = quat[1]
        spawn_point.orientation.y = quat[2]
        spawn_point.orientation.z = quat[3]
        return spawn_point

    def check_spawn_point_param(self, spawn_point_parameter):
        components = spawn_point_parameter.split(",")
        if len(components) != 6:
            self.logwarn("Invalid spawnpoint '{}'".format(spawn_point_parameter))
            return None
        spawn_point = self.create_spawn_point(
            float(components[0]),
            float(components[1]),
            float(components[2]),
            float(components[3]),
            float(components[4]),
            float(components[5]),
        )
        return spawn_point

    def destroy(self):
        """
        destroy all the players and sensors
        """
        self.loginfo("Destroying spawned objects...")
        try:
            # destroy vehicles sensors
            for actor_id in self.vehicles_sensors:
                destroy_object_request = roscomp.get_service_request(DestroyObject)
                destroy_object_request.id = actor_id
                self.call_service(
                    self.destroy_object_service,
                    destroy_object_request,
                    timeout=0.5,
                    spin_until_response_received=True,
                )
                self.loginfo("Object {} successfully destroyed.".format(actor_id))
            self.vehicles_sensors = []

            # destroy global sensors
            for actor_id in self.global_sensors:
                destroy_object_request = roscomp.get_service_request(DestroyObject)
                destroy_object_request.id = actor_id
                self.call_service(
                    self.destroy_object_service,
                    destroy_object_request,
                    timeout=0.5,
                    spin_until_response_received=True,
                )
                self.loginfo("Object {} successfully destroyed.".format(actor_id))
            self.global_sensors = []

            # destroy player
            for player_id in self.players:
                destroy_object_request = roscomp.get_service_request(DestroyObject)
                destroy_object_request.id = player_id
                self.call_service(
                    self.destroy_object_service,
                    destroy_object_request,
                    timeout=0.5,
                    spin_until_response_received=True,
                )
                self.loginfo("Object {} successfully destroyed.".format(player_id))
            self.players = []
        except ServiceException:
            self.logwarn(
                "Could not call destroy service on objects, the ros bridge is probably already shutdown"
            )


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main(args=None):
    """
    main function
    """
    roscomp.init("spawn_objects", args=args)
    spawn_objects_node = None
    try:
        spawn_objects_node = CarlaSpawnObjects()
        roscomp.on_shutdown(spawn_objects_node.destroy)
    except KeyboardInterrupt:
        roscomp.logerr("Could not initialize CarlaSpawnObjects. Shutting down.")

    if spawn_objects_node:
        try:
            spawn_objects_node.spawn_objects()
            try:
                spawn_objects_node.spin()
            except (ROSInterruptException, ServiceException, KeyboardInterrupt):
                pass
        except (ROSInterruptException, ServiceException, KeyboardInterrupt):
            spawn_objects_node.logwarn(
                "Spawning process has been interrupted. There might be actors that have not been destroyed properly"
            )
        except RuntimeError as e:
            roscomp.logfatal("Exception caught: {}".format(e))
        finally:
            roscomp.shutdown()


if __name__ == "__main__":
    main()
