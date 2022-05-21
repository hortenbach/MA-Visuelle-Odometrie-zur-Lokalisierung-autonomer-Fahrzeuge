#!/usr/bin/env python3
import sys

import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

from cv_bridge import CvBridge
import cv2

# source ~/carla-ros-bridge/install/setup.bash


class BagFileParser:
    def __init__(self, bag_file):
        self.conn = sqlite3.connect(bag_file)
        self.cursor = self.conn.cursor()

        ## create a message type map
        topics_data = self.cursor.execute(
            "SELECT id, name, type FROM topics"
        ).fetchall()
        self.topic_type = {name_of: type_of for id_of, name_of, type_of in topics_data}
        self.topic_id = {name_of: id_of for id_of, name_of, type_of in topics_data}
        try:
            self.topic_msg_message = {
                name_of: get_message(type_of) for id_of, name_of, type_of in topics_data
            }
        except:
            print("Error reading bag file. Have you sourced carla-ros-brdige?")

    def __del__(self):
        self.conn.close()

    def get_topics(self):
        return self.topic_id

    # Return [(timestamp0, message0), (timestamp1, message1), ...]
    def get_messages(self, topic_name):

        topic_id = self.topic_id[topic_name]
        # Get from the db
        rows = self.cursor.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id = {}".format(topic_id)
        ).fetchall()
        # Deserialise all and timestamp them
        return [
            (timestamp, deserialize_message(data, self.topic_msg_message[topic_name]))
            for timestamp, data in rows
        ]


# timestamp at parser.get_messages("/carla/ego_vehicle/odometry")[*][0]
# nav_msgs/Odometry.msg at parser.get_messages("/carla/ego_vehicle/odometry")[*][1]

if __name__ == "__main__":

    # try:
    #     bag_file =  sys.argv[1]
    # except IndexError:
    #     raise SystemExit("Usage: ./BagFileParser.py <path_to_bagfile>")
    # bag_file = "/home/hortenbach/CrucialM2/rosbags/01_all/rosbag2_2022_03_24-21_25_08/rosbag2_2022_03_24-21_25_08_0.db3"
    bag_file = "/home/hortenbach/CrucialM2/rosbags/cam_recordings/wetsunset_00/wetsunset_00_0.db3"
    parser = BagFileParser(bag_file)
    topics = parser.get_topics()

    # type sensor_msgs.msg._image.Image
    camImg_l = parser.get_messages("/carla/ego_vehicle/rgb_left/image")
    camImg_l = [camImg_l[i][1] for i in range(len(camImg_l))]
    camImg_r = parser.get_messages("/carla/ego_vehicle/rgb_right/image")
    camImg_r = [camImg_r[i][1] for i in range(len(camImg_r))]
    rgb_d = parser.get_messages("/carla/ego_vehicle/depth_front/image")
    rgb_d = [rgb_d[i][1] for i in range(len(rgb_d))]

    bridge = CvBridge()
    for i in range(len(camImg_l)):
        cv_img_l = bridge.imgmsg_to_cv2(camImg_l[i], desired_encoding="passthrough")
        cv_img_r = bridge.imgmsg_to_cv2(camImg_r[i], desired_encoding="passthrough")
        cv_depth = bridge.imgmsg_to_cv2(rgb_d[i], desired_encoding="passthrough")
        cv2.imwrite(f"./pictures/wetsunset/left/image_left_{i}.jpg", cv_img_l)
        cv2.imwrite(f"./pictures/wetsunset/right/image_right_{i}.jpg", cv_img_r)
        cv2.imwrite(f"./pictures/wetsunset/depth/image_depth_{i}.jpg", cv_depth)
