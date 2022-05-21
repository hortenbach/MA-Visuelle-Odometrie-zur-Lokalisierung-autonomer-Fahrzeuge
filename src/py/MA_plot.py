#!/usr/bin/env python3
import sys
import sqlite3
from typing import Tuple
import os
from tomlkit import string
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import cv2
from cv_bridge import CvBridge as cvb
import numpy as np
from multipledispatch import dispatch

# source ~/carla-ros-bridge/install/setup.bash 

class BagFileParser():
    def __init__(self, bag_file):
        self.conn = sqlite3.connect(bag_file)
        self.cursor = self.conn.cursor()

        ## create a message type map
        topics_data = self.cursor.execute("SELECT id, name, type FROM topics").fetchall()
        self.topic_type = {name_of:type_of for id_of,name_of,type_of in topics_data}
        self.topic_id = {name_of:id_of for id_of,name_of,type_of in topics_data}
        try:
            self.topic_msg_message = {name_of:get_message(type_of) for id_of,name_of,type_of in topics_data}
        except:
            print("Error readinf bag file. Have you sourced carla-ros-brdige?")

    def __del__(self):
        self.conn.close()

    # Return [(timestamp0, message0), (timestamp1, message1), ...]
    def get_messages(self, topic_name):

        topic_id = self.topic_id[topic_name]
        # Get from the db
        rows = self.cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = {}".format(topic_id)).fetchall()
        # Deserialise all and timestamp them
        return [ (timestamp,deserialize_message(data, self.topic_msg_message[topic_name])) for timestamp,data in rows]

@dispatch(list)
def dist(data):
    sum_ = 0
    for i in range(len(data)-1):
        sum_ += np.sqrt(pow((data[i+1][1].pose.pose.position.x - data[i][1].pose.pose.position.x),2) + pow((data[i+1][1].pose.pose.position.y - data[i][1].pose.pose.position.y), 2))
    return sum_

@dispatch(float, float)
def dist(x,y):
    return np.sqrt(pow(x,2) + pow(y,2))

@dispatch(float, float, float)
def dist(x,y,z):
    return np.sqrt(pow(x,2) + pow(y,2) + pow(z,2))


def print_geom_msg_info(msg, num, name=None):
    f = int(len(msg)/num)
    i = 0
    for j in range(num):
        print(f"{'{0: <13}'.format(f'[{name}({i})')} {'{0:>18}'.format('Header]')}\t {msg[i][1].header}")
        print(f"{'{0: <13}'.format(f'[{name}({i})')} {'{0:>18}'.format('Pose Position]')}\t {msg[i][1].pose.pose.position}")
        print(f"{'{0: <13}'.format(f'[{name}({i})')} {'{0:>18}'.format('Pose Orientation]')}\t {msg[i][1].pose.pose.orientation}")
        print(f"{'{0: <13}'.format(f'[{name}({i})')} {'{0:>18}'.format('Twist Linear]')}\t {msg[i][1].twist.twist.linear}")
        print(f"{'{0: <13}'.format(f'[{name}({i})')} {'{0:>18}'.format('Twist Angle]')}\t {msg[i][1].twist.twist.angular}")
        i += f

#=====================================
# Plot trajectories with subplots
#=====================================
def plot2d_trajectories(gt, vo, rgbd, title='test', path='_'):
    fig = plt.figure(figsize=plt.figaspect(0.5))
    fig.suptitle(title, fontsize=12)
    #===================
    # get difference between initial poses of ground thruth and vo
    #===================
    gt_init_pos = gt[0][1].pose.pose.position
    stereo_init_pos = vo[0][1].pose.pose.position
    rgbd_init_pos = rgbd[0][1].pose.pose.position
    gtdx = 0.0 - (gt_init_pos.x)
    gtdy = 0.0 - (gt_init_pos.y)
    stereodx = 0.0 - (stereo_init_pos.x)
    stereody = 0.0 - (stereo_init_pos.y)
    rgbddx = 0.0 - (rgbd_init_pos.x)
    rgbddy = 0.0 - (rgbd_init_pos.y)
    print(f"Initial pose difference [gt]:\ndx: {gtdx}\tdy: {gtdy}\t")
    print(f"Initial pose difference [stereo]:\ndx: {stereodx}\tdy: {stereody}\t")
    print(f"Initial pose difference [rgbd]:\ndx: {rgbddx}\tdy: {rgbddy}\t")

    #===================
    # plot gound truth
    #===================
    # get timestamps
    #t_des = [trajectory[i][0] for i in range(len(trajectory))]
    # prepare coordinates
    p_des_x = []
    p_des_y = []
    p_des_z = []
    for i in range(len(gt)):
        p_des_x.append(gt[i][1].pose.pose.position.x + gtdx)
        p_des_y.append(gt[i][1].pose.pose.position.y + gtdy)
    print(f"frames gt {len(gt)}")
    
    ax = fig.add_subplot(1,1,1)
    ax.set_xlabel('X', fontsize=10)
    ax.set_ylabel('Y', fontsize=10)
    ax.scatter(p_des_x, p_des_y, s=1)
    p_stereo_x = []
    p_stereo_y = []
    for i in range(len(vo)):
        # get initial pose and use for normalizing trajectories
        p_stereo_x.append(vo[i][1].pose.pose.position.x + stereodx)
        p_stereo_y.append(vo[i][1].pose.pose.position.y + stereody)
    print(f"frames stereo {len(vo)}")
    
    ax.set_xlabel('X', fontsize=10)
    ax.set_ylabel('Y', fontsize=10)
    # ax.yaxis._axinfo['label']['space_factor'] = 3.0
    ax.scatter(p_stereo_x, p_stereo_y, s=1)
    # plot rgbd vo estimation
    p_rgbd_x = []
    p_rgbd_y = []
    for i in range(len(rgbd)):
        # get initial pose and use for normalizing trajectories
        p_rgbd_x.append(rgbd[i][1].pose.pose.position.x + rgbddx)
        p_rgbd_y.append(rgbd[i][1].pose.pose.position.y + rgbddy)
    print(f"frames rgbd {len(rgbd)}")
    
    ax.set_xlabel('X', fontsize=10)
    ax.set_ylabel('Y', fontsize=10)
    # ax.yaxis._axinfo['label']['space_factor'] = 3.0
    # ax.set_zlim(-10,50) 
    ax.scatter(p_rgbd_x, p_rgbd_y, s=1)
    ax.legend(['ground truth','stereo rgb','rgb-d']) 
    #===== show plots 
    plt.grid()
    # plt.axis('equal')
    p_xmax = max([max(p_des_x), max(p_stereo_x), max(p_rgbd_x)])
    p_ymax = max([max(p_des_y), max(p_stereo_y), max(p_rgbd_y)])
    p_xmin = min([min(p_des_x), min(p_stereo_x), min(p_rgbd_x)])
    p_ymin = min([min(p_des_y), min(p_stereo_y), min(p_rgbd_y)])
    offset = 20
    plt.ylim(ymax = p_ymax+offset, ymin = p_ymin+offset)
    plt.xlim(xmax = p_xmax+offset, xmin =p_xmin+offset)
    # path=os.path.dirname(".")
    folder = os.path.basename(path)
    plt.savefig(f"{folder}.png")
    # plt.show()

@dispatch(list, list)
def plot_trajectories(gt, vo):
    title="test"
    #===============
    # set up
    #===============
    fig = plt.figure(figsize=plt.figaspect(0.5))
    fig.suptitle(title, fontsize=12)
    figcnt = 1
    
    #===================
    # get difference between initial poses of ground thruth and vo
    #===================
    gt_init_pos = gt[0][1].pose.pose.position
    vo_init_pos = vo[0][1].pose.pose.position
    dx = (gt_init_pos.x)
    dy = (gt_init_pos.y)
    dz = (gt_init_pos.z)
    print(f"Initial pose difference [gt-vo]:\ndx: {dx}\tdy: {dy}\tdz: {dz}")

    #===================
    # plot gound truth
    #===================
    # get timestamps
    #t_des = [trajectory[i][0] for i in range(len(trajectory))]
    # prepare coordinates
    p_des_x = []
    p_des_y = []
    p_des_z = []

    for i in range(len(gt)):
        # get initial pose and use for normalizing trajectories
        p_des_x.append(gt[i][1].pose.pose.position.x-dx)
        p_des_y.append(gt[i][1].pose.pose.position.y)
        p_des_z.append(gt[i][1].pose.pose.position.z)
        
    print(f"frames gt {len(gt)}")
    
    ax = fig.add_subplot(1,1,1, projection='3d')
    ax.set_title('ground truth')
    ax.set_xlabel('X', fontsize=10)
    ax.set_ylabel('Y', fontsize=10)
    ax.set_zlabel('Z', fontsize=10)
    ax.yaxis._axinfo['label']['space_factor'] = 3.0
    ax.scatter(p_des_x, p_des_y, p_des_z, s=1)
    ax.set_zlim(-10,50) 
    
    # plot stereo vo estimation
    p_vo_x = []
    p_vo_y = []
    p_vo_z = []
    for i in range(len(vo)):
        # get initial pose and use for normalizing trajectories
        p_vo_x.append(vo[i][1].pose.pose.position.x)
        p_vo_y.append(vo[i][1].pose.pose.position.y)
        p_vo_z.append(vo[i][1].pose.pose.position.z)
        # debug
    print(f"frames stereo {len(vo)}")
    
    ax.set_xlabel('X', fontsize=10)
    ax.set_ylabel('Y', fontsize=10)
    ax.set_zlabel('Z', fontsize=10)
    ax.yaxis._axinfo['label']['space_factor'] = 3.0
    ax.scatter(p_vo_x, p_vo_y, p_vo_z, s=1)
    
    #===== show plots 
    plt.show()

@dispatch(list, list, list)
def plot_trajectories(gt, vo, rgbd):
    title='test'
    #===============
    # set up
    #===============
    fig = plt.figure(figsize=plt.figaspect(0.5))
    fig.suptitle(title, fontsize=12)
    figcnt = 1
    # ax.autoscale_view()
    # ax.set_zlim(0.0, 0.1)
    
    #===================
    # get difference between initial poses of ground thruth and vo
    #===================
    gt_init_pos = gt[0][1].pose.pose.position
    vo_init_pos = vo[0][1].pose.pose.position
    dx = (gt_init_pos.x)
    dy = (gt_init_pos.y)
    dz = (gt_init_pos.z)
    # dx = (gt_init_pos.x-vo_init_pos.x)
    # dy = (gt_init_pos.y-vo_init_pos.y)
    # dz = (gt_init_pos.z-vo_init_pos.z)
    print(f"Initial pose difference [gt-vo]:\ndx: {dx}\tdy: {dy}\tdz: {dz}")

    #===================
    # plot gound truth
    #===================
    # get timestamps
    #t_des = [trajectory[i][0] for i in range(len(trajectory))]
    # prepare coordinates
    p_des_x = []
    p_des_y = []
    p_des_z = []

    for i in range(len(gt)):
        # get initial pose and use for normalizing trajectories
        #p_des_x.append(-(gt[i][1].pose.pose.position.x)-dx)
        #p_des_y.append(-(gt[i][1].pose.pose.position.y))
        p_des_x.append(gt[i][1].pose.pose.position.x-dx)
        p_des_y.append(gt[i][1].pose.pose.position.y)
        p_des_z.append(gt[i][1].pose.pose.position.z)
        
        # debug
    # debug
    print(f"frames gt {len(gt)}")
    
    ax = fig.add_subplot(1,1,1, projection='3d')
    ax.set_xlabel('X', fontsize=10)
    ax.set_ylabel('Y', fontsize=10)
    ax.set_zlabel('Z', fontsize=10)
    ax.yaxis._axinfo['label']['space_factor'] = 3.0
    # p_des_y = p_des_y*(-1)
    ax.scatter(p_des_x, p_des_y, p_des_z, s=1)
    ax.set_zlim(-10,50) 
    # plot stereo vo estimation
    
    p_vo_x = []
    p_vo_y = []
    p_vo_z = []

    for i in range(len(vo)):
        # get initial pose and use for normalizing trajectories
        p_vo_x.append(vo[i][1].pose.pose.position.x)
        p_vo_y.append(vo[i][1].pose.pose.position.y)
        p_vo_z.append(vo[i][1].pose.pose.position.z)
        # debug
    print(f"frames stereo {len(vo)}")
    
    # ax = fig.add_subplot(1,2,2, projection='3d')
    # ax.set_title('stereo vo')
    ax.set_xlabel('X', fontsize=10)
    ax.set_ylabel('Y', fontsize=10)
    ax.set_zlabel('Z', fontsize=10)
    ax.yaxis._axinfo['label']['space_factor'] = 3.0
    # ax.set_zlim(-10,50) 
    ax.scatter(p_vo_x, p_vo_y, p_vo_z, s=1)
    
    # plot rgbd vo estimation
    
    p_rgbd_x = []
    p_rgbd_y = []
    p_rgbd_z = []

    pathlen_rgbd = 0
    for i in range(len(rgbd)):
        # get initial pose and use for normalizing trajectories
        p_rgbd_x.append(rgbd[i][1].pose.pose.position.x)
        p_rgbd_y.append(rgbd[i][1].pose.pose.position.y)
        p_rgbd_z.append(rgbd[i][1].pose.pose.position.z)

    print(f"frames rgbd {len(rgbd)}")
    
    # ax = fig.add_subplot(1,3,2, projection='3d')
    # ax.set_title('rgbd vo')
    ax.set_xlabel('X', fontsize=10)
    ax.set_ylabel('Y', fontsize=10)
    ax.set_zlabel('Z', fontsize=10)
    ax.yaxis._axinfo['label']['space_factor'] = 3.0
    # ax.set_zlim(-10,50) 
    ax.scatter(p_rgbd_x, p_rgbd_y, p_rgbd_z, s=1)
    ax.legend(['gt','stereo','rgb']) 

    #===== show plots 
    plt.show()

def viewImage(parser, frame):
    # use CVBridge
    imgL = parser.get_messages("/carla/ego_vehicle/rgb_left/image")
    cv_img = cvb.imgmsg_to_cv2(imgL, desired_encoding="passthrough")
    print(type(cv_img))


# timestamp at parser.get_messages("/carla/ego_vehicle/odometry")[*][0]
# nav_msgs/Odometry.msg at parser.get_messages("/carla/ego_vehicle/odometry")[*][1]

if __name__ == "__main__":

    try:    
        bag_file =  sys.argv[1]
    except IndexError:
        raise SystemExit("Usage: ./rosbag_plot.py <path_to_bagfile>")

    parser = BagFileParser(bag_file)

#         trajectory = parser.get_messages("/carla/ego_vehicle/odometry")[0][1] 
#         p_des_1 = [trajectory.points[i].positions[0] for i in range(len(trajectory.points))]
#         t_des = [trajectory.points[i].time_from_start.sec + trajectory.points[i].time_from_start.nanosec*1e-9 for i in range(len(trajectory.points))]
    gt = parser.get_messages("/carla/ego_vehicle/odometry") 
    gt_tmp = parser.get_messages("/carla/ego_vehicle/odometry") 
    vo00 = parser.get_messages("/odom_stereo_00")
    rgbd00 = parser.get_messages("/odom_rgbd_00") 
    vo01 = parser.get_messages("/odom_stereo_01")
    rgbd01 = parser.get_messages("/odom_rgbd_01") 
    vo02 = parser.get_messages("/odom_stereo_02")
    rgbd02 = parser.get_messages("/odom_rgbd_02") 
    vo03 = parser.get_messages("/odom_stereo_03")
    rgbd03 = parser.get_messages("/odom_rgbd_03") 
    # trajectory = parser.get_messages("/carla/ego_vehicle/odometry") 
    # viewImage(parser, 0)

    switchXY = 1
    invX = 0
    invY = 1


    # gt_tmp = gt
    # |+/-X|+/-Y|Switch XY|
    if (switchXY == 1):
        for i in range(len(gt)):
            gt[i][1].pose.pose.position.x = gt_tmp[i][1].pose.pose.position.y
            gt[i][1].pose.pose.position.y = gt_tmp[i][1].pose.pose.position.x
    if (invX == 1):
        for i in range(len(gt)):
            gt[i][1].pose.pose.position.x = -gt[i][1].pose.pose.position.x
    if (invY == 1):
        for i in range(len(gt)):
            gt[i][1].pose.pose.position.y = (-1.0)*gt[i][1].pose.pose.position.y

 
    plot2d_trajectories(gt, vo00, rgbd00, sys.argv[1], f"{sys.argv[2]}_00") 
    plot2d_trajectories(gt, vo01, rgbd01, sys.argv[1], f"{sys.argv[2]}_01")
    plot2d_trajectories(gt, vo02, rgbd02, sys.argv[1], f"{sys.argv[2]}_02")
    plot2d_trajectories(gt, vo03, rgbd03, sys.argv[1], f"{sys.argv[2]}_03")
    # plot_trajectories(gt, vo, rgbd) 
    # plot_trajectories(gt, vo) 
    # print_geom_msg_info(gt, 1, 'gt')
    # print_geom_msg_info(vo00, 1, 'stereo00')
    # print_geom_msg_info(rgbd00, 1, 'rgb-d00')
    # print_geom_msg_info(vo01, 1, 'stereo01')
    # print_geom_msg_info(rgbd01, 1, 'rgb-d01')
    # print_geom_msg_info(vo02, 1, 'stereo02')
    # print_geom_msg_info(rgbd02, 1, 'rgb-d02')

    print("#################################################")
    print("### Total Distance ###")
    gt_dist = dist(gt)
    print(f"Dist gt:\t{'{0:<20}'.format(f'{gt_dist}')}")
    print("### Estimated Distance ###")
    vo_dist = dist(vo00)
    print(f"Dist 2rgb[00]:\t{'{0:<20}'.format(f'{vo_dist}')}")
    rgbd_dist = dist(rgbd00)
    print(f"Dist rgb-d[00]:\t{'{0:<20}'.format(f'{rgbd_dist}')}")
    print('### Err ###')
    print(f"Err 2rgb[00]: {float(vo_dist-gt_dist)/(gt_dist*100.0)}")
    print(f"Err rgb-d[00]: {float(rgbd_dist-gt_dist)/(gt_dist*100.0)}")
    vo_dist = dist(vo01)
    print(f"Dist 2rgb[01]:\t{'{0:<20}'.format(f'{vo_dist}')}")
    rgbd_dist = dist(rgbd01)
    print(f"Dist rgb-d[01]:\t{'{0:<20}'.format(f'{rgbd_dist}')}")
    print('### Err ###')
    print(f"Err 2rgb[01]: {float(vo_dist-gt_dist)/(gt_dist*100.0)}")
    print(f"Err rgb-d[01]: {float(rgbd_dist-gt_dist)/(gt_dist*100.0)}")
    vo_dist = dist(vo02)
    print(f"Dist 2rgb[02]:\t{'{0:<20}'.format(f'{vo_dist}')}")
    rgbd_dist = dist(rgbd02)
    print(f"Dist rgb-d[02]:\t{'{0:<20}'.format(f'{rgbd_dist}')}")
    print('### Err ###')
    print(f"Err 2rgb[02]: {float(vo_dist-gt_dist)/(gt_dist*100.0)}")
    print(f"Err rgb-d[02]: {float(rgbd_dist-gt_dist)/(gt_dist*100.0)}")
    vo_dist = dist(vo03)
    print(f"Dist 2rgb[03]:\t{'{0:<20}'.format(f'{vo_dist}')}")
    rgbd_dist = dist(rgbd03)
    print(f"Dist rgb-d[03]:\t{'{0:<20}'.format(f'{rgbd_dist}')}")
    print('### Err ###')
    print(f"Err 2rgb[03]: {float(vo_dist-gt_dist)/(gt_dist*100.0)}")
    print(f"Err rgb-d[03]: {float(rgbd_dist-gt_dist)/(gt_dist*100.0)}")

