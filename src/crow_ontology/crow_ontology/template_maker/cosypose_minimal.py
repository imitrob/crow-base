#!/usr/bin/env python3
import numpy as np
import sys, zmq, time
from zmq.error import Again as zmqErrorAgain, ZMQError
import pickle
import json
import cv2
import rclpy
from rclpy.node import Node

import spatialmath as sm
from spatialmath import UnitQuaternion

from geometry_msgs.msg import Pose, Point, Quaternion
import crow_ontology.crow_ontology.template_maker.ycb_data as ycb_data
from copy import deepcopy

def se3_to_Pose(se3):
    p = se3.t
    q = UnitQuaternion(se3).vec_xyzs
    pose = Pose(position=Point(x=p[0], y=p[1], z=p[2]), orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))
    return pose

def Pose_to_SE3(pose):
    p = np.array([pose.position.x, pose.position.y, pose.position.z])
    q_wxyz = np.array([pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z])
    T_ori = np.diag([0.,0.,0.,1.])
    T_ori[0:3,0:3] = UnitQuaternion(q_wxyz).R
    return sm.SE3(sm.base.transl(*p) @ sm.SE3(T_ori).A)

def Pose_list_to_SE3(pose):
    p = np.array([pose[0], pose[1], pose[2]])
    q_wxyz = np.array([pose[6], pose[3], pose[4], pose[5]])
    T_ori = np.diag([0.,0.,0.,1.])
    T_ori[0:3,0:3] = UnitQuaternion(q_wxyz).R
    return sm.SE3(sm.base.transl(*p) @ sm.SE3(T_ori).A)

def T_to_Pose(T):
    #T = np.diag([1.,1.,1.,1.])
    se3 = sm.SE3(T)
    p = se3.t
    q = UnitQuaternion(se3).vec_xyzs
    return Pose(position=Point(x=p[0], y=p[1], z=p[2]), orientation=Quaternion(x=q[0],y=q[1],z=q[2],w=q[3]))


class ZMQArmerServer(Node):
    ''' Interface communicating with real robot server
    '''
    def __init__(self, dt=0.05, scene_pub_dt=0.5, pub_ros_topic="/tree/scene_in"):
        super().__init__("zmqarmer_server")
        context = zmq.Context()
        self.cosyposesocket = context.socket(zmq.REQ)
        self.cosyposesocket.connect("tcp://192.168.88.146:5559")

        self.scene_objects = {}
        self.seq = 0
        self.dt = dt
        self.scene_pub_modulo_dt = int(scene_pub_dt / dt)

    def get_object_positions(self):
        self.scene_objects = {}
        objects_list = list(ycb_data.COSYPOSE2NAME.keys())
        for object_name in objects_list:
            self.cosyposesocket.send_string(f"tf,{object_name}")
            msg = self.cosyposesocket.recv()
            state = pickle.loads(msg)
            if len(state['objects']) == 0:
                continue

            object_data = state['objects'][0]
            # blacklist - ghost objects
            if ycb_data.COSYPOSE2NAME[object_data['label']] in ['foam brick', 'foam_brick', 'scissors']:
                continue
            #type = (ycb_data.NAME2TYPE[ycb_data.COSYPOSE2NAME[object['label']]]).capitalize()
            name = ycb_data.COSYPOSE2NAME[object_data['label']]
            position_real = np.array(object_data['pose'][0])
            quaternion = np.array(object_data['pose'][1])
            pose_list = np.hstack([position_real, quaternion])

            self.scene_objects[name] = Pose_list_to_SE3(pose_list)
        print("scene objects: ", self.scene_objects)

    def step_scene(self):
        time.sleep(self.dt)
        if self.seq % self.scene_pub_modulo_dt == 0:
            self.get_object_positions()

        self.seq += 1

def main():
    rclpy.init()
    s = ZMQArmerServer()
    while rclpy.ok():
        for _ in range(10):
            s.step_scene()

if __name__ == '__main__':
    main()
