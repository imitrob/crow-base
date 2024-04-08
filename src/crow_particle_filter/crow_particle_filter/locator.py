import os
os.environ["OMP_NUM_THREADS"] = "10"

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from ros2param.api import call_get_parameters
import message_filters
from rclpy.time import Duration

#Pointcloud
from crow_msgs.msg import DetectionMask, SegmentedPointcloud
from sensor_msgs.msg import PointCloud2
from crow_cameras.utils.pcl_utils import ftl_pcl2numpy, ftl_numpy2pcl

from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

import json
import numpy as np
# import cv2
import cv_bridge

import pkg_resources
import time
import tf2_py as tf
import tf2_ros
import transforms3d as tf3

from ctypes import *  # convert float to uint32
from numba import jit
from crow_params.client import ParamClient

from crow_particle_filter.tracker.tracker_avatar import Avatar
import traceback as tb
from crow_monitor.profiler.profiling import StatTimer
from crow_utils.crow_config import get_config_file
from typing import Any


WORKSPACE_SETUP: dict[str, Any] = get_config_file("crow_setup/workspace.yaml")

USE_BOUNDS = False
# BASE_TF = np.array([
#     [5.179117321968078613e-01,4.867464601993560791e-01,-7.034524679183959961e-01,1.386950317382812500e+03],
#     [8.552457690238952637e-01,-3.118827939033508301e-01,4.138644337654113770e-01,-3.258915710449218750e+02],
#     [-1.794767379760742188e-02,-8.159700036048889160e-01,-5.778155922889709473e-01,6.687719116210937500e+02],
#     [0.000000000000000000e+00,0.000000000000000000e+00,0.000000000000000000e+00,1.000000000000000000e+0]
# ])
# BASE_TF[:3, 3] /= 1000
BASE_TF = np.eye(4)
# BASE_TF = BASE_TF.T
# BASE_TF = np.linalg.pinv(BASE_TF)
# BASE_TF = np.linalg.pinv(BASE_TF).T

APPLY_DTC = True  # Whether to apply depth-to-color transformation (use False if depth is aligned to color)

class Locator(Node):
    # MIN_X = -0.2  # work table boundaries - set specifically for each workplace
    # MAX_X = 1.25
    # MIN_Y = -0.4
    # MAX_Y = 1
    # MIN_Z = -0.15
    # MAX_Z = 0.15
    MIN_X = -0.2  # work table boundaries - set specifically for each workplace
    MAX_X = 1.25
    MIN_Y = -0.4
    MAX_Y = 1
    MIN_Z = -0.15
    MAX_Z = 0.9
    # MIN_X = -0.2  # work table boundaries - set specifically for each workplace
    # MAX_X = 1.25
    # MIN_Y = -0.4
    # MAX_Y = 1
    # MIN_Z = -0.01
    # MAX_Z = 0.15

    LR_X = 0.84  # x position of the cutout for the leftrobot
    R_Y_MIN = -0.15  # y start of th cutout for the robots
    R_Y_MAX = 0.21  # y end of th cutout for the robots
    RR_X = 0.2  # x position of the cutout for the leftrobot

    def __init__(self, node_name="locator", min_points_pcl=2, depth_range=(0.2, 4)):
        """
        @arg min_points_plc : >0, default 500, In the segmented pointcloud, minimum number for points (xyz) to be a (reasonable) cloud.
        @arg depth_range: tuple (int,int), (min, max) range for estimated depth [in mm], default 10cm .. 1m. Points in cloud
            outside this range are dropped. Sometimes camera fails to measure depth and inputs 0.0m as depth, this is to filter out those values.
        """
        super().__init__(node_name, start_parameter_services=False)
        # self.measurementTolerance = Duration(nanoseconds=1)

        # Wait for calibrator
        calib_client = self.create_client(GetParameters, '/calibrator/get_parameters')
        self.get_logger().info("Waiting for calibrator to setup cameras")
        calib_client.wait_for_service()
        # Retreive camera information
        self.image_topics, self.cameras, self.camera_instrinsics, self.camera_extrinsics, self.camera_frames = [p.string_array_value for p in call_get_parameters(node=self, node_name="/calibrator", parameter_names=["image_topics", "camera_namespaces", "camera_intrinsics", "camera_extrinsics", "camera_frames"]).values]
        while len(self.cameras) == 0: #wait for cams to come online
            self.get_logger().warn("No cams detected, waiting 2s.")
            time.sleep(2)
            self.image_topics, self.cameras, self.camera_instrinsics, self.camera_extrinsics, self.camera_frames = [p.string_array_value for p in call_get_parameters(node=self, node_name="/calibrator", parameter_names=["image_topics", "camera_namespaces", "camera_intrinsics", "camera_extrinsics", "camera_frames"]).values]

        # get global transform
        self.robot2global_tf = np.reshape([p.double_array_value for p in call_get_parameters(node=self, node_name="/calibrator", parameter_names=["robot2global_tf"]).values], (4, 4))
        self.global_frame_id = call_get_parameters(node=self, node_name="/calibrator", parameter_names=["global_frame_id"]).values[0].string_value

        # Set camera parameters and topics
        self.camera_instrinsics = [json.loads(cintr) for cintr in self.camera_instrinsics]
        self.camera_extrinsics = [json.loads(cextr) for cextr in self.camera_extrinsics]
        self.mask_topics = [cam + "/detections/masks" for cam in self.cameras] #input masks from 2D rgb (from our detector.py)
        self.pcl_topics = [cam + "/camera/depth/color/points" for cam in self.cameras] #input pcl data (from RS camera)

        self.depth_min, self.depth_max = depth_range
        assert self.depth_min < self.depth_max and self.depth_min > 0.0

        # create publisher for located objects (segmented PCLs)
        qos = QoSProfile(depth=40, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.pubPCL = self.create_publisher(SegmentedPointcloud, '/detections/segmented_pointcloud', qos_profile=QoSProfile(depth=20, reliability=QoSReliabilityPolicy.RELIABLE))
        self.pubPCL_debug = self.create_publisher(PointCloud2, '/detections/segmented_pointcloud_debug', qos_profile=qos)

        # For now, every camera published segmented hand data PCL.
        self.pubPCL_avatar = self.create_publisher(SegmentedPointcloud, '/detections/segmented_pointcloud_avatar', qos_profile=qos)

        self.cvb = cv_bridge.CvBridge()
        self.mask_dtype = {'names':['f{}'.format(i) for i in range(2)], 'formats':2 * [np.int32]}

        #create listeners, synchronized for each camera (Mask + PCL)
        for i, (cam, pclTopic, maskTopic, camera_instrinsics, camera_extrinsics) in enumerate(zip(self.cameras, self.pcl_topics, self.mask_topics, self.camera_instrinsics, self.camera_extrinsics)):
            # convert camera data to numpy
            self.camera_instrinsics[i]["camera_matrix"] = np.array(camera_instrinsics["camera_matrix"], dtype=np.float32)
            self.camera_instrinsics[i]["distortion_coefficients"] = np.array(camera_instrinsics["distortion_coefficients"], dtype=np.float32)

            self.camera_extrinsics[i]["dtc_tf"] = np.array(camera_extrinsics["dtc_tf"], dtype=np.float32)  # dtc = depth to color sensor transform
            self.camera_extrinsics[i]["ctg_tf"] = np.array(camera_extrinsics["ctg_tf"], dtype=np.float32)  # ctg = color/camera to global coord system transform

            # create approx syncro callbacks
            cb_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
            subPCL = message_filters.Subscriber(self, PointCloud2, pclTopic, qos_profile=qos, callback_group=cb_group) #listener for pointcloud data from RealSense camera
            subMasks = message_filters.Subscriber(self, DetectionMask, maskTopic, qos_profile=qos, callback_group=cb_group) #listener for masks from detector node
            sync = message_filters.ApproximateTimeSynchronizer([subPCL, subMasks], 50, slop=0.15) #create and register callback for syncing these 2 message streams, slop=tolerance [sec]
            sync.registerCallback(lambda pcl_msg, mask_msg, cam=cam: self.detection_callback(pcl_msg, mask_msg, cam))

            self.get_logger().info(f"Connected to camera {cam} on PCL topic '{pclTopic}' and mask topic '{maskTopic}'.")

        self.pclient = ParamClient()
        self.pclient.define("locator_alive", time.time())
        self.pclient.define("located_objects", [])
        self.pclient.define("located_avatars", [])

        self.min_points_pcl = min_points_pcl
        self.avatar_data_classes = Avatar.AVATAR_PARTS

        self.camera_data = {}  # store camera data in a dict
        for idx, cam in enumerate(self.cameras):
            self.camera_data[cam] = self.generateCameraData(cam)

        StatTimer.init()

    @staticmethod
    # @jit(nopython=True, parallel=False)
    @jit("float32[:, ::1](float32[:, ::1],float32[:, ::1])", nopython=True, parallel=False, nogil=True, fastmath=True, cache=True)
    def project(camera_matrix, point_cloud):
        # converts pcl (shape 3,N) of [x,y,z] (3D) into image space (with cam_projection matrix) -> [u,v,w] -> [u/w, v/w] which is in 2D
        imspace = np.dot(camera_matrix, point_cloud)
        # [u,v,w] -> [u/w, v/w, w/w] -> [u',v'] = 2D
        return imspace[:2, :] / imspace[2, :]

    @staticmethod
    # @jit(nopython=True, parallel=False)
    @jit("List(int64[::1])(int32[:, ::1],uint8[:, :, ::1])", nopython=True, parallel=False, nogil=True, fastmath=True, cache=True)
    def compareMasksPCL_fast(idxs, masks):
        idxs1d = idxs[1, :] + idxs[0, :] * masks[0].shape[1]
        wheres = []
        masks_raveled = masks.reshape(masks.shape[0], -1)  # ravel all masks
        # set (0, 0) to 0 - "wrong" pixels are projected to this
        masks_raveled[:, 0] = 0
        for mask in masks_raveled:
            wheres.append(np.nonzero(mask[idxs1d])[0])
        return wheres

    @staticmethod
    @jit("float32[:, :](float32[:, ::1],float32[:, ::1])", nopython=True, parallel=False, nogil=True, fastmath=True, cache=True)
    def numba_dot(pcl, matrix):  # requires pcl Nx3
        padded_pcl = np.vstack((pcl.T, np.ones((1, pcl.shape[0]), dtype=np.float32)))
        transformed_pcl = np.dot(matrix, padded_pcl)[:3, :]
        return transformed_pcl

    @staticmethod
    @jit("float32[:, :](float32[:, ::1],float32[:, ::1],int64[::1])", nopython=True, parallel=False, nogil=True, fastmath=True, cache=True)
    def numba_full_dot_filter(pcl, matrix, where):  # requires pcl 3xN
        pcl = pcl[:, where]
        padded_pcl = np.vstack((pcl, np.ones((1, pcl.shape[1]), dtype=np.float32)))
        transformed_pcl = np.dot(matrix, padded_pcl)
        return transformed_pcl[:3, :] / transformed_pcl[3, :]

    def detection_callback(self, pcl_msg, mask_msg, camera):
        self.pclient.locator_alive = time.time()
        if not mask_msg.masks:
            # self.get_logger().fatal("no masks, no party. Quitting early.")
            return  # no mask detections (for some reason)

        # self.get_logger().fatal("Yo, masks, party on!.")
        # StatTimer.enter(f"loc____{camera}")
        # StatTimer.enter(f"loc_{camera}_preproc")
        # StatTimer.enter(f"loc_{camera}_/extract")
        
        cameraData = self.getCameraData(camera)
        masks = np.array([self.cvb.imgmsg_to_cv2(mask, "mono8") for mask in mask_msg.masks])
        object_ids, class_names, scores = mask_msg.object_ids, mask_msg.class_names, mask_msg.scores

        point_cloud, _, rgb_raw = ftl_pcl2numpy(pcl_msg)
        # StatTimer.exit(f"loc_{camera}_/extract")
        # StatTimer.enter(f"loc_{camera}_/dot")
        tf_mat = cameraData["dtc_tf"]
        # tf_mat = np.linalg.inv(cameraData["dtc_tf"])
        # point_cloud = np.dot(tf_mat, np.pad(point_cloud, ((0, 1), (0, 0)), mode="constant", constant_values=1))[:3, :]
        if APPLY_DTC:
            point_cloud = Locator.numba_dot(np.ascontiguousarray(point_cloud), tf_mat) # <<<<<<<
        else:
            point_cloud = np.ascontiguousarray(point_cloud.T.astype(np.float32))  # secret hack to make sure this works...
        # StatTimer.exit(f"loc_{camera}_/dot")
        ##process pcd
        # 1. convert 3d pcl to 2d image-space
        # StatTimer.enter(f"loc_{camera}_/project")
        imspace = self.project(cameraData["camera_matrix"], point_cloud) # converts pcl (shape 3,N) of [x,y,z] (3D) into image space (with cam_projection matrix) -> [u,v,w] -> [u/w, v/w] which is in 2D
        # StatTimer.exit(f"loc_{camera}_/project")
        # StatTimer.enter(f"loc_{camera}_/filter")
        imspace[np.isnan(imspace)] = -1 #marking as -1 results in deletion (omission) of these points in 3D, as it's impossible to match to -1
        # dist = np.linalg.norm(point_cloud, axis=0)
        # bad_points = np.logical_or(point_cloud[2, :] < self.depth_min, point_cloud[2, :] > self.depth_max)
        # bad_points = np.logical_or(np.logical_or(point_cloud[2, :] < self.depth_min, point_cloud[2, :] > self.depth_max), point_cloud[0, :] < -0.65)
        # bad_points = np.logical_or(np.logical_or(dist < self.depth_min, dist > self.depth_max), point_cloud[0, :] < -0.65)
        # imspace[:, bad_points] = -1
        # assert np.isnan(imspace).any() == False, 'must not have NaN element'  # sorry, but this is expensive (half a ms) #optimizationfreak
        imspace = imspace.astype(np.int32)
        mshape = masks[0].shape
        imspace[:,
                (imspace[0] < 0) |
                (imspace[1] < 0) |
                (imspace[1] >= mshape[0]) |
                (imspace[0] >= mshape[1])
                ] = 0
        # StatTimer.exit(f"loc_{camera}_/filter")
        # StatTimer.enter(f"loc_{camera}_/wheres")

        wheres = self.compareMasksPCL_fast(imspace[[1, 0], :], masks)
        ctg_tf_mat = cameraData["ctg_tf"]
        # StatTimer.exit(f"loc_{camera}_/wheres")

        # StatTimer.exit(f"loc_{camera}_preproc")
        # StatTimer.enter(f"loc_{camera}_postproc")
        for where, object_id, class_name, score in zip(wheres, object_ids, class_names, scores):
            # 2. segment PCL & compute median
            # skip pointclouds with too few datapoints to be useful
            # if class_name in self.avatar_data_classes:
            #     self.get_logger().error(f"{class_name}")
            if len(where) < self.min_points_pcl:
                self.get_logger().warn(
                    "Cam {}: Skipping pcl {} for '{}' mask_score: {} -- too few datapoints. ".format(camera, len(where), class_name, score))
                continue

            seg_pcd = Locator.numba_full_dot_filter(point_cloud, ctg_tf_mat, where)
            # seg_pcd = np.dot(ctg_tf_mat, np.pad(point_cloud[:, where], ((0, 1), (0, 0)), mode="constant", constant_values=1))
            # seg_pcd = seg_pcd[:3, :] / seg_pcd[3, :]
            # filter points outside the main work area
            if USE_BOUNDS:
                m = np.mean(seg_pcd, axis=1)
                if m[2] < self.MIN_Z or m[2] > self.MAX_Z or m[0] < self.MIN_X or m[0] > self.MAX_X or m[1] < self.MIN_Y or m[1] > self.MAX_Y:
                    self.get_logger().warn("Cam {}: Skipping '{}'  -- out of bounds. ".format(camera, class_name))
                    continue
                elif m[1] > self.R_Y_MIN and m[1] < self.R_Y_MAX and (m[0] > self.LR_X or m[0] < self.RR_X):
                    self.get_logger().warn("Cam {}: Skipping '{}'  -- in the robot cutout. ".format(camera, class_name))
                    continue
            # if "2" in camera:
            # total_pcd = np.c_[total_pcd, seg_pcd]

            if rgb_raw is not None:
                seg_color = rgb_raw[where]
            else:
                seg_color = None

            # output: create back a pcl from seg_pcd and publish it as ROS PointCloud2
            segmented_pcl = ftl_numpy2pcl(seg_pcd, pcl_msg.header, seg_color)
            segmented_pcl.header.frame_id = self.global_frame_id
            segmented_pcl.header.stamp = pcl_msg.header.stamp
            # wrap together PointCloud2 + label + score => SegmentedPointcloud
            seg_pcl_msg = SegmentedPointcloud()
            seg_pcl_msg.header = segmented_pcl.header
            seg_pcl_msg.header.stamp.nanosec += 2
            seg_pcl_msg.pcl = segmented_pcl
            seg_pcl_msg.object_id = object_id
            seg_pcl_msg.label = str(class_name)
            seg_pcl_msg.confidence = float(score)

            # Data about hand position is published on different topic

            if class_name in self.avatar_data_classes:
                self.pubPCL_avatar.publish(seg_pcl_msg)
                self.pclient.located_avatars += [time.time()]
                # self.get_logger().error(f"{class_name} = {np.mean(seg_pcd, 1)}")
            else:
                # self.get_logger().warn(f">>> {seg_pcl_msg.label}")
                self.pubPCL.publish(seg_pcl_msg)
                self.pclient.located_objects += [time.time()]
            # total_processed += 1
            # self.runtimes.append(time.time() - start_loop)

        # total_runtime = time.time() - start
        # now = self.get_clock().now()
        # mdelay = (now - rclpy.time.Time.from_msg(mask_msg.header.stamp)).nanoseconds * 1e-9
        # pdelay = (now - rclpy.time.Time.from_msg(pcl_msg.header.stamp)).nanoseconds * 1e-9
        # self.get_logger().error(f"Processed {total_processed} objects for camera {camera}. Total runtime was {total_runtime:0.3f}. Delay {mdelay:0.3f} | {pdelay:0.3f}")
        # if len(total_pcd) > 0:
        #     dmsg = ftl_numpy2pcl(total_pcd, pcl_msg.header)
        #     dmsg.header.frame_id = self.global_frame_id
        #     dmsg.header.stamp = pcl_msg.header.stamp
        #     self.pubPCL_debug.publish(dmsg)
        # StatTimer.exit(f"loc_{camera}_postproc")
        # StatTimer.exit(f"loc____{camera}")

    """
    |    | section                                        |   mean time |   median time | st.dev.   |   # of runs |         min |        max |   severity |
    |---:|:-----------------------------------------------|------------:|--------------:|:----------|------------:|------------:|-----------:|-----------:|
    |  0 | infer                                          | 0.00987885  |   0.0086577   | ±0.00446  |        4902 | 0.00741907  | 0.104714   |          0 |
    |  1 | annotate                                       | 0.00432898  |   0.00324148  | ±0.00380  |        4902 | 0.00176691  | 0.0867532  |          0 |
    |  2 | compute masks                                  | 0.0131837   |   0.010701    | ±0.00962  |        4902 | 0.00314511  | 0.176977   |          0 |
    |  3 | full detection                                 | 0.0727524   |   0.0653707   | ±0.02722  |        4901 | 0.03389     | 0.293724   |          0 |
    |  4 | pose_/camera3/color/image_raw_detect           | 0.00837627  |   0.00578731  | ±0.00576  |        1754 | 0.00433624  | 0.0636786  |          0 |
    |  5 | pose_/camera3/color/image_raw_parse_objects    | 0.00251498  |   0.00105717  | ±0.00424  |        1754 | 0.000148485 | 0.0489378  |          0 |
    |  6 | pose_/camera3/color/image_raw_parse_detections | 0.000197964 |   9.37755e-05 | ±0.00047  |        1754 | 3.2719e-05  | 0.00860396 |          0 |
    |  7 | tracking                                       | 0.00389008  |   0.00280691  | ±0.00319  |         730 | 0.0011522   | 0.0240778  |          0 |
    |  8 | correcting uuids                               | 0.000183922 |   6.4952e-05  | ±0.00061  |         730 | 2.3269e-05  | 0.0110991  |          0 |
    |  9 | Filter PCL publish                             | 0.0522357   |   0.0458922   | ±0.02998  |         729 | 0.0121337   | 0.176717   |          0 |
    | 10 | Filter publishing                              | 0.0629023   |   0.0547348   | ±0.03351  |         729 | 0.0166986   | 0.196542   |          0 |
    | 11 | Filter node update loop                        | 0.106083    |   0.096049    | ±0.05109  |         729 | 0.026784    | 0.291739   |          0 |
    | 12 | loc_/camera3_preproc                           | 0.031347    |   0.0225566   | ±0.02207  |         216 | 0.0100997   | 0.115096   |          0 |
    | 13 | loc_/camera3_postproc                          | 0.0136538   |   0.00868847  | ±0.01277  |         216 | 0.00150818  | 0.0799383  |          0 |
    | 14 | loc____/camera3                                | 0.0460449   |   0.04078     | ±0.02613  |         216 | 0.014108    | 0.153175   |          0 |
    | 15 | loc_/camera2_preproc                           | 0.0275718   |   0.0197994   | ±0.01812  |         241 | 0.0104992   | 0.116178   |          0 |
    | 16 | loc_/camera2_postproc                          | 0.0180658   |   0.0128785   | ±0.01451  |         241 | 0.00246268  | 0.0784233  |          0 |
    | 17 | loc____/camera2                                | 0.0464409   |   0.0420518   | ±0.02474  |         241 | 0.0148731   | 0.154777   |          0 |
    | 18 | loc_/camera1_preproc                           | 0.0327599   |   0.0244479   | ±0.02543  |         161 | 0.0118295   | 0.186795   |          0 |
    | 19 | loc_/camera1_postproc                          | 0.0279398   |   0.0208749   | ±0.02630  |         161 | 0.00360949  | 0.203428   |          0 |
    | 20 | loc____/camera1                                | 0.0613202   |   0.0536385   | ±0.03829  |         161 | 0.0174523   | 0.293714   |          0 |
    | 21 | loc_/camera4_preproc                           | 0.0339814   |   0.0266218   | ±0.02275  |         159 | 0.0105088   | 0.120053   |          0 |
    | 22 | loc_/camera4_postproc                          | 0.0247221   |   0.0175217   | ±0.02270  |         159 | 0.00375549  | 0.143592   |          0 |
    | 23 | loc____/camera4                                | 0.0593693   |   0.0519074   | ±0.03194  |         159 | 0.0180796   | 0.16181    |          0 |

    cache, nogil, fastmath, 10 threads
    ^C>>>>>>> RUNTIME SUMMARY <<<<<<<
    |    | section                                        |   mean time |   median time | st.dev.   |   # of runs |         min |        max |   severity |
    |---:|:-----------------------------------------------|------------:|--------------:|:----------|------------:|------------:|-----------:|-----------:|
    |  0 | tracking                                       | 0.000727666 |   0.000657302 | ±0.00049  |         611 | 0.00040242  | 0.00929558 |          0 |
    |  1 | correcting uuids                               | 0.000196326 |   5.231e-05   | ±0.00131  |         611 | 2.321e-05   | 0.0234226  |          0 |
    |  2 | loc_/camera4_postproc                          | 0.0238099   |   0.0191918   | ±0.01989  |         230 | 0.00331954  | 0.122291   |          0 |
    |  3 | Filter PCL publish                             | 0.0461176   |   0.0434984   | ±0.02063  |         611 | 0.010752    | 0.136711   |          0 |
    |  4 | Filter publishing                              | 0.0521542   |   0.0491279   | ±0.02141  |         611 | 0.0148572   | 0.142336   |          0 |
    |  5 | Filter node update loop                        | 0.0869144   |   0.0824946   | ±0.02898  |         611 | 0.0368627   | 0.221321   |          0 |
    |  6 | pose_/camera3/color/image_raw_detect           | 0.00667881  |   0.00546097  | ±0.00302  |        1468 | 0.00432401  | 0.0359523  |          0 |
    |  7 | pose_/camera3/color/image_raw_parse_objects    | 0.00160051  |   0.000633612 | ±0.00284  |        1468 | 0.00014568  | 0.0409175  |          0 |
    |  8 | pose_/camera3/color/image_raw_parse_detections | 0.00014491  |   8.083e-05   | ±0.00027  |        1468 | 3.434e-05   | 0.00555456 |          0 |
    |  9 | loc_/camera2_/extract                          | 0.000838472 |   0.000644918 | ±0.00069  |         326 | 0.000374861 | 0.0079666  |          0 |
    | 10 | loc_/camera2_/dot                              | 0.00789559  |   0.00501714  | ±0.00721  |         326 | 0.00292316  | 0.0592942  |          0 |
    | 11 | loc_/camera2_/project                          | 0.00386907  |   0.00241083  | ±0.00448  |         326 | 0.00106356  | 0.0319118  |          0 |
    | 12 | loc_/camera2_/filter                           | 0.00484366  |   0.00309385  | ±0.00439  |         326 | 0.0020421   | 0.043609   |          0 |
    | 13 | loc_/camera2_/wheres                           | 0.00692733  |   0.00458728  | ±0.00508  |         326 | 0.00277691  | 0.0344607  |          0 |
    | 14 | loc_/camera2_preproc                           | 0.0261059   |   0.0196704   | ±0.01499  |         325 | 0.010524    | 0.0825728  |          0 |
    | 15 | loc_/camera2_postproc                          | 0.0147999   |   0.0100831   | ±0.01360  |         326 | 0.00264347  | 0.119806   |          0 |
    | 16 | loc____/camera2                                | 0.0418283   |   0.0379185   | ±0.02089  |         326 | 0.0149589   | 0.136878   |          0 |
    | 17 | loc_/camera4_/extract                          | 0.00105818  |   0.000790309 | ±0.00094  |         229 | 0.000425771 | 0.0112097  |          0 |
    | 18 | loc_/camera4_/dot                              | 0.00882165  |   0.00518326  | ±0.00853  |         229 | 0.00295006  | 0.0540639  |          0 |
    | 19 | loc_/camera4_/project                          | 0.00418163  |   0.0023573   | ±0.00551  |         229 | 0.00102228  | 0.0350517  |          0 |
    | 20 | loc_/camera4_/filter                           | 0.00553587  |   0.00374016  | ±0.00407  |         229 | 0.00219828  | 0.0285324  |          0 |
    | 21 | loc_/camera4_/wheres                           | 0.00974814  |   0.00685108  | ±0.00675  |         229 | 0.00354071  | 0.0534583  |          0 |
    | 22 | loc_/camera4_preproc                           | 0.0316854   |   0.0255383   | ±0.01835  |         229 | 0.0113157   | 0.0927509  |          0 |
    | 23 | loc____/camera4                                | 0.0558354   |   0.050177    | ±0.02762  |         229 | 0.0163116   | 0.148716   |          0 |
    | 24 | loc_/camera1_/extract                          | 0.00140683  |   0.00107551  | ±0.00134  |         237 | 0.000572613 | 0.0169669  |          0 |
    | 25 | loc_/camera1_/dot                              | 0.00948861  |   0.00590647  | ±0.00934  |         237 | 0.00285342  | 0.0629345  |          0 |
    | 26 | loc_/camera1_/project                          | 0.00510438  |   0.00277056  | ±0.00616  |         237 | 0.000991798 | 0.0402597  |          0 |
    | 27 | loc_/camera1_/filter                           | 0.005652    |   0.00379907  | ±0.00457  |         237 | 0.00251827  | 0.0459442  |          0 |
    | 28 | loc_/camera1_/wheres                           | 0.0118148   |   0.00776976  | ±0.00873  |         237 | 0.00431749  | 0.065344   |          0 |
    | 29 | loc_/camera1_preproc                           | 0.0356594   |   0.0306422   | ±0.01942  |         237 | 0.0135537   | 0.121896   |          0 |
    | 30 | loc_/camera1_postproc                          | 0.0268094   |   0.0242572   | ±0.01870  |         236 | 0.00410864  | 0.129092   |          0 |
    | 31 | loc____/camera1                                | 0.0633679   |   0.0563179   | ±0.02768  |         236 | 0.0180635   | 0.186222   |          0 |
    | 32 | pose____/camera3/color/image_raw               | 0.0249417   |   0.0228918   | ±0.00781  |          11 | 0.0147753   | 0.0441831  |          0 |
    | 33 | loc_/camera3_/extract                          | 0.00026571  |   0.000269419 | ±0.00005  |           3 | 0.00020873  | 0.00031898 |          0 |
    | 34 | loc_/camera3_/dot                              | 0.00633031  |   0.0059826   | ±0.00183  |           3 | 0.00428166  | 0.00872666 |          0 |
    | 35 | loc_/camera3_/project                          | 0.00745604  |   0.00405488  | ±0.00671  |           3 | 0.00148671  | 0.0168265  |          0 |
    | 36 | loc_/camera3_/filter                           | 0.00635487  |   0.00502665  | ±0.00189  |           3 | 0.0050105   | 0.00902745 |          0 |
    | 37 | loc_/camera3_/wheres                           | 0.00201703  |   0.0021574   | ±0.00044  |           3 | 0.0014157   | 0.00247799 |          0 |
    | 38 | loc_/camera3_preproc                           | 0.0235414   |   0.0243562   | ±0.00632  |           3 | 0.015428    | 0.0308398  |          0 |
    | 39 | loc_/camera3_postproc                          | 0.000454609 |   0.000161089 | ±0.00045  |           3 | 0.000118149 | 0.00108459 |          0 |
    | 40 | loc____/camera3                                | 0.0241881   |   0.0257003   | ±0.00639  |           3 | 0.0157215   | 0.0311424  |          0 |
    [INFO] [profiler_node]: Storing runtime summary to file /home/imitrob/crow2/runtimes_2022_06_07_12_39_20.csv.

    """
    # def compareMaskPCL(self, mask_array, projected_points):  # OLD, SLOW version
    #     a = mask_array.T.astype(np.int32).copy()
    #     b = projected_points.T.copy()
    #     self.mask_dtype = {'names':['f{}'.format(i) for i in range(2)], 'formats':2 * [np.int32]}
    #     result = np.intersect1d(a.view(self.mask_dtype), b.view(self.mask_dtype), return_indices=True)
    #     return result[2]

    def getCameraData(self, camera):
        # idx = self.cameras.index(camera)
        return self.camera_data[camera]

    def generateCameraData(self, camera):
        idx = self.cameras.index(camera)
        self.get_logger().info(str((self.robot2global_tf @ BASE_TF @ self.camera_extrinsics[idx]["ctg_tf"]).astype(np.float32).tolist()))
        self.get_logger().info(str(self.robot2global_tf @ BASE_TF @ self.camera_extrinsics[idx]["ctg_tf"].astype(np.float32).tolist()))
        self.get_logger().info(str(BASE_TF.astype(np.float32).tolist()))
        self.get_logger().info(str(self.camera_extrinsics[idx]["ctg_tf"].astype(np.float32).tolist()))
        self.get_logger().info("=====================")
        return {
            "camera": camera,
            "image_topic": self.image_topics[idx],
            "camera_matrix": self.camera_instrinsics[idx]["camera_matrix"].astype(np.float32),
            "distortion_coefficients": self.camera_instrinsics[idx]["distortion_coefficients"],
            "dtc_tf": self.camera_extrinsics[idx]["dtc_tf"].astype(np.float32),
            # "ctg_tf": self.camera_extrinsics[idx]["ctg_tf"],
            # "ctg_tf": (robot_2_global @ realsense_2_robot @ self.camera_extrinsics[idx]["ctg_tf"]).astype(np.float32),
            "ctg_tf": (self.robot2global_tf @ BASE_TF @ self.camera_extrinsics[idx]["ctg_tf"]).astype(np.float32),
            # "ctg_tf": (self.robot2global_tf @ self.camera_extrinsics[idx]["ctg_tf"]).astype(np.float32),
            "optical_frame": self.camera_frames[idx],
            "mask_topic": self.mask_topics[idx],
            "pcl_topic": self.pcl_topics[idx],
        }


def main():
    rclpy.init()
    locator = Locator()
    try:
        n_threads = len(locator.cameras)
        mte = MultiThreadedExecutor(num_threads=n_threads, context=rclpy.get_default_context())
        rclpy.spin(locator, executor=mte)
        # rclpy.spin(locator)
    except KeyboardInterrupt:
        # from scipy.stats import describe
        # print(describe(locator.runtimes))
        print("User requested shutdown.")
    except BaseException as e:
        print("Terminated with and error: {e}")
        tb.print_exc()
    finally:
        locator.destroy_node()


if __name__ == "__main__":
    main()