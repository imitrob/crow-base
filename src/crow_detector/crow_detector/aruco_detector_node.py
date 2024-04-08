from pathlib import Path
import yaml
import rclpy, time, sys
from rclpy.node import Node
from teleop_msgs.msg import HRICommand
from crow_msgs.msg import FilteredPose, PclDimensions
from geometry_msgs.msg import Pose, Point, Quaternion

from imitrob_templates.small_ontology_scene_reader import SceneOntologyClient
from crow_ontology.crowracle_client import CrowtologyClient

import cv2
from cv2 import aruco
from scipy.spatial.transform import Rotation as R
import numpy as np
import rclpy
import sensor_msgs
from crow_monitor.profiler.profiling import StatTimer
from crow_utils.crow_config import load_yaml_config
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import Pose, Point, Quaternion

import crow_detector.utils.camera as utils_camera

from bt_msgs.msg import SceneFeedback
from bt_msgs.msg import SimpleObject
from std_msgs.msg import Int16MultiArray

# Recent callib. provided by Jan
T_CALIB = np.array([
    [-0.00141572, -0.01912743, -0.99981605, 1.32933866],
    [ 0.99958873, 0.02860982, -0.00196274, 0.0273636 ],
    [ 0.02864209, -0.99940763, 0.01907906, 0.18158126],
    [ 0. , 0. , 0. , 1. ]
])


CAMERA_MATRIX = np.array([[632.6622924804688, 0.0, 642.265625], [0.0, 631.8963012695312, 414.34735107421875], [0.0, 0.0, 1.0]])
DIST_COEF = np.array([-0.05664148926734924, 0.06621451675891876, -0.000982369645498693, -3.391642894712277e-05, -0.02081259898841381])

#1.32 0.03 0.19 -0.49119764 -0.5 0.5 0.50865005

# CAM_2_ROB_T = np.array([-0.036, -0.095, 1.049])
# CAM_2_ROB_Q = np.array([0.586, 0.602, -0.394, 0.373])
BASE_2_SCENECAM_T = np.array([1.32, 0.03, 0.19])
BASE_2_SCENECAM_Q = np.array([-0.49119764, -0.5, 0.5, 0.50865005])

IGNORE_UNKNOWN_ARUCO_MARKERS = True

def transform_marker_position(rvec, tvec):
    # Convert rotation vector to rotation matrix
    R_cam_marker, _ = cv2.Rodrigues(rvec)
    # Construct homogeneous transformation matrix for marker in camera base
    T_cam_marker = np.eye(4)
    T_cam_marker[:3, :3] = R_cam_marker
    T_cam_marker[:3, 3] = tvec
    # Construct homogeneous transformation matrix for camera in robot base
    R_robot = R.from_quat(BASE_2_SCENECAM_Q)
    T_robot_cam = np.eye(4)
    T_robot_cam[:3, :3] = R_robot.as_matrix()
    T_robot_cam[:3, 3] = np.array(BASE_2_SCENECAM_T)
    T_robot_cam = T_CALIB

    # Calculate homogeneous transformation matrix for marker in robot base
    # T_robot_cam = np.linalg.pinv(T_cam_robot)
    T_robot_marker = np.matmul(T_robot_cam, T_cam_marker)

    # Extract marker position and orientation in robot base
    marker_pos = T_robot_marker[:3, 3]
    orientation = R.from_matrix(T_cam_marker[:3, :3]).as_quat()
    return marker_pos, orientation

def publish_aruco(node, qos, topic_prefix, topic, msg_type):
    topic = "/" + topic_prefix + "/" + topic
    publisher = node.create_publisher(msg_type, topic, qos_profile=qos)
    return topic, publisher


class CrowVision(Node):
    """
    ROS2 node for CNN used in Crow.

    Run as
    `ros2 run crow_detector detector`

    This node listens on network for topic "/camera1/color/image_raw",
    obtains raw RGB image, processes it in neural net, and publishes results in form of (for given camera):
    - "/detections/image_annot" - processed image with visualized masks, bboxes and labels
    - "/detections/masks"
    - "/detections/labels"
    - "/detections/bboxes"
    - "/detections/confidences"
    """

    qos = QoSProfile(
        depth=10,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
    )

    def __init__(self):
        super().__init__("crow_detector")

        # Attempt to load config
        self.config = load_yaml_config("crow_detector/config_affordance.yaml")
        self.config_objects = load_yaml_config("crow_detector/config_objects.yaml")
        self.ids_for_parts = []
        objects = self.config_objects['objects']
        for co in objects.keys():
            for marker_key in objects[co].keys():
                self.ids_for_parts.append(objects[co][marker_key]['aruco_id'])
        print("self.ids_for_parts: ", self.ids_for_parts)

        # Wait for cameras to ready
        self.get_logger().info("Waiting for calibrator to setup cameras")

        utils_camera.wait_for_any_connected_cameras(self)

        # Start processing cameras
        self.cameras, self.camera_frames = utils_camera.get_available_cameras(self)
        self.ros = {}

        print("cameras", self.cameras)
        for cam in self.cameras:
            cam = "/" + cam
            camera_topic = f"{cam}/camera/color/image_raw"

            # create input listener with raw images
            # we're using extra callback here to pass additional(topic) arg to the listner.
            # Which then calls a different Publisher for relevant topic.
            def on_image_receive(msg, topic=camera_topic):
                self.input_callback(msg, topic)

            print("camera_topic", camera_topic)
            self.listener = self.create_subscription(
                msg_type=sensor_msgs.msg.Image,
                topic=camera_topic,
                callback=on_image_receive,
                callback_group=rclpy.callback_groups.MutuallyExclusiveCallbackGroup(),
                # the listener QoS has to be =1, "keep last only"
                qos_profile=self.qos,
            )

            self.get_logger().info(f'Input listener created on topic: "{camera_topic}"')
            # camera_topic is used as an ID for this input, all I/O listeners,publishers will be based under that id.
            self.ros[camera_topic] = {}
            
            topic, publisher = publish_aruco(
                self,
                self.qos,
                'tree',
                'scene_in',
                SceneFeedback,
            )

            self.ontology_publisher = self.create_publisher(FilteredPose, "/filtered_poses", 5)


            self.ros[camera_topic] = publisher

            self.get_logger().info(f'Output publisher created for topic: "{topic}"')
            # camera_topic is used as an ID for this input, all I/O listeners,publishers will be based under that id.
        self.msg_backup = {}
        self.object_pub = self.create_publisher(Int16MultiArray, 'objects', 10)
        self.noMessagesYet = True
        self.cvb = CvBridge()
        self.st_objects = []

        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        self.parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.parameters)

        self.get_logger().info("Hi from detector!")

        StatTimer.init()
        
    def get_co(self, id):
        objects = self.config_objects['objects']
        for co in objects.keys():
            markers = objects[co].keys()
            for marker in markers:
                if id == objects[co][marker]['aruco_id']:
                    return co
        return None
        
    def get_offset(self, id):
        objects = self.config_objects['objects']
        for co in objects.keys():
            markers = objects[co].keys()
            for marker in markers:
                if str(id) == str(objects[co][marker]['aruco_id']):
                    m = objects[co][marker]
                    return Pose(position=Point(x=m['offset'][0], y=m['offset'][1], z=m['offset'][2]), orientation=Quaternion(x=m['offset'][3], y=m['offset'][4], z=m['offset'][5], w=m['offset'][6]))
        return None
                
    def pose_mean(self, poses):
        xs,ys,zs = [],[],[]
        for pose in poses:
            xs.append(pose.position.x)
            ys.append(pose.position.y)
            zs.append(pose.position.z)
        
        return Pose(position=Point(x=np.mean(xs),y=np.mean(ys),z=np.mean(zs)), orientation=Quaternion(x=poses[0].orientation.x,y=poses[0].orientation.y,z=poses[0].orientation.z,w=poses[0].orientation.w))
    
    def sub_poses(self, pose1, pose2):
        point = Point(x=pose1.position.x - pose2.position.x, \
        y=pose1.position.y - pose2.position.y,\
        z=pose1.position.z - pose2.position.z)
        return Pose(position=point, orientation=pose1.orientation)

    def add_poses(self, pose1, pose2):
        point = Point(x=pose1.position.x + pose2.position.x, \
        y=pose1.position.y + pose2.position.y,\
        z=pose1.position.z + pose2.position.z)
        return Pose(position=point, orientation=pose1.orientation)
    
    def name_to_size(self, name):
        # check if the name is in compound objects
        objects = self.config_objects['sizes']
        if name in objects.keys():
            return PclDimensions(dimensions=objects[name])
        else:
            return PclDimensions(dimensions=[0.0,0.0,0.0])

    def name_to_uuid(self, name):
        # check if the name is in compound objects
        objects = self.config_objects['uuid']
        if name in objects.keys():
            return objects[name]
        else:
            return "586da263-0d69-xxxx-shel-f51a511c4e66"
        
    
    def input_callback(self, msg, topic):
        """
        @param msg - ROS msg (Image data) to be processed. From camera
        @param topic - str, from camera/input on given topic.
        @return nothing, but send new message(s) via output.json Publishers.
        """
        print("callback")
        if self.noMessagesYet:
            self.get_logger().info("No msgs")
            self.noMessagesYet = False

        if '1' in topic:
            img = self.cvb.imgmsg_to_cv2(msg, "bgr8")
            self.st_objects = []
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            # aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
            # parameters = aruco.DetectorParameters_create()
            corners, ids, rejectedImgPoints = self.detector.detectMarkers(gray)
            # aruco_dict_alt = aruco.Dictionary_get(aruco.DICT_4X4_1000)
            # corners2, ids2, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict_alt, parameters=parameters)
            # corners.extend(corners2)
            # ids.extend(ids2)
            # UNCOMMENT FOR VISUALIZATION
            frame_markers = aruco.drawDetectedMarkers(img.copy(), corners, ids)
            cv2.imshow('Result', frame_markers)
            cv2.waitKey(1)
            ## 
            msg_publish = SceneFeedback()
            objects = []

            compound_object_context = {}
            # print("ids", ids)
            # try:
            if corners is None or ids is None: 
                print("No AruCo markers detected returning")
                return
            for corner, id in zip(corners, ids):
                if id in range(1000):
                    # Marina marker length: 0.029
                    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corner, 0.038, CAMERA_MATRIX, DIST_COEF)
                    marker_pos, marker_orient = transform_marker_position(rvec, tvec)
                    pose = Pose(position=Point(x=marker_pos[0], y=marker_pos[1],z=marker_pos[2]),
                                orientation=Quaternion(x=marker_orient[0], y=marker_orient[1], z=marker_orient[2], w=marker_orient[3]))
                    
                    # check if not compound object part
                    if str(id) in self.ids_for_parts:
                        co_id = self.get_co(str(id))
                        if co_id not in compound_object_context:
                            compound_object_context[co_id] = [] 
                        
                        offset = self.get_offset(id)    
                        assert offset is not None, f"offset id {id} is None"
                        pose_ = self.sub_poses(pose, self.get_offset(id))
                        compound_object_context[co_id].append(pose_)
                    else:
                        # from id "[123]" saves name as "aruco_marker_123"
                        if IGNORE_UNKNOWN_ARUCO_MARKERS:
                            print(f"Ignoring unknown aruco_marker id: {id} with pose: {pose}, define in config_objects.yaml")
                        else:
                            self.st_objects.append(SimpleObject(name=f"aruco_marker_{id[1:-1]}", pose=pose))
                            objects.append(int(id))
                            
            # except TypeError:
            #     self.st_objects = []

            print("compound_object_context", compound_object_context)
            # compound object postprocess
            for key in compound_object_context.keys():
                co_mean_pose = self.pose_mean(compound_object_context[key])
                self.st_objects.append(SimpleObject(name=key, pose=co_mean_pose))
                
            msg_objects = Int16MultiArray()
            msg_objects.data = objects
            self.object_pub.publish(msg_objects)
            msg_publish.objects = self.st_objects
            self.ros[topic].publish(msg_publish)
            self.ontology_publisher.publish(self.scene_feedback_to_filtered_pose(msg_publish))
            print("pub")
            print(self.scene_feedback_to_filtered_pose(msg_publish))

    def scene_feedback_to_filtered_pose(self, scene_feedback):

        poses = []
        sizes = []
        uuids = []
        tracked = []
        labels = []
        
        for obj in scene_feedback.objects:
            labels.append(obj.name)
            poses.append(obj.pose) # pose == pose
            sizes.append(self.name_to_size(obj.name))
            uuids.append(self.name_to_uuid(obj.name))
            tracked.append(True)

        filtered_poses = FilteredPose(poses=poses,size=sizes,tracked=tracked,label=labels,uuid =uuids)
        filtered_poses.header.stamp = self.get_clock().now().to_msg()
        return filtered_poses

def main(args=None):
    if len(sys.argv) and sys.argv[1] == 'static':
        scene_setup_from_scene_properties_file()
        exit()
    rclpy.init(args=args)

    try:
        cnn = CrowVision()
        rclpy.spin(cnn)
        cnn.destroy_node()
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()


''' This is duplicate TEMPORARY '''
def name_to_size(config_objects, name):
    # check if the name is in compound objects
    objects = config_objects['sizes']
    if name in objects.keys():
        return PclDimensions(dimensions=objects[name])
    else:
        return PclDimensions(dimensions=[0.0,0.0,0.0])

def name_to_uuid(config_objects, name):
    # check if the name is in compound objects
    objects = config_objects['uuid']
    if name in objects.keys():
        return objects[name]
    else:
        return "586da263-0d69-xxxx-shel-f51a511c4e66"

def scene_feedback_to_filtered_pose(rosnode, scene_feedback, config_objects):

    poses = []
    sizes = []
    uuids = []
    tracked = []
    labels = []
    
    for obj in scene_feedback.objects:
        labels.append(obj.name)
        poses.append(obj.pose) # pose == pose
        sizes.append(name_to_size(config_objects, obj.name))
        uuids.append(name_to_uuid(config_objects, obj.name))
        tracked.append(True)

    filtered_poses = FilteredPose(poses=poses,size=sizes,tracked=tracked,label=labels,uuid =uuids)
    filtered_poses.header.stamp = rosnode.get_clock().now().to_msg()
    return filtered_poses


def scene_setup_from_scene_properties_file():
    config_objects = load_yaml_config("crow_detector/config_objects.yaml")

    with open(Path("~/crow-base/config/crow_hri/scene_properties.yaml").expanduser()) as f:
        scene_properties = yaml.safe_load(f)


    msg_publish = SceneFeedback()
    st_objects = []

    def gen_static_poses(i, unreachable):
        if unreachable:
            y = 0.7
        else: 
            y = 0.4
        
        X = [-0.2, 0.0, 0.2, -0.1, 0.1, -0.3, 0.3]

        return Pose(position=Point(x=X[i],y=y,z=0.0), orientation=Quaternion(x=0.0,y=0.0,z=0.0,w=1.0))

    for n,name in enumerate(scene_properties.keys()):
        o = scene_properties[name]
        
        st_objects.append(SimpleObject(name=name, pose=gen_static_poses(n, o['reachable'])))

    
    rclpy.init()
    rosnode = Node("adder_tester")
    pub = rosnode.create_publisher(FilteredPose, "/filtered_poses", 5)

    msg_publish.objects = st_objects
    
    msg_filtered_poses = scene_feedback_to_filtered_pose(rosnode, msg_publish, config_objects)
    pub.publish(msg_filtered_poses)

    # refresh objects
    while True:
            msg_filtered_poses.header.stamp = rosnode.get_clock().now().to_msg()
            pub.publish(msg_filtered_poses)
            time.sleep(0.5)

    rclpy.spin(rosnode)


def scene_experimental_setup_node(objects={
    '0': {
        'label': 'wheel',
        'pose': Pose(position=Point(x=0.0,y=0.0,z=0.1),orientation=Quaternion(x=1.0,y=0.0,z=0.0,w=0.0)),
        'uuid': f'{np.random.randint(10000000,99999999)}-0000-000d-0000-wheel0000001', 
        'size': PclDimensions(dimensions=[0.1,0.1,0.1]),
        'tracked': True,
    },
    '1': {
        'label': 'cube_holes',
        'pose': Pose(position=Point(x=0.05,y=0.0,z=0.1),orientation=Quaternion(x=1.0,y=0.0,z=0.0,w=0.0)),
        'uuid': f'{np.random.randint(10000000,99999999)}-0000-0000-0000-cubeholes001', 
        'size': PclDimensions(dimensions=[0.1,0.1,0.1]),
        'tracked': True,            
    },
    '2': {
        'label': 'cube_holes',
        'pose': Pose(position=Point(x=0.1,y=0.0,z=0.1),orientation=Quaternion(x=1.0,y=0.0,z=0.0,w=0.0)),
        'uuid': f'{np.random.randint(10000000,99999999)}-0000-0000-0000-cubeholes002', 
        'size': PclDimensions(dimensions=[0.1,0.1,0.1]),
        'tracked': True,            
    },
    '3': {
        'label': 'drawer_socket',
        'pose': Pose(position=Point(x=0.15,y=0.1,z=0.1),orientation=Quaternion(x=1.0,y=0.0,z=0.0,w=0.0)),
        'uuid': f'{np.random.randint(10000000,99999999)}-0000-0000-0001-drawer000001', 
        'size': PclDimensions(dimensions=[0.01,0.01,0.01]),
        'tracked': True,            
    },
    '4': {
        'label': 'drawer_cabinet',
        'pose': Pose(position=Point(x=0.2,y=0.1,z=0.1),orientation=Quaternion(x=1.0,y=0.0,z=0.0,w=0.0)),
        'uuid': f'{np.random.randint(10000000,99999999)}-0000-0000-0001-drawer000002', 
        'size': PclDimensions(dimensions=[0.01,0.01,0.01]),
        'tracked': True,            
    },
    }):
    """ object_adder_node
    Constantly publishes predefined set of objects to the ontology to fulfill test

    Args:
        objects (dict, optional): _description_. 
    """    
    
    poses = []
    sizes = []
    uuids = []
    tracked = []
    labels = []
    for obj_key in objects.keys():
        obj = objects[obj_key]
        poses.append(obj['pose'])
        sizes.append(obj['size'])
        uuids.append(obj['uuid'])
        tracked.append(obj['tracked'])
        labels.append(obj['label'])

    pubmsg = FilteredPose(poses=poses,size=sizes,tracked=tracked,label=labels,uuid =uuids)
    
    rclpy.init()
    rosnode = Node("adder_tester")
    pub = rosnode.create_publisher(FilteredPose, "/filtered_poses", 5)

    # crate scene objects
    pubmsg.header.stamp = rosnode.get_clock().now().to_msg()
    pub.publish(pubmsg)
    time.sleep(0.5)


    soc = SceneOntologyClient(rosnode)
    # change color to one object
    crowracle = CrowtologyClient(node=rosnode)
    onto = crowracle.onto
    tangible_objects = crowracle.getTangibleObjects()
    print("tangible_objects", tangible_objects)
    colors = ['green', 'red', 'blue', 'blue']
    for n,to in enumerate(tangible_objects):
        crowracle.update_color(tangible_objects[n], colors[n])
   
    print(f"Object with idx: {[(o.name, o.color) for o in soc.get_scene2().objects]}")


    # refresh objects
    while True:
            pubmsg.header.stamp = rosnode.get_clock().now().to_msg()
            pub.publish(pubmsg)
            time.sleep(0.5)

    rclpy.spin(rosnode)

if __name__ == "__main__":
    main()