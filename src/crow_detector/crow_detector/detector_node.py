import copy

import cv2
import numpy as np
import rclpy
import sensor_msgs
import torch
from crow_monitor.profiler.profiling import StatTimer
from crow_msgs.msg import BBox, DetectionBBox, DetectionMask
from crow_utils.crow_config import load_yaml_config
from cv_bridge import CvBridge
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


import crow_detector.utils.camera as utils_camera
from crow_detector.implementations.yolact_detector import YolactDetector


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
        # reliability=QoSReliabilityPolicy.BEST_EFFORT,
        reliability=QoSReliabilityPolicy.RELIABLE,
    )

    def __init__(self):
        super().__init__("crow_detector")

        # Attempt to load config
        self.config = load_yaml_config("crow_detector/config_affordance.yaml")

        # Wait for cameras to ready
        self.get_logger().info("Waiting for calibrator to setup cameras")
        calib_client = self.create_client(GetParameters, "/calibrator/get_parameters")
        calib_client.wait_for_service()
        utils_camera.wait_for_any_connected_cameras(self)

        # Start processing cameras
        self.cameras, image_topics = utils_camera.get_available_camera_image_topics(self)
        self.ros = {}

        # for cam in self.cameras:
        #     cam = "/" + cam
        #     camera_topic = f"{cam}/camera/color/image_raw"

        for cam, camera_topic in zip(self.cameras, image_topics):
            # create input listener with raw images
            # we're using extra callback here to pass additional(topic) arg to the listner.
            # Which then calls a different Publisher for relevant topic.
            def on_image_receive(msg, topic=camera_topic):
                self.input_callback(msg, topic)

            self.listener = self.create_subscription(
                msg_type=sensor_msgs.msg.Image,
                topic=camera_topic,
                callback=on_image_receive,
                # callback_group=rclpy.callback_groups.MutuallyExclusiveCallbackGroup(),
                # the listener QoS has to be =1, "keep last only"
                qos_profile=QoSProfile(
                    depth=1,
                    reliability=QoSReliabilityPolicy.BEST_EFFORT,
                ),
            )

            self.get_logger().info(f'Input listener created on topic: "{camera_topic}"')

            # camera_topic is used as an ID for this input, all I/O listeners,publishers will be based under that id.
            self.ros[camera_topic] = {}

            # there are multiple publishers (for each input/camera topic).
            # the results are separated into
            # different (optional) subtopics the clients can subscribe to (eg 'labels', 'masks')
            # If an output topic is empty (""), we skip publishing on that stream, it is disabled.
            # Use to save computation resources.
            if "image_annotated" in self.config["outputs"] and self.config["outputs"]["image_annotated"]:
                topic, publisher = utils_camera.publish(
                    cam,
                    self,
                    self.qos,
                    self.config["outputs"]["prefix"],
                    self.config["outputs"]["image_annotated"],
                    sensor_msgs.msg.Image,
                )

                self.ros[camera_topic]["pub_img"] = publisher

                self.get_logger().info(f'Output publisher created for topic: "{topic}"')

            if "masks" in self.config["outputs"] and self.config["outputs"]["masks"]:
                # publishes the processed (annotated,detected) masks
                topic, publisher = utils_camera.publish(
                    cam,
                    self,
                    self.qos,
                    self.config["outputs"]["prefix"],
                    self.config["outputs"]["masks"],
                    DetectionMask,
                )

                self.ros[camera_topic]["pub_masks"] = publisher

                self.get_logger().info(f'Output publisher created for topic: "{topic}"')

            if "bboxes" in self.config["outputs"] and self.config["outputs"]["bboxes"]:
                # publishes the processed (annotated,detected) bboxes
                topic, publisher = utils_camera.publish(
                    cam,
                    self,
                    self.qos,
                    self.config["outputs"]["prefix"],
                    self.config["outputs"]["bboxes"],
                    DetectionBBox
                )

                self.ros[camera_topic]["pub_bboxes"] = publisher

                self.get_logger().info(f'Output publisher created for topic: "{topic}"')

        self.noMessagesYet = True
        self.cvb = CvBridge()

        self.detector = YolactDetector(
            self.config,
            self.config["config_file"],
            self.config["weights_file"],
            lambda s: self.get_logger().info(s)
        )

        self.get_logger().info("Hi from detector!")

        StatTimer.init()

    def input_callback(self, msg, topic):
        """
        @param msg - ROS msg (Image data) to be processed. From camera
        @param topic - str, from camera/input on given topic.
        @return nothing, but send new message(s) via output Publishers.
        """

        if self.noMessagesYet:
            self.get_logger().info("Hello from callback!")
            self.noMessagesYet = False

        img = self.cvb.imgmsg_to_cv2(msg, "bgr8")
        num_objects, processed = self.detector.infer(img)

        if num_objects == 0:
            self.get_logger().info("Nothing detected")
            return

        if "pub_img" in self.ros[topic]:

            img_labeled = self.detector.get_labeled(processed)

            msg_labeled = self.cvb.cv2_to_imgmsg(img_labeled, encoding="rgb8")

            self.ros[topic]["pub_img"].publish(msg_labeled)

        if "pub_masks" in self.ros[topic]:

            masks, object_ids, classes, class_names, scores = self.detector.get_masks(processed)

            msg_masks = DetectionMask()
            m_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))

            msg_masks.masks = [
                self.cvb.cv2_to_imgmsg(
                    cv2.morphologyEx(mask, cv2.MORPH_OPEN, m_kernel),
                    encoding="mono8",
                )
                for mask in masks.astype(np.uint8)
            ]

            msg_masks.header.stamp = msg.header.stamp

            for mask in msg_masks.masks:
                mask.header.stamp = msg.header.stamp

            msg_masks.header.frame_id = msg.header.frame_id
            msg_masks.object_ids = object_ids
            msg_masks.classes = classes
            msg_masks.class_names = class_names
            msg_masks.scores = scores

            self.ros[topic]["pub_masks"].publish(msg_masks)

        if "pub_bboxes" in self.ros[topic]:
            bboxes, classes, class_names, scores = self.detector.get_bboxes(processed)

            msg_bbox = DetectionBBox()
            msg_bbox.bboxes = [BBox(bbox=bbox) for bbox in bboxes]

            msg_bbox.header.stamp = msg.header.stamp
            msg_bbox.header.frame_id = msg.header.frame_id
            msg_bbox.classes = classes
            msg_bbox.class_names = class_names
            msg_bbox.scores = scores

            self.ros[topic]["pub_bboxes"].publish(msg_bbox)


def main(args=None):
    print(f"Running PyTorch:")
    print(f"\tver: {torch.__version__}")
    print(f"\tfile: {torch.__file__}")

    rclpy.init(args=args)

    try:
        cnn = CrowVision()
        rclpy.spin(cnn)
        cnn.destroy_node()
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
