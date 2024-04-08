import time

import sensor_msgs
from ros2param.api import call_get_parameters


def wait_for_any_connected_cameras(node):
    cameras, camera_frames = get_available_cameras(node)

    while len(cameras) == 0:
        node.get_logger().warn("Waiting for any cameras!")
        time.sleep(2)
        cameras, camera_frames = get_available_cameras(node)


def get_available_cameras(node):

    topics = node.get_topic_names_and_types()
    cameras = set()

    for topic, datatype in topics:

        if not topic.startswith("/camera"):
            continue

        cam = topic.split("/")[1]
        cameras.add(cam)

    return list(cameras), None


def get_available_cameras_from_calibrator(node):
    cameras, camera_frames = [
        p.string_array_value
        for p in call_get_parameters(
            node=node,
            node_name="/calibrator",
            parameter_names=["camera_namespaces", "camera_frames"],
        ).values
    ]
    return cameras, camera_frames


def get_available_camera_image_topics(node):
    cameras, image_topics = [
        p.string_array_value
        for p in call_get_parameters(
            node=node,
            node_name="/calibrator",
            parameter_names=["camera_namespaces", "image_topics"],
        ).values
    ]
    return cameras, image_topics


def publish(cam, node, qos, topic_prefix, topic, msg_type):
    topic = cam + "/" + topic_prefix + "/" + topic
    publisher = node.create_publisher(msg_type, topic, qos_profile=qos)
    return topic, publisher

def publish_aruco(node, qos, topic_prefix, topic, msg_type):
    topic = "/" + topic_prefix + "/" + topic
    publisher = node.create_publisher(msg_type, topic, qos_profile=qos)
    return topic, publisher
