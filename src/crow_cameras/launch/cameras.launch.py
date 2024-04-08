from pprint import pprint

import pyrealsense2 as rs
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.actions import LogInfo

from crow_cameras.camera_node import get_camera_setup
from crow_cameras.fake_camera_node import get_camera_setup as fake_get_camera_setup

from crow_cameras.utils.get_camera_transformation import CameraGlobalTFGetter

import crow_utils.crow_config as crow_config

import os.path as osp


def launch_cameras(launchContext):
    devices = list(rs.context().query_devices())

    if len(devices) == 0:
        raise Exception("Could not find a real device")

    mode = "real"

    msg = LogInfo(
        msg=f"Running in mode {mode}, available cameras found are: {len(devices)}"
    )

    nodes = None

    cgtfg = CameraGlobalTFGetter()
    nodes = get_camera_setup(cgtfg, devices)

    return nodes + [msg]


def generate_launch_description():
    return LaunchDescription(
        [
            OpaqueFunction(function=launch_cameras),
        ]
    )
