import pyrealsense2 as realsense
import pyrealsense2 as rs
import re
import crow_utils.crow_config as crow_config
from launch_ros.actions import Node

frames = [
    "accel_frame_id",
    "accel_optical_frame_id",
    "aligned_depth_to_color_frame_id",
    "aligned_depth_to_fisheye1_frame_id",
    "aligned_depth_to_fisheye_frame_id",
    "aligned_depth_to_infra1_frame_id",
    "aligned_depth_to_infra_frame_id",
    "base_frame_id",
    "color_frame_id",
    "color_optical_frame_id",
    "depth_frame_id",
    "depth_optical_frame_id",
    "fisheye1_frame_id",
    "fisheye1_optical_frame_id",
    "fisheye2_frame_id",
    "fisheye2_optical_frame_id",
    "fisheye_frame_id",
    "fisheye_optical_frame_id",
    "gyro_frame_id",
    "gyro_optical_frame_id",
    "imu_optical_frame_id",
    "infra1_frame_id",
    "infra1_optical_frame_id",
    "infra2_frame_id",
    "infra2_optical_frame_id",
    "infra_frame_id",
    "infra_optical_frame_id",
    "odom_frame_id",
    "pose_frame_id",
    "pose_optical_frame_id",
]

frame_regex = re.compile(r"(\w+_frame)_id")


def create_node(cam_id, device, config, globalTFGetter=None):

    camera_namespace = f"camera{cam_id + 1}"
    device_serial = str(device.get_info(realsense.camera_info.serial_number))

    camera_frames_dict = {
        f: f"camera{cam_id + 1}_" + frame_regex.search(f).group(1) for f in frames
    }
    
    camera_frames_dict["base_frame_id"] = f"camera{cam_id + 1}_link"

    if globalTFGetter is not None:
        transform = globalTFGetter.get_camera_transformation(device_serial)
    else:
        transform = None

    config_dict = crow_config.load_yaml_config(config)
    color_fps = config_dict.get("color_fps") or "30.0"

    launch_params = {
        "align_depth": True,
        "enable_infra1": False,
        "enable_infra2": False,
        "serial_no": "_" + device_serial,
        'base_frame_id': '_link',
        'camera_name': camera_namespace,
    }

    launch_params = {**launch_params, **camera_frames_dict, **config_dict}

    camera_node = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace=camera_namespace,
        parameters=[launch_params],
        output="log",
        emulate_tty=True,
    )

    return camera_node, "/" + camera_namespace, transform, color_fps


def get_camera_setup(globalTFGetter, devices):

    nodes = []

    camera_namespaces = []
    camera_serials = []
    camera_transforms = []
    color_fps = []

    for cam_id, device in enumerate(devices):
        cam_name = device.get_info(rs.camera_info.name)
        device_serial = str(device.get_info(rs.camera_info.serial_number))

        if cam_name.lower() == "platform camera":
            continue

        node, ns, transform, cfps = create_node(
            cam_id, device, "crow_cameras/rs_native435.yaml", globalTFGetter
        )

        nodes.append(node)
        camera_namespaces.append(ns)
        camera_serials.append(device_serial)
        camera_transforms.append(transform)
        color_fps.append(cfps)

    calibrator_node = Node(
        package="crow_cameras",
        executable="calibrator",
        output="screen",
        emulate_tty=True,
        parameters=[{"halt_calibration": True}],
        arguments=[
            "--camera_namespaces",
            " ".join(camera_namespaces),
            "--camera_serials",
            " ".join(camera_serials),
            "--color_fps",
            " ".join([str(f) for f in color_fps]),
            "--camera_transforms",
            " | ".join(camera_transforms),
        ],
    )

    return nodes + [calibrator_node]
