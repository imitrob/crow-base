from launch_ros.actions import Node

from crow_cameras.utils.get_camera_transformation import CameraGlobalTFGetter


def create_node(
    name, fake_video_path, calib_path, image_topic, camera_info_topic
):

    fake_camera_node = Node(
        name=name,
        namespace="camera1",
        package="camera_simulator",
        executable="camera_simulator",
        output="log",
        arguments=[
            "--type",
            "video",
            "--path",
            fake_video_path,
            "--calibration_file",
            calib_path,
        ],
        parameters=[{"image_topic": image_topic, "camera_info_topic": camera_info_topic}],
        emulate_tty=True,
    )

    return fake_camera_node


def get_camera_setup(fake_rgb_path, fake_depth_path, fake_camera_calibration_file, globalTFGetter=None):

    nodes = [
        create_node(
            "camera1_color_node",
            fake_rgb_path,
            fake_camera_calibration_file,
            image_topic="/camera1/color/image_raw",
            camera_info_topic="/camera1/color/camera_info",
        ),
        create_node(
            "camera1_depth_node1",
            fake_depth_path,
            fake_camera_calibration_file,
            image_topic="/camera1/depth/image_raw",
            camera_info_topic="/camera1/depth/camera_info",
        ),
    ]

    # calibrator_node = Node(
    #     package="crow_cameras",
    #     executable="calibrator",
    #     output="screen",
    #     emulate_tty=True,
    #     parameters=[{"halt_calibration": True}],
    #     arguments=[
    #         "--camera_namespaces", "camera1",
    #         "--camera_serials", "fak3s3r1al",
    #         "--color_fps", "15.0",
    #         "--camera_transforms", f"{CameraGlobalTFGetter.UNKNOWN_CAMERA_TRANSFORM}"
    #     ],
    # )

    return nodes #+ [calibrator_node]

