# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import argparse
import sys

import rclpy
import sensor_msgs.msg
from rclpy.node import Node
import time
import cv2
import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument("--input", default="/camera1/color/image_raw", type=str, help="Input of the image.")
parser.add_argument("--restackrgb", action='store_true', default=False, help="Whether to convert RBG -> BGR for Opencv.")


class ImageSubscriber(Node):

    def __init__(self, args):
        super().__init__('image_subscriber')

        self.input_topic = args.input
        self.restackrgb = args.restackrgb

        self.create_subscription(msg_type=sensor_msgs.msg.Image, topic=self.input_topic,
                                 callback=lambda msg: self.listener_callback(msg), qos_profile=1)

        self.fps_time = -1
        self.fps_counter = 0
        self.fps = 0

        self.adjuster_min = float("inf")
        self.adjuster_max = float("-inf")

        print("Node started, waiting for input...")

    def listener_callback(self, msg):

        print(".", end='')

        data = np.asanyarray(msg.data)
        data = np.reshape(data, (msg.height, msg.width, -1))

        if self.fps_time == -1 or self.fps_time != int(time.time()):
            self.fps_time = int(time.time())
            self.fps = self.fps_counter
            self.fps_counter = 0
            print(f"FPS: {self.fps}")
            print(f"Min: {self.adjuster_min}, max: {self.adjuster_max}")
            print(f"Shape of input: {data.shape}")
        else:
            self.fps_counter += 1

        new_min = np.min(data)
        new_max = np.max(data)

        self.adjuster_min = min(self.adjuster_min, new_min)
        self.adjuster_max = max(self.adjuster_max, new_max)

        if self.restackrgb:
            r = data[:, :, 0]
            g = data[:, :, 1]
            b = data[:, :, 2]
            data = np.stack(arrays=(b, g, r), axis=2)

        cv2.namedWindow(self.input_topic, cv2.WINDOW_AUTOSIZE)
        cv2.imshow(self.input_topic, data)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init()

    args = parser.parse_args(args)
    minimal_subscriber = ImageSubscriber(args)
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main(parser.parse_args(None))
