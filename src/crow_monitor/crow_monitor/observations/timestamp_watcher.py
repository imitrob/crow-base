import argparse
import itertools
import os
import time

import rclpy
import realsense2_camera_msgs.msg
from rclpy.node import Node

from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import sensor_msgs.msg
import curses

if "MWX_DEBUG" in os.environ:
    import pydevd_pycharm

    pydevd_pycharm.settrace('localhost', port=25565, stdoutToServer=True, stderrToServer=True)

parser = argparse.ArgumentParser()


def find_msg_types():
    import sensor_msgs.msg
    import crow_msgs.msg
    import realsense2_camera_msgs

    types = {}

    def include_package(pkg, name):
        for tp in dir(pkg):
            if tp.startswith("_") or tp.startswith("."):
                continue
            types[name + "." + tp] = getattr(pkg, tp)

    include_package(sensor_msgs.msg, "sensor_msgs.msg")
    include_package(crow_msgs.msg, "crow_msgs.msg")
    include_package(realsense2_camera_msgs.msg, "realsense2_camera_msgs.msg")

    return types


def match_count(str1, str2):
    i = 0
    for s1, s2 in zip(str1, str2):
        if s1 == s2:
            i += 1
        else:
            break

    return i


def table_printer(stdscr, location, header, table, column_sizes):
    x = location[0]
    y = location[1]

    stdscr.clear()

    pos = 0
    for head, column_size in zip(header, column_sizes):
        stdscr.addstr(y + 0, x + pos, head)
        pos += column_size

    for line, i in zip(table, range(len(table))):
        pos = 0
        for cell, column_size in zip(line, column_sizes):
            stdscr.addstr(y + i + 1, x + pos, str(cell))
            pos += column_size


class TimeStampViewer(Node):

    def __init__(self, args):
        super().__init__('timestamp_viewer')

        self.camera_timestamps = set()
        self.other_timestamps = set()
        self.log = []

        self.fps_counters = {}
        self.types = {}
        self.total_counter = {}
        self.create_timer(1, self.loop)

        self.origins = {}
        self.difference = {}

    def loop(self):
        self.discovery()
        self.render()

    def discovery(self):

        for topic, types in self.get_topic_names_and_types():

            self.types[topic] = types[0]

            if topic in self.fps_counters:
                continue

            topic_type_str = types[0].replace("/", ".")

            types = find_msg_types()

            if topic_type_str not in types:
                self.fps_counters[topic] = -1
                continue

            self.fps_counters[topic] = 0

            qos = 1

            if "detections" in topic or "nl_input" in topic:
                qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)

            self.create_subscription(
                msg_type=types[topic_type_str],
                topic=topic,
                callback=lambda msg, topic=topic: self.callback(msg, topic),
                qos_profile=qos
            )

            self.log += [f"subscribed to {topic}"]

    def render(self):

        stdscr = curses.initscr()
        curses.noecho()
        curses.cbreak()
        stdscr.keypad(True)

        table = []
        SIZE = 40

        camera_timestamps = sorted(self.camera_timestamps)[-SIZE:]
        other_timestamps = sorted(self.other_timestamps)[-SIZE:]
        logs = self.log[-SIZE:]

        suspicious_timestamps = self.other_timestamps - self.camera_timestamps
        suspicious_timestamps = sorted(suspicious_timestamps)[-SIZE:]

        differences = list(self.difference.values())
        differences = differences[-SIZE:]

        for sus in suspicious_timestamps:
            msg = f"Found a weird timestamp: {sus} from {self.origins[sus]}"

            if msg not in self.log:
                self.log.append(msg)

        for timestamp_camera, timestamp_other, sus, dif, log in itertools.zip_longest(camera_timestamps,
                                                                                      other_timestamps,
                                                                                      suspicious_timestamps,
                                                                                      differences, logs):
            timestamp_camera = timestamp_camera or ''
            timestamp_other = timestamp_other or ''
            dif = str(dif)[:10] if dif else ''
            sus = sus or ''
            log = log or ''

            table.append([timestamp_camera, timestamp_other, sus, dif, log])

        table_printer(
            stdscr,
            location=(0, 0),
            header=[
                "Camera Timestamps",
                "Pipeline Timestamps",
                "Suspicious Timestamps",
                "Latency",
                "Logs"
            ],
            table=table,
            column_sizes=[
                30,
                30,
                30,
                15,
                20
            ]
        )

        stdscr.refresh()

    def callback(self, msg, topic):
        topic = str(topic)

        if topic.startswith("/camera1/color/image_raw") or topic.startswith("/camera1/depth"):
            category = self.camera_timestamps
        else:
            category = self.other_timestamps

        ts = str(msg.header.stamp.sec) + "." + str(msg.header.stamp.sec)
        category.add(ts)

        if ts not in self.origins:
            self.origins[ts] = set()

        self.origins[ts].add(topic)

        if ts not in self.difference:
            self.difference[ts] = 0

        if self.difference[ts] < time.time() - float(ts):
            self.difference[ts] = time.time() - float(ts)

        msg = f"Received message from {topic}"

        if msg not in self.log:
            self.log.append(msg)


def main(args=None):
    rclpy.init()

    args = parser.parse_args(args)
    node = TimeStampViewer(args)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main(parser.parse_args(None))
