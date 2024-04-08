import argparse
import os

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


class TopicViewer(Node):

    def __init__(self, args):
        super().__init__('topic_viewer')
        self.fps_counters = {}
        self.create_timer(1, self.loop)
        self.types = {}
        self.total_counter = {}

    def loop(self):
        self.discovery()
        self.render()

    def discovery(self):

        for topic, types in self.get_topic_names_and_types():

            self.types[topic] = types[0]

            if topic not in self.total_counter:
                self.total_counter[topic] = 0

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

    def render(self):

        stdscr = curses.initscr()
        curses.noecho()
        curses.cbreak()
        stdscr.keypad(True)

        padding = 3
        max_topic_len = max(list(map(lambda s: len(s), self.fps_counters.keys()))) + padding
        max_fps = max(list(map(lambda s: self.fps_counters[s], self.fps_counters.keys()))) or 1
        speed_bar_len = 30

        raw_table = []

        tree = {"__name": ""}

        topics = self.fps_counters.keys()

        for topic in topics:
            subtree = tree
            subtree_name = ""
            for part in topic.split("/"):
                subtree_name += part + "/"
                if part not in subtree:
                    subtree[part] = {}
                    subtree[part]["__name"] = subtree_name if len(subtree_name) < 3 else subtree_name[:-1]
                subtree = subtree[part]

        def populate_table(table, tree, indent=''):

            if "__name" in tree:
                topic = tree["__name"]
                tabbed = indent + topic
                if topic in topics:
                    raw_table.append([
                        tabbed,
                        self.fps_counters[topic],
                        self.total_counter[topic],
                        "#" * int(((speed_bar_len / max_fps) * self.fps_counters[topic])),
                        f"Could not find type {self.types[topic]}" if self.fps_counters[topic] == -1 else ''
                    ])
                else:
                    raw_table.append([tabbed, "", "", "", ""])

            for child_name in tree:
                if child_name == "__name":
                    continue
                populate_table(table, tree[child_name], indent + ' ')

        populate_table(raw_table, tree)

        table_printer(
            stdscr,
            location=(0, 0),
            header=[
                "Topic",
                "FPS",
                "Count",
                "Visual",
                f"Max fps: {max_fps}"
            ],
            table=raw_table,
            column_sizes=[
                max_topic_len + 20,
                5,
                7,
                speed_bar_len + 2,
                10,
            ]
        )

        stdscr.refresh()

        for topic in self.fps_counters:
            if self.fps_counters[topic] != -1:
                self.fps_counters[topic] = 0

    def callback(self, msg, topic):
        self.fps_counters[topic] += 1
        self.total_counter[topic] += 1


def main(args=None):
    rclpy.init()

    args = parser.parse_args(args)
    minimal_subscriber = TopicViewer(args)
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main(parser.parse_args(None))
