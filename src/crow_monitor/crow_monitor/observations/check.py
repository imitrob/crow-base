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
import time

from crow_params.client import ParamClient

parser = argparse.ArgumentParser()
parser.add_argument('--verbose', '-v', action='store_true', help="Print more info")
parser.add_argument('--tolerance', '-t', type=int, default=10, help="How many seconds since last tick should we consider the node alive")
parser.add_argument("--node_name", '-n', help="Node name")

def main(args):

    if args.verbose:
        print("Establishing connection...")

    pclient = ParamClient()

    if args.verbose:
        print("Receiving data...")

    last_ping = pclient.wait_receive_param(f"{args.node_name}_alive", timeout = 5)

    if last_ping is None:

        if args.verbose:
            print(f"Failed to find property '{args.node_name}_alive'.")

        last_ping = 0
    else:
        last_ping = float(last_ping)

    if args.verbose:
        print(f"Last ping: {last_ping}, current time: {time.time()}, difference: {time.time() - last_ping}")

    if time.time() - last_ping > 10:

        if args.verbose:
            print("Node NOT OK")

        exit(1)

    if args.verbose:
        print("Node OK")

    exit(0)


def entrypoint():
    args = parser.parse_args([] if "__file__" not in globals() else None)
    main(args)

if __name__ == '__main__':
    entrypoint()
