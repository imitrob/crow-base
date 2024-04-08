#!/usr/bin/env bash

. ./install/setup.bash
ros2 launch crow_cameras cameras.launch.py "force_fake:=True" "$@"
