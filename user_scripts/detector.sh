#!/usr/bin/env bash

. ./install/setup.bash
ros2 run crow_detector detector "$@"
