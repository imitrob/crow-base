#!/usr/bin/env bash

set -eux

if [[ $# != 3 || "$1" == "-h" || "$1" == "--help" ]]; then
  echo "Usage:"
  echo $'\t'"$0 <rgb_folder> <depth_folder> <output_folder>"
  echo ""
  echo "This script expects a series of jpg images in both <rgb_folder> and <depth_folder>"
  exit 1
fi

if ! hash ffmpeg &> /dev/null; then
  echo "Must have ffmpeg installed"
  exit 1
fi

pwd

(
  cd $1 || { echo "Failed to enter $1"; exit 1; }
  ffmpeg -framerate 15 -pattern_type glob -i '*.jpg' -c:v libx264 -pix_fmt yuv420p out.mp4
  mv out.mp4 "$3/rgb.mp4"
)

(
  cd $2 || { echo "Failed to enter $1"; exit 1; }
  ffmpeg -framerate 15 -pattern_type glob -i '*.jpg' -c:v libx264 -pix_fmt yuv420p out.mp4
  mv out.mp4 "$3/depth.mp4"
)