# crow-base

Crow-base is the basic crow setup with core modules and ontology setup. Installable under conda environment for ROS2 Humble and Linux 20.04.

## Configuration

Configuration files are stored in `config` folder.

If you wish to use a different folder, use the environment variable `CROW_CONFIG_DIR`

## Nodes

### CROW Cameras

See [Cameras README.md](src/crow_cameras/README.md) for more details

Status: Functional

### CROW Detector

See [Detector README.md](src/crow_detector/README.md) for more details

Status: Functional

### Crow Onto Server

Status: Finished

### Crow Onto Adder

Status: Buggy

### Crow Filter

# Usage:

1. Run ontology, object detector, cameras, adder, visualizer
`alias crow="conda deactivate; conda activate crow_env;source install/setup.bash; export ROS_DOMAIN_ID=<YOUR_ID>;export CROW_CONFIG_DIR=~/crow-base/config; export CROW_LIB_DIR=~/crow-base/lib; export CROW_SETUP_FILE=setup_404.yaml;"`
`alias crowrun="crow; cd ~/crow-base/user_scripts; python tmux_all.py --config run_tmux_404.yaml; tmux attach-session -t crow_run_all"`

2. Try Modality Merger demo: `https://github.com/imitrob/imitrob_hri`


