#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import libtmux
import argparse
import sys
import yaml
import os
from warnings import warn
import time


CROW_SESSION = "crow_run_all"
CONFIG = "run_tmux_config.yaml"


def find_session(sessions, name):
    try:
        session = next(filter(lambda x: name in x.name, sessions))
    except (StopIteration, libtmux.exc.LibTmuxException):
        return None
    return session


def execute(pane, command):
    pane.send_keys(command)


def run_windows(session, config):
    window = config['window']
    is_first_window = True
    for window_name, args in window.items():
        if is_first_window:
            window = session.windows[0]
            window.rename_window(window_name)
            is_first_window = False
        else:
            window = session.new_window(window_name)

        pane = window.panes[0]

        for arg in args:
            if arg.startswith("-"):
                if "h" in arg:
                    warn("-h argument does not work, only vertical split is supported.")
                    pane = pane.split_window(vertical="h" in arg)
                elif "v" in arg:
                    pane = pane.split_window(vertical="v" in arg)
                elif "sleep" in arg:
                    duration = float(arg.split("=")[-1])
                    time.sleep(duration)
                elif "print" in arg:
                    print(arg.split("=")[-1])
                else:
                    raise ValueError(f"Unknown argument value: {arg}")
            else:
                execute(pane, arg)

        window.select_layout('even-vertical')

    return True


def main(args):
    server = libtmux.Server()
    if args.close:
        session = find_session(server.sessions, CROW_SESSION)
        if session is None:
            raise RuntimeError(f"Could not find running session {CROW_SESSION}!")
        session.kill_session()
        print("Session closed successfully.")
        sys.exit(0)

    server.new_session(CROW_SESSION)
    session = find_session(server.sessions, CROW_SESSION)

    if not os.path.exists(args.config):
        warn(f"Config file {args.config} does not exist, trying to search it locally next to the executed script.")
        local_config_file = os.path.join(os.path.dirname(__file__), args.config)
        if os.path.exists(local_config_file):
            print(f"Config file {local_config_file} does exist, using it.")
            args.config = local_config_file
        else:
            raise ValueError(f"Config file {args.config} does not exist!")

    with open(args.config, 'r') as config_file:
        config = yaml.safe_load(config_file)

    success = run_windows(session, config)
    if success:
        print(f"Startup of tmux appears to be successful.\nConnect to it via 'tmux attach-session -t {CROW_SESSION}'.\nKill it by running this script with '-k' switch.")


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", "-c", type=str, default=CONFIG, help="The startup configuration YAML file.")
    parser.add_argument('--close', "-k", action='store_true', help='Close the session and quit. Without this, the session will start.')
    args = parser.parse_args()

    main(args)
