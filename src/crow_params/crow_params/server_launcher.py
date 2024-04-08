
import os.path as osp
from crow_utils.crow_config import load_yaml_config
from crow_params.server import ParamServer

def main(args=None):

    config = load_yaml_config(osp.join("crow_param_server", "crow_param_server.yaml"))
    print(f"Starting server with config: {config}")

    # config["bind_ip"] = '127.0.0.1'
    config["bind_ip"] = '*'


    try:
        # Start param server
        pserver = ParamServer(start_port=config["bind_port"], addr=config["bind_ip"])

        # Hang while param server runs
        pserver.poller_thread.join()
    except InterruptedError:
        pserver.destroy()


if __name__ == "__main__":
    main()