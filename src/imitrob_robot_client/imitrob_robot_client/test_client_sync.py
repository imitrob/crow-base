import rclpy
import argparse
from imitrob_robot_client.robot_client import RobotActionClient
from time import sleep
import pickle


def choose_and_run_action(racl: RobotActionClient, **kwargs):
    print(f"Chosen action: {kwargs['action']}")
    if kwargs['action'] == 'point':
        point_task = racl.basic_point(kwargs['point'])
    elif kwargs['action'] == 'pick':
        point_task = racl.basic_pick(kwargs['point'], kwargs['quaternion'])
    elif kwargs['action'] == 'pnp':
        point_task = racl.basic_pnp(kwargs['point'], kwargs['quaternion'], kwargs['point2'], kwargs['quaternion2'])
    elif kwargs['action'] == 'traj':
        with open(kwargs['trajectory_file'], 'rb') as f:
            trajectory = pickle.load(f)

        jtraj = []
        for p in trajectory:
            jtraj.append(p[:7])
        point_task = racl.trajectory_joint(jtraj)
    else:
        raise ValueError(f"Unknown action: {kwargs['action']}!")
    print(f"Task for action created: {point_task}")


def run(args):
    rclpy.init()
    node = rclpy.create_node("test1")
    # node = None
    print("Init")
    racl = RobotActionClient(node)
    print("Created a robot action client")
    racl.start()

    choose_and_run_action(racl, **args.__dict__)
    print("now")

    # sleep(20)
    # print("then")
    # choose_and_run_action(racl, **args.__dict__)
    # print("end")
    rclpy.spin(node)

    rclpy.shutdown()


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("action", type=str, choices=['point', 'pick', 'pnp', 'traj'])
    parser.add_argument('--point', "-p", nargs=3, type=float, default=[1, 2, 3])
    parser.add_argument('--quaternion', "-q", nargs=4, type=float, default=[0, 0, 0, 1])
    parser.add_argument('--point2', "--p2", nargs=3, type=float, default=[1, 2, 3])
    parser.add_argument('--quaternion2', "--q2", nargs=4, type=float, default=[0, 0, 0, 1])
    parser.add_argument('--trajectory-file', "--traj-file", type=str, default="joint_poses.pckl")
    args, _ = parser.parse_known_args(args)

    run(args)


if __name__ == '__main__':
    main()
