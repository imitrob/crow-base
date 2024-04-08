import rclpy
import argparse
from imitrob_robot_client.robot_client import RobotActionClient
import asyncio


async def spinning(node):
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)
        await asyncio.sleep(0.001)


async def run(args, loop):
    print("Init")
    racl = RobotActionClient()
    print("Created a robot action client")
    spin_task = loop.create_task(spinning(racl.node))
    print("Started spinning")

    print(f"Chosen action: {args.action}")
    if args.action == 'point':
        point_task = loop.create_task(racl.basic_point(args.point))
    elif args.action == 'pick':
        point_task = loop.create_task(racl.basic_pick(args.point, args.quaternion))
    elif args.action == 'pnp':
        point_task = loop.create_task(racl.basic_pnp(args.point, args.quaternion, args.point2, args.quaternion2))
    else:
        raise ValueError(f"Unknown action: {args.action}!")
    print("Task for action created")

    await asyncio.sleep(1.0)

    wait_future = asyncio.wait([point_task])
    print("Waiting for action...")
    await asyncio.sleep(1.0)

    print(wait_future)
    finished, unfinished = await wait_future
    print(f'unfinished: {len(unfinished)}')
    for task in finished:
        try:
            print('Task {} done\nresult: {}\nstatus flag: {}\ncomment: {}'.format(task.get_coro(), *task.result()))
        except TypeError as e:
            print(task.result())
            pass    # cancel spinning task

    spin_task.cancel()
    try:
        await spin_task
    except asyncio.exceptions.CancelledError:
        pass
    rclpy.shutdown()


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("action", type=str, choices=['point', 'pick', 'pnp'])
    parser.add_argument('--point', "-p", nargs=3, type=float, default=[1, 2, 3])
    parser.add_argument('--quaternion', "-q", nargs=4, type=float, default=[0, 0, 0, 1])
    parser.add_argument('--point2', "--p2", nargs=3, type=float, default=[1, 2, 3])
    parser.add_argument('--quaternion2', "--q2", nargs=4, type=float, default=[0, 0, 0, 1])
    args, _ = parser.parse_known_args(args)

    loop = asyncio.get_event_loop()
    loop.run_until_complete(run(args, loop=loop))


if __name__ == '__main__':
    args = ["point"]
    main(args)
