import numpy as np
import asyncio
import concurrent.futures
import time
import threading

from action_msgs.msg import GoalStatus
from imitrob_common_interfaces.action import BasicRobotAction
from imitrob_common_interfaces.msg import RobotActionType, ResourceHandling, TargetObject, OperationMode, OperationModeChangeReFlag
from imitrob_common_interfaces.srv import ReserveResource, GetOperationMode, SetOperationMode, ListOperationModes

from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion
from uuid import uuid4

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from typing import Optional, Tuple, Iterable
from collections import deque
import traceback as tb


SERVICE_LIST = {
    "resource": 'reserve_robot_resource',
    "get_mode": 'get_operation_mode',
    "set_mode": 'set_operation_mode',
    "list_mode": 'list_operation_modes',
}
STANDARD_FRAME = "robot_base"
DEFAULT_ROBOT = 1


class RobotServiceCaller():

    def __init__(self, node, robot_ns):
        self.__node = node
        self._resource_srv = self.__node.create_client(ReserveResource, f'{robot_ns}{SERVICE_LIST["resource"]}')
        self._get_mode_srv = self.__node.create_client(GetOperationMode, f'{robot_ns}{SERVICE_LIST["get_mode"]}')
        self._set_mode_srv = self.__node.create_client(SetOperationMode, f'{robot_ns}{SERVICE_LIST["set_mode"]}')
        self._list_mode_srv = self.__node.create_client(ListOperationModes, f'{robot_ns}{SERVICE_LIST["list_mode"]}')

    async def __call_service(self, service, request):
        # future = await service.call_async(request)
        future = await asyncio.to_thread(service.call_async, request)
        rclpy.spin_until_future_complete(self.__node, future)
        res = future.result()
        return res

    def get_operation_mode(self):
        return self.__call_service(self._get_mode_srv, GetOperationMode.Request())

    def set_operation_mode(self, operation_mode):
        request = SetOperationMode.Request(operation_mode=OperationMode(mode=operation_mode))
        return self.__call_service(self._set_mode_srv, request)

    def list_operation_modes(self):
        return self.__call_service(self._list_mode_srv, ListOperationModes.Request())

    async def reserve_robot(self, comment="pick_at_pose", rid=DEFAULT_ROBOT):
        reqest = ReserveResource.Request(
            resource=rid,
            requester=self.__node.get_name(),
            action=ResourceHandling.RESERVE,
            comment=comment,
        )
        print("rrrrrrrrrrrrr")
        return await self.__call_service(self._resource_srv, reqest)

    def release_robot(self, key, rid=DEFAULT_ROBOT):
        reqest = ReserveResource.Request(
            resource=rid,
            requester=self.__node.get_name(),
            action=ResourceHandling.RELEASE,
            key=key,
        )
        return self.__call_service(self._resource_srv, reqest)


class ModusOperandi():

    def __init__(self, node: Node, robot_caller: RobotServiceCaller):
        self._robot_caller = robot_caller
        self._node = node

        #
        self._action_client = None
        self._action_class = None
        self._required_operation_mode = None

        self.__robot_key = None

    @property
    def action_class(self):
        return self._action_class

    async def _perform_action(self, action, *args, **kwargs) -> Tuple[bool, int, str]:
        print("1 Llllllllllllllllllllllll")
        if self.__robot_key is None:
            print("2 Llllllllllllllllllllllll")
            response = await self._robot_caller.reserve_robot()
            print("3 Llllllllllllllllllllllll")
            if not response.success:
                return False, -1, "Failed to reserve robot"
            self.__robot_key = response.key

        if self._robot_caller.get_operation_mode().operation_mode.mode != self._required_operation_mode:
            response = self._robot_caller.set_operation_mode(self._required_operation_mode)
            if response.flag & OperationModeChangeReFlag.NOK:
                success, flag, message = False, response.flag, f"Couldn't set the required operation mode.\n{response.msg}"

        try:
            goal = action(*args, **kwargs)
        except BaseException as e:
            success, flag, message = f"Failed to formulate action goal.\n{e}"

        try:
            success, flag, message = await self.__handle_action(goal)
        except BaseException as e:
            success, flag, message = False, -1, f"Action execution failure.\n{e}"

        self._robot_caller.release_robot(self.__robot_key)
        return success, flag, message

    def _feedback_callback(self, feedback):
        self._node.get_logger().info('Received feedback: {0}'.format(feedback.feedback))

    async def __handle_action(self, goal):
        self._node.get_logger().debug("Waiting for server")
        self._action_client.wait_for_server()
        self._node.get_logger().debug('Sending goal request...')

        try:
            goal_handle = await self._action_client.send_goal_async(
                goal,
                feedback_callback=self._feedback_callback
            )
        except BaseException as e:
            return False, -1, f"Failed to send goal request because:\n{e}"

        if not goal_handle.accepted:
            self._node.get_logger().warn('Goal rejected :(')
            return False, -1, "Goal rejected"

        self._node.get_logger().debug('Goal accepted :)')

        try:
            res = await goal_handle.get_result_async()
        except:
            return False, -1, "Action execution failed"

        result = res.result
        status = res.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._node.get_logger().debug('Goal succeeded! Result: {0}'.format(result))
        else:
            self._node.get_logger().warn('Goal failed with status: {0}'.format(status))
        return status == GoalStatus.STATUS_SUCCEEDED, result.action_result_flag.flag, "Action done"

    def fill_header(self):
        header = Header()
        header.frame_id = STANDARD_FRAME
        header.stamp = self._node.get_clock().now().to_msg()
        return header

    @property
    def _robot_key(self) -> str:
        return self.__robot_key


class BasicSkills(ModusOperandi):

    def __init__(self, node: Node, robot_caller: RobotServiceCaller, robot_ns: str):
        super().__init__(node, robot_caller)
        self._action_class = BasicRobotAction
        self._required_operation_mode = OperationMode.BASIC_SKILL_MODE
        self._action_client = ActionClient(node, BasicRobotAction, f'/{robot_ns}basic_robot_action')

    async def point(self, position):
        print("wwwwwwwwwwwwwwwwwwwwww")
        try:
            res = await self._perform_action(self._point_goal, position)
        except Exception as e:
            tb.print_exc()
            return e
        return res

    def _point_goal(self, position):
        goal = BasicRobotAction.Goal()
        goal.header = self.fill_header()
        goal.key = self._robot_key
        goal.robot_action_type = RobotActionType(type=RobotActionType.POINT)
        goal.robot_id = DEFAULT_ROBOT
        goal.targets = [
            TargetObject(
                eef_pose=Pose(
                    position=Point(x=float(position[0]), y=float(position[1]), z=float(position[2])),
                )
            )
        ]
        return goal

    async def pick(self, position, orientation):
        return await self._perform_action(self._pick_goal, pos=position, quat=orientation)

    def _pick_goal(self, pos, quat):
        goal = BasicRobotAction.Goal()
        goal.header = self.fill_header()
        goal.key = self._robot_key
        goal.robot_action_type = RobotActionType(type=RobotActionType.PICK_N_HOME)
        goal.robot_id = DEFAULT_ROBOT
        goal.targets = [
            TargetObject(
                eef_pose=Pose(
                    position=Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2])),
                    orientation=Quaternion(x=float(quat[0]), y=float(quat[1]), z=float(quat[2]), w=float(quat[3])),
                )
            )
        ]
        return goal

    async def pnp(self, position_from, orientation_from, position_to, orientation_to):
        return await self._perform_action(self._pick_goal, pos_from=position_from, quat_from=orientation_from, pos_to=position_to, quat_to=orientation_to)

    def _pnp_goal(self, pos_from, quat_from, pos_to, quat_to):
        goal = BasicRobotAction.Goal()
        goal.header = self.fill_header()
        goal.key = self._robot_key
        goal.robot_action_type = RobotActionType(type=RobotActionType.PICK_N_PLACE)
        goal.robot_id = DEFAULT_ROBOT
        goal.targets = [
            TargetObject(
                eef_pose=Pose(
                    position=Point(x=float(pos_from[0]), y=float(pos_from[1]), z=float(pos_from[2])),
                    orientation=Quaternion(x=float(quat_from[0]), y=float(quat_from[1]), z=float(quat_from[2]), w=float(quat_from[3])),
                )
            ),
            TargetObject(
                eef_pose=Pose(
                    position=Point(x=float(pos_to[0]), y=float(pos_to[1]), z=float(pos_to[2])),
                    orientation=Quaternion(x=float(quat_to[0]), y=float(quat_to[1]), z=float(quat_to[2]), w=float(quat_to[3])),
                )
            )
        ]
        return goal


class RobotActionClient():
    ROBOT_NAMESPACE = 'action_robot/'

    def __init__(self, node: Node = None, robot_server_address: str = ROBOT_NAMESPACE):
        if not rclpy.ok():  # assumes that rclpy was not initialized
            rclpy.init()  # TODO: additional error handling
        if node is None:  # if no node was provided, create a new node
            self._node = rclpy.create_node(f"robot_client_{str(uuid4()).replace('-', '_')}")
            self._internal_node = True
        else:
            self._node = node
            self._internal_node = False

        self._robot_caller = RobotServiceCaller(self._node, robot_server_address)
        self._basic_skills = BasicSkills(self._node, self._robot_caller, robot_server_address)

        self._job_queue = asyncio.Queue()
        # self._task_queue = asyncio.Queue()
        # self._job_queue = deque()
        self._task_queue = deque()

        self._started = False
        self._runner_thread = None
        self._pool = None

    async def basic_point_async(self, *position) -> Tuple[int, str]:
        print("cccccccccccccccccccc")
        return await self._basic_skills.point(*position)

    def basic_point(self, *position) -> Tuple[int, str]:
        self._loop.call_soon_threadsafe(self._job_queue.put_nowait, (self.basic_point_async, *position))
        return True

    async def basic_pick_async(self, position, orientation) -> Tuple[int, str]:
        return await self._basic_skills.pick(position, orientation)

    async def basic_pnp_async(self, position_from, orientation_from, position_to, orientation_to) -> Tuple[int, str]:
        return await self._basic_skills.pnp(position_from, orientation_from, position_to, orientation_to)

    async def _spinning(self):
        while rclpy.ok():
            rclpy.spin_once(self._node, timeout_sec=0.01)
            await asyncio.sleep(0.001)

    async def _run(self):
        print("running...")
        if self._internal_node:
            spin_task = self._loop.create_task(self._spinning())
        else:
            spin_task = None
        while rclpy.ok():
            print("looping")
            if not self._job_queue.empty():
                task_func, params = await self._job_queue.get()
                task = self._loop.create_task(
                # task = asyncio.run_coroutine_threadsafe(
                    task_func(params),
                    # self._loop
                    )
                # self._loop.call_soon_threadsafe(self._task_queue.put_nowait, task)
                self._job_queue.task_done()
                self._task_queue.appendleft(task)

            print("1")
            if len(self._task_queue) > 0:
            # if not self._task_queue.empty():
                print(self._task_queue)
                # print(self._task_queue[0].__dir__())
                # print(self._task_queue[0].running())
                # finished, unfinished = self._loop.run_until_complete(asyncio.wait(self._task_queue, timeout=0.01))
                finished, unfinished = await asyncio.wait(self._task_queue, timeout=0.01)

                print("2")
                print(f'unfinished: {len(unfinished)}')
                print(f'finished: {len(finished)}')
                print("3")
                for task in finished:
                    # self._task_queue.task_done()
                    try:
                        print('Task {} done\nresult: {}\nstatus flag: {}\ncomment: {}'.format(task.get_coro(), *task.result()))
                    except TypeError as e:
                        print(task.result())

            print("4")
        if spin_task is not None:
            spin_task.cancel()

    def _start_loop(self):
        print("starting...")
        self._loop.run_until_complete(self._run())
        # future = asyncio.run_coroutine_threadsafe(self._run, self._loop)
        # print(future)

    def start(self) -> None:
        if self._started:
            raise RuntimeError('Cannot start the client multiple times!')
        self._started = True

        self._loop = asyncio.get_event_loop()
        self._loop.set_debug(enabled=True)

        self._pool = concurrent.futures.ThreadPoolExecutor(10)
        # self._pool = concurrent.futures.ProcessPoolExecutor(10)
        # self._loop.run_in_executor(self._pool, self._start_loop)
        # self._loop.run_in_executor(self._pool, self._run)
        self._runner_thread = threading.Thread(target=self._start_loop, daemon=False)
        self._runner_thread.start()

    def __del__(self):
        print("Shutdown")
        if self._pool is not None:
            self._pool.shutdown(wait=False, cancel_futures=True)

    @property
    def node(self):
        return self._node
