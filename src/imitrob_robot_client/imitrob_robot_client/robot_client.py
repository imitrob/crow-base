import numpy as np
import time
import threading

from action_msgs.msg import GoalStatus
from imitrob_common_interfaces.action import BasicRobotAction, TrajectoryAction
from imitrob_common_interfaces.msg import RobotActionType, ResourceHandling, TargetObject, OperationMode, OperationModeChangeReFlag, ObjectType
from imitrob_common_interfaces.srv import ReserveResource, GetOperationMode, SetOperationMode, ListOperationModes
from trajectory_msgs.msg import JointTrajectoryPoint

from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from uuid import uuid4

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from typing import Optional, Tuple, Iterable
from collections import deque
from copy import deepcopy
import traceback as tb


SERVICE_LIST = {
    "resource": 'reserve_robot_resource',
    "get_mode": 'get_operation_mode',
    "set_mode": 'set_operation_mode',
    "list_mode": 'list_operation_modes',
}
STANDARD_FRAME = "robot_base"
DEFAULT_ROBOT = 1


class ROSHandler():
    SPIN_TIMEOUT = 1000

    def __init__(self, node: Node, internal_mode: bool, executor: Optional[rclpy.executors.Executor] = None) -> None:
        self.__node = node
        self._internal_mode = internal_mode
        self._peding_lock = False

        self._executor = executor
        self._logger = self.__node.get_logger()

    def wait_for_future(self, future, timeout: Optional[float] = None) -> None:
        if timeout is None:
            timeout = self.SPIN_TIMEOUT
        if not self._internal_mode:
            rclpy.spin_until_future_complete(self.__node, future, self._executor, timeout)
        else:
            can_spin = True
            try:
                rclpy.spin_once(self.__node)
            except ValueError:
                can_spin = False
                raise NotImplementedError("Cannot spin the node for some reason. Handling of this situation is not implemented yet.")

            if can_spin:
                rclpy.spin_until_future_complete(self.__node, future, self._executor, timeout)
            else:
                self._peding_lock = True  # lock and wait
                future.add_done_callback(self._unlock_peding_lock)
                print("pending")
                while self._peding_lock:
                    time.sleep(0.01)

        if not future.done():
            raise RuntimeError(f"Somehow, the future {future} is not complete!")

        res = future.result()
        return res

    def _unlock_peding_lock(self, _) -> None:
        print(f"Future is done: {_.done()}")
        self._peding_lock = False

    @property
    def node(self):
        return self.__node

    def info(self, msg):
        self._logger.info(msg)

    def debug(self, msg):
        self._logger.debug(msg)

    def warn(self, msg):
        self._logger.warn(msg)

    def error(self, msg):
        self._logger.error(msg)


class RobotServiceCaller():

    def __init__(self, ros_handler: ROSHandler, robot_ns: str):
        self.__ros_handler = ros_handler
        self._resource_srv = self.__ros_handler.node.create_client(ReserveResource, f'{robot_ns}{SERVICE_LIST["resource"]}')
        self._get_mode_srv = self.__ros_handler.node.create_client(GetOperationMode, f'{robot_ns}{SERVICE_LIST["get_mode"]}')
        self._set_mode_srv = self.__ros_handler.node.create_client(SetOperationMode, f'{robot_ns}{SERVICE_LIST["set_mode"]}')
        self._list_mode_srv = self.__ros_handler.node.create_client(ListOperationModes, f'{robot_ns}{SERVICE_LIST["list_mode"]}')

    def __call_service(self, service, request):
        service.wait_for_service(5)
        future = service.call_async(request)
        # print(f"T: {future}, {type(future)}")
        try:
            res = self.__ros_handler.wait_for_future(future)
        except BaseException as e:
            self.__ros_handler.error(f"Error when calling service '{service.srv_name}': \n{e}")
            tb.print_exc()
        # rclpy.spin_until_future_complete(self.__node, future)
        # res = future.result()
        return res

    def get_operation_mode(self) -> GetOperationMode.Response:
        response = self.__call_service(self._get_mode_srv, GetOperationMode.Request())
        assert isinstance(response, GetOperationMode.Response)
        return response

    def set_operation_mode(self, operation_mode, key) -> SetOperationMode.Response:
        request = SetOperationMode.Request(operation_mode=OperationMode(mode=operation_mode), key=key)
        response = self.__call_service(self._set_mode_srv, request)
        assert isinstance(response, SetOperationMode.Response)
        return response

    def list_operation_modes(self):
        return self.__call_service(self._list_mode_srv, ListOperationModes.Request())

    def reserve_robot(self, comment="pick_at_pose", rid=DEFAULT_ROBOT) -> ReserveResource.Response:
        reqest = ReserveResource.Request(
            resource=rid,
            requester=self.__ros_handler.node.get_name(),
            action=ResourceHandling.RESERVE,
            comment=comment,
        )
        response = self.__call_service(self._resource_srv, reqest)
        assert isinstance(response, ReserveResource.Response)
        return response

    def release_robot(self, key, rid=DEFAULT_ROBOT):
        reqest = ReserveResource.Request(
            resource=rid,
            requester=self.__ros_handler.node.get_name(),
            action=ResourceHandling.RELEASE,
            key=key,
        )
        return self.__call_service(self._resource_srv, reqest)


class ModusOperandi():

    def __init__(self, ros_handler: ROSHandler, robot_caller: RobotServiceCaller):
        self._robot_caller = robot_caller
        self._ros_handler = ros_handler

        #
        self._action_client = None
        self._action_class = None
        self._required_operation_mode = None

        self.__robot_key = None

    @property
    def action_class(self):
        return self._action_class

    def _perform_action(self, action, *args, **kwargs) -> Tuple[bool, int, str]:
        if self.__robot_key is None:
            response: ReserveResource.Response = self._robot_caller.reserve_robot()
            if not response.success:
                return False, -1, "Failed to reserve robot"
            self.__robot_key = response.key
        success = False
        flag = -2
        message = "<>"
        try:
            if self._robot_caller.get_operation_mode().operation_mode.mode != self._required_operation_mode:
                self._ros_handler.warn("Robot was in different operation mode, trying to change it...")
                response_set_mode = self._robot_caller.set_operation_mode(self._required_operation_mode, self.__robot_key)
                if response_set_mode.response_flag.value & OperationModeChangeReFlag.NOK:
                    success, flag, message = False, response_set_mode.response_flag.value, f"Couldn't set the required operation mode.\n{response_set_mode.msg}"
                else:
                    self._ros_handler.info(f"Operation mode changed to {response_set_mode}")

            try:
                goal = action(*args, **kwargs)
            except BaseException as e:
                success, flag, message = False, -1, f"Failed to formulate action goal.\n{e}"

            if flag != -1:
                try:
                    success, flag, message = self.__handle_action(goal)
                except BaseException as e:
                    success, flag, message = False, -1, f"Action execution failure.\n{e}"
        finally:
            self._ros_handler.info(f"Robot action ended with\n\tsuccess: {success}\n\tflag: {flag}\n\tmessage: {message}")
            self._release_robot()

        return success, flag, message

    def _release_robot(self):
        self._robot_caller.release_robot(self.__robot_key)
        self.__robot_key = None

    def _feedback_callback(self, feedback):
        self._ros_handler.info('Received feedback: {0}'.format(feedback.feedback))

    def __handle_action(self, goal):
        self._ros_handler.debug("Waiting for server")
        assert self._action_client is not None, "Action client was not initliazed!"
        self._action_client.wait_for_server()
        self._ros_handler.debug('Sending goal request...')

        try:
            goal_future = self._action_client.send_goal_async(
                goal,
                feedback_callback=self._feedback_callback
            )
        except BaseException as e:
            return False, -1, f"Failed to send goal request because:\n{e}"

        goal_handle = self._ros_handler.wait_for_future(goal_future)
        if not goal_handle.accepted:
            self._ros_handler.warn('Goal rejected :(')
            return False, -1, "Goal rejected"

        self._ros_handler.debug('Goal accepted :)')

        try:
            result_future = goal_handle.get_result_async()
        except:
            return False, -1, "Action execution failed"

        res = self._ros_handler.wait_for_future(result_future)
        result = res.result
        status = res.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._ros_handler.debug('Goal succeeded! Result: {0}'.format(result))
        else:
            self._ros_handler.warn('Goal failed with status: {0}'.format(status))
        return status == GoalStatus.STATUS_SUCCEEDED, result.action_result_flag.flag, "Action done"

    def fill_header(self):
        header = Header()
        header.frame_id = STANDARD_FRAME
        header.stamp = self._ros_handler.node.get_clock().now().to_msg()
        return header

    def create_n_fill_goal(self, robot_id: Optional[int] = None):
        if self._action_class is None:
            raise ValueError("Action class was not set!")

        if self.__robot_key is None:
            raise ValueError("Robot key was not set!")

        goal = self._action_class.Goal()
        goal.header = self.fill_header()
        goal.key = self._robot_key
        goal.robot_id = robot_id if robot_id is not None else DEFAULT_ROBOT
        return goal

    @property
    def _robot_key(self) -> str:
        return self.__robot_key


class BasicSkills(ModusOperandi):

    def __init__(self, ros_handler: ROSHandler, robot_caller: RobotServiceCaller, robot_ns: str):
        super().__init__(ros_handler, robot_caller)
        self._action_class = BasicRobotAction
        self._required_operation_mode = OperationMode.BASIC_SKILL_MODE
        self._action_client = ActionClient(ros_handler.node, self._action_class, f'/{robot_ns}basic_robot_action')

    def point(self, position):
        try:
            res = self._perform_action(self._point_goal, position)
        except Exception as e:
            raise e  # FIXME: check whether to raise or return the exception
        return res

    def _point_goal(self, position):
        goal = self.create_n_fill_goal()
        goal.robot_action_type = RobotActionType(type=RobotActionType.POINT)
        goal.targets = [
            TargetObject(
                eef_pose=Pose(
                    position=Point(x=float(position[0]), y=float(position[1]), z=float(position[2])),
                )
            )
        ]
        return goal
    
    def go_home(self):
        return self._perform_action(self._go_home_goal)
    
    def _go_home_goal(self):
        goal = self.create_n_fill_goal()
        goal.robot_action_type = RobotActionType(type=RobotActionType.HOME)
        return goal

    def pick(self, position, orientation):
        return self._perform_action(self._pick_goal, pos=position, quat=orientation)

    def _pick_goal(self, pos, quat):
        goal = self.create_n_fill_goal()
        goal.robot_action_type = RobotActionType(type=RobotActionType.PICK_N_HOME)
        goal.targets = [
            TargetObject(
                eef_pose=Pose(
                    position=Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2])),
                    orientation=Quaternion(x=float(quat[0]), y=float(quat[1]), z=float(quat[2]), w=float(quat[3])),
                )
            )
        ]
        return goal

    def pnp(self, position_from, orientation_from, position_to, orientation_to):
        return self._perform_action(self._pick_goal, pos_from=position_from, quat_from=orientation_from, pos_to=position_to, quat_to=orientation_to)

    def _pnp_goal(self, pos_from, quat_from, pos_to, quat_to):
        goal = self.create_n_fill_goal()
        goal.robot_action_type = RobotActionType(type=RobotActionType.PICK_N_PLACE)
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


class TrajectoryExecution(ModusOperandi):

    def __init__(self, ros_handler: ROSHandler, robot_caller: RobotServiceCaller, robot_ns: str):
        super().__init__(ros_handler, robot_caller)
        self._action_class = TrajectoryAction
        self._required_operation_mode = OperationMode.TRAJECTORY_MODE
        self._action_client = ActionClient(ros_handler.node, self._action_class, f'/{robot_ns}trajectory_robot_action')


    def trajectory_cart(self, p: list[Iterable[float]], q: list[Iterable[float]]):
        try:
            res = self._perform_action(self._trajectory_cart_goal, p, q)
        except Exception as e:
            raise e  # FIXME: check whether to raise or return the exception
        return res

    def _trajectory_cart_goal(self, p: list[Iterable[float]], q: list[Iterable[float]]):
        goal: TrajectoryAction.Goal = self.create_n_fill_goal()
        goal.ee_frame = "ee"

        assert len(p) == len(q)

        for p_, q_ in zip(p, q):
            pose = PoseStamped()
            pose.pose.position.x = p_[0]
            pose.pose.position.y = p_[1]
            pose.pose.position.z = p_[2]
            pose.pose.orientation.w = q_[0]
            pose.pose.orientation.x = q_[1]
            pose.pose.orientation.y = q_[2]
            pose.pose.orientation.z = q_[3]

            goal.pose.append(pose)

        goal.use_joint_traj = False

        return goal

    def trajectory_joint(self, trajectory: list[Iterable[float]], gripper: list):
        try:
            res = self._perform_action(self._trajectory_joint_goal, trajectory, gripper)
        except Exception as e:
            raise e  # FIXME: check whether to raise or return the exception
        return res

    def _trajectory_joint_goal(self, trajectory=(), gripper=()):
        goal: TrajectoryAction.Goal = self.create_n_fill_goal()
        goal.ee_frame = "ee"

        if len(gripper) == 1:
            goal.close_gripper.append(gripper[0])
            goal.use_joint_traj = False

        elif len(gripper) == len(trajectory):
            goal.use_joint_traj = True

            for traj_pt in trajectory:
                new_pt = JointTrajectoryPoint()
                new_pt.positions = list(traj_pt)
                goal.joint_traj.points.append(new_pt)

            for gr_pt in gripper:
                goal.close_gripper.append(gr_pt)

        elif len(gripper) == 0 and len(trajectory) >= 1:
            goal.use_joint_traj = True

            for traj_pt in trajectory:
                new_pt = JointTrajectoryPoint()
                new_pt.positions = list(traj_pt)
                goal.joint_traj.points.append(new_pt)

        else:
            raise NotImplementedError(f"Combination of #gripper values {len(gripper)} and #traj_points {len(trajectory)} is not implemented.")



        # for traj_pt in trajectory:
        #     new_pt = JointTrajectoryPoint()
        #     new_pt.positions = list(traj_pt)
        #     goal.joint_traj.points.append(new_pt)

        
        goal.scripted_approach = True

        # FIXME: define this via arguments
        goal.obj_pose.pose.position.x, goal.obj_pose.pose.position.y, goal.obj_pose.pose.position.z = 0.5, 0.0, 0.02
        goal.obj_pose.pose.orientation.x, goal.obj_pose.pose.orientation.y, goal.obj_pose.pose.orientation.z, goal.obj_pose.pose.orientation.w = 0.0, 0.0, 0.0, 1.0
        goal.obj_pose.header.frame_id = "cube_frame_0"
        goal.obj_pose.header.stamp = self._ros_handler.node.get_clock().now().to_msg()
        goal.obj_type.type = ObjectType.CUBE
        return goal


""" IK part might be moved somewhere else 
    - However it is quite compact & complete
"""
import roboticstoolbox as rtb
from spatialmath import SE3, UnitQuaternion

class Rtb_IK_custom_wrapper():
    def __init__(self):
        self.robot = rtb.models.Panda()
        self.last_q = []

    def __call__(self, p, q = [1., 0., 0., 0.]):
        """ do custom inverse kinematics

        Args:
            p (Float[3]): Cartesian Position
            q (Float[4]): Absolute Quaternion Orienation w.r.t. robot base
                - quternion notation: x,y,z,w
            
        Raises:
            Exception: When IK not found

        Returns:
            Float[7]: 7 DoF robot joint configuration
        """
        assert len(p) == 3, f"position length: {len(p)} != 3"
        assert len(q) == 4, f"quaternion length: {len(q)} != 4"
        
        p = deepcopy(p)
        q = deepcopy(q)
        q[1], q[2], q[3], q[0] = q[0], q[1], q[2], q[3] 

        q1, succ, reason, iter, res = self.robot.ik_LM(SE3.Trans(*p) * UnitQuaternion(np.array(q)).SE3(), q0=self.last_q)
        if not succ:
            raise Exception(f"IK not found because: {reason}")
        print("last q before: ", self.last_q)
        self.last_q = q1
        print("last q: ", self.last_q)
        return q1


class RobotActionClient():
    ROBOT_NAMESPACE = 'action_robot/'

    def __init__(self, node: Optional[Node] = None, robot_server_address: str = ROBOT_NAMESPACE):
        if not rclpy.ok():  # assumes that rclpy was not initialized
            rclpy.init()  # TODO: additional error handling
        if node is None:  # if no node was provided, create a new node
            # self._node = rclpy.create_node(f"robot_client_{str(uuid4()).replace('-', '_')}")
            self._node = Node("robot_client_123")

            self._internal_node = True
        else:
            self._node = node
            self._internal_node = False

        self._ros_handler = ROSHandler(self._node, internal_mode=self._internal_node)

        self._robot_caller = RobotServiceCaller(self._ros_handler, robot_server_address)
        self._basic_skills = BasicSkills(self._ros_handler, self._robot_caller, robot_server_address)
        self._trajectory_skills = TrajectoryExecution(self._ros_handler, self._robot_caller, robot_server_address)

        self._job_queue = deque()
        self._task_queue = deque()

        self._started = False
        self._runner_thread = None
        self._pool = None

        self.ik = Rtb_IK_custom_wrapper()

    def basic_pick(self, position, orientation) -> Tuple[bool, int, str]:
        return self._basic_skills.pick(position=position, orientation=orientation)

    def basic_point(self, *position) -> Tuple[bool, int, str]:
        return self._basic_skills.point(*position)

    def trajectory_joint(self, trajectory: list[Iterable], gripper: Optional[list[Iterable]] = []) -> Tuple[bool, int, str]:
        return self._trajectory_skills.trajectory_joint(trajectory, gripper)

    def trajectory_pose(self, p: list[Iterable], q: list[Iterable]):
        """ Executes list of eef poses, Array version of move_pose function

        Args:
            p (list[n][3]): List of Points, n = # of poses
            q (list[n][4]): List of Orientations, n = # of poses

        Returns:
            success, flag, message
        """        
        trajectory = []
        for p_, q_ in zip(p, q):
            print(f"[Move check] Moving to {p_}, {q_}")
            trajectory.append(self.ik(p=p_, q=q_))

        return self.trajectory_joint(trajectory = trajectory)

    def move_pose(self, p: list[Iterable], q: list[Iterable]):
        """ Moves to single eef pose

        Args:
            p (list[3]): Cartesian Position of robot eef
            q (list[3]): Quaternion Orietation of robot eef w.r.t. robot base

        Returns:
            success, flag, message
        """        
        assert len(p) == 3, f"position length: {len(p)} != 3"
        assert len(q) == 4, f"quaternion length: {len(q)} != 4"
    
        print(f"[Move check] Moving to {p}, {q}")
        return self.trajectory_joint(trajectory = [self.ik(p=p, q=q)])

    def close_gripper(self):
        """ Close gripper convenience function
        """        
        return self._trajectory_skills.trajectory_joint(trajectory=[], gripper=[True])

    def open_gripper(self):
        """ Open gripper convenience function
        """        
        return self._trajectory_skills.trajectory_joint(trajectory=[], gripper=[False])


    # async def basic_pick_async(self, position, orientation) -> Tuple[int, str]:
    #     return await self._basic_skills.pick(position, orientation)

    # async def basic_pnp_async(self, position_from, orientation_from, position_to, orientation_to) -> Tuple[int, str]:
    #     return await self._basic_skills.pnp(position_from, orientation_from, position_to, orientation_to)

    def _spinning(self):
        mle = rclpy.executors.MultiThreadedExecutor(3)
        rclpy.spin(self._node, mle)

    def start(self) -> None:
        if self._started:
            raise RuntimeError('Cannot start the client multiple times!')
        self._started = True

        if self._internal_node:
            self._runner_thread = threading.Thread(target=self._spinning, daemon=True)
            self._runner_thread.start()

    @property
    def node(self):
        return self._node
