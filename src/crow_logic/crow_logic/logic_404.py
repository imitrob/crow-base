import json
import time
import traceback as tb
from typing import List

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from rclpy.action import ActionClient
from rclpy.callback_groups import (MutuallyExclusiveCallbackGroup,
                                   ReentrantCallbackGroup)
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from rdflib import BNode, Graph, Literal, URIRef

from context_based_gesture_operation.srcmodules.Objects import \
    Object as Object2
from context_based_gesture_operation.srcmodules.Scenes import Scene as Scene2
from crow_monitor.profiler.profiling import StatTimer
from crow_monitor.utils.queue_server import QueueServer
from crow_msgs.msg import CommandType
from crow_ontology.crowracle_client import CrowtologyClient
from crow_params.client import ParamClient
from crow_robot_msgs.action import RobotAction
from crow_robot_msgs.msg import ActionResultFlag, ObjectType, RobotActionType
from crow_robot_msgs.srv import GetRobotStatus
from crow_utils import crow_config
from imitrob_robot_client.robot_client import RobotActionClient
from imitrob_hri.imitrob_nlp.TemplateFactory import create_template
from imitrob_templates.templates import TaskExecutionMode
from mvae.sentence2trajectory import MVAEProcessor
from teleop_msgs.msg import HRICommand, Intent


class OntologyClientAdHoc():
    def __init__(self, crowracle_client):
        self.crowracle = crowracle_client
        self.onto = self.crowracle.onto
        print("Ontology Client Ready")

    NAME2TYPE = {
        'cube': 'Object',
    }

    def get_updated_scene(self):
        s = None
        s = Scene2(init='', objects=[], random=False)

        objects = self.get_objects_from_onto()
        ''' object list; item is dictionary containing uri, id, color, color_nlp_name_CZ, EN, nlp_name_CZ, nlp_name_EN; absolute_location'''

        '''
        import rdflib
        uri = rdflib.term.URIRef('http://imitrob.ciirc.cvut.cz/ontologies/crow#test_CUBE_498551_od_498551')
        '''
        # [{'uri': rdflib.term.URIRef('http://imitrob.ciirc.cvut.cz/ontologies/crow#test_CUBE_498551_od_498551'), 'id': 'od_498551', 'color': rdflib.term.URIRef('http://imitrob.ciirc.cvut.cz/ontologies/crow#COLOR_GREEN'), 'color_nlp_name_CZ': 'zelená', 'color_nlp_name_EN': 'green', 'nlp_name_CZ': 'kostka', 'nlp_name_EN': 'cube', 'absolute_location': [-0.34157065, 0.15214929, -0.24279054]}]
        # [{'uri': rdflib.term.URIRef('http://imitrob.ciirc.cvut.cz/ontologies/crow#test_CUBE_498551_od_498551'), 'id': 'od_498551', 'color': rdflib.term.URIRef('http://imitrob.ciirc.cvut.cz/ontologies/crow#COLOR_GREEN'), 'color_nlp_name_CZ': 'zelená', 'color_nlp_name_EN': 'green', 'nlp_name_CZ': 'kostka', 'nlp_name_EN': 'cube', 'absolute_location': [-0.34157065, 0.15214929, -0.24279054]}]

        # Colors:
        # [COLOR_GREEN.

        for object in objects:
            ''' o is dictionary containing properties '''
            uri = object['uri']
            obj_id = object['id']

            # created name `wheel_od_0` extracted from 'http://imitrob.ciirc.cvut.cz/ontologies/crow#wheel_od_0'
            name = str(object['uri']).split("#")[-1]

            # color = object['color']
            # color_nlp_name_CZ = object['color_nlp_name_CZ']
            color_nlp_name_EN = object['color_nlp_name_EN']
            nlp_name_CZ = object['nlp_name_CZ']
            nlp_name_EN = object['nlp_name_EN']
            absolute_location = object['absolute_location']

            o = Object2(name=name, position_real=np.array(absolute_location), random=False)
            # o.quaternion = np.array(object['pose'][1])
            # o.color_uri = color
            o.color = color_nlp_name_EN
            # o.color_name_CZ = color_nlp_name_CZ

            o.nlp_name_CZ = nlp_name_CZ
            o.nlp_name_EN = nlp_name_EN
            o.crow_id = obj_id
            o.crow_uri = uri

            s.objects.append(o)

        return s

    def match_object_in_onto(self, obj):
        onto = self.get_objects_from_onto()
        obj = json.loads(obj)
        for o in onto:
            url_raw = o["uri"]
            if url_raw is None:
                continue
            url = str(url_raw)
            if url == obj[0]["target"][0]:
                return o
        return None

    def get_objects_from_onto(self):
        o_list = self.crowracle.getTangibleObjectsProps()
        return o_list

    # def get_action_from_cmd(self, cmd):
    #     action = json.loads(cmd)[0]["action_type"].lower()
    #     if action in ACTION_TRANSL.keys():
    #         return ACTION_TRANSL[action]
    #     else:
    #         print("No czech translation for action " + action)
    #         return action


class ControlLogic(Node):
    COMMAND_TOPIC = "/hri/command"
    DEBUG = False
    UPDATE_INTERVAL = 0.1
    MAX_QUEUE_LENGTH = 10

    STATUS_IDLE = 0  # nothing is happening
    STATUS_PROCESSING = 2  # a command from the main buffer is being processed
    STATUS_SENDING_GOAL = 4  # action goal is being processed end sent to the robot
    STATUS_EXECUTING = 8  # robot is executing the goal

    ROBOT_ACTION_PHASE = 0

    def __init__(self):
        super().__init__('control_logic')

        # TODO: load exec mode from config
        # setup_config = crow_config.get_config_file()
        # self.execution_mode: TaskExecutionMode = setup_config["robot"]["execution_mode"]
        self.execution_mode = TaskExecutionMode.BASIC
        # self.execution_mode = TaskExecutionMode.MVAE

        if self.execution_mode not in TaskExecutionMode:
            raise ValueError(f"Invalid execution mode: {self.execution_mode}! Check setup config.")
        elif self.execution_mode == TaskExecutionMode.UNDEF:
            raise ValueError("Undefined execution mode (set to 0)! Check setup config.")

        self.crowracle = CrowtologyClient(node=self)
        self.onto = self.crowracle.onto
        self.oc = OntologyClientAdHoc(self.crowracle)

        self.pclient = ParamClient()

        self.command_main_buffer = QueueServer(maxlen=self.MAX_QUEUE_LENGTH, queue_name='main')
        self.status = self.STATUS_IDLE
        self.create_timer(self.UPDATE_INTERVAL, self.update_main_cb, callback_group=ReentrantCallbackGroup())
        self.create_subscription(HRICommand, self.COMMAND_TOPIC, self.command_cb, 10)

        self.robot_client = RobotActionClient(self)
        self.robot_client.start()

        self.mvae = MVAEProcessor(direct_execute=False)

        StatTimer.init()

    def command_cb(self, msg):
        # self.get_logger().info("Received command msg!")
        data = json.loads(msg.data[0])
        if type(data) is dict:
            data = [data]

        self.get_logger().info(f"Received {len(data)} command(s)")
        # self.get_logger().info(f"Received {len(data)} command(s):\n{data}")
        for d in data:
            self.command_main_buffer.append(d)

    def update_main_cb(self):
        self.pclient.logic_alive = time.time()
        self.get_logger().debug(f"Status: {self.status}")
        if self.status == self.STATUS_IDLE:  # when robot is idle
            self._set_status(self.STATUS_PROCESSING)
            try:
                hri_command = self.command_main_buffer.pop()
            except IndexError:  # no new commands to process
                self._set_status(self.STATUS_IDLE)
            else:
                # command = self.COMMAND_DICT.get(action, self.command_error)
                # TODO: execute command
                ret = self.handle_hricommand(hri_command)
                if ret:
                    self.get_logger().info("Command processed successfully.")
                else:
                    self.get_logger().error("Command processing failed.")
            finally:
                self._set_status(self.STATUS_IDLE)
                # self.status = self.status & ~self.STATUS_PROCESSING
        # else:
        #     print("busy")

    def _print_error_and_tb(self, error_msg=None, custom_message=None, include_traceback=True, traceback_limit=20):
        if custom_message is not None:
            self.get_logger().error(custom_message)
        if error_msg is not None:
            self.get_logger().error(error_msg)
        if include_traceback:
            self.get_logger().error(f"Traceback:\n{tb.format_exc(limit=traceback_limit)}")

    def handle_hricommand(self, receivedHRIcommand_parsed) -> bool:
        if receivedHRIcommand_parsed is None:
            self.get_logger().error("Received empty command! (somehow?)")
            return False
        if type(receivedHRIcommand_parsed) is not dict:
            self.get_logger().error(f"The command is not a dictionary! {type(receivedHRIcommand_parsed)}\n{receivedHRIcommand_parsed}")
            return False

        target_action = receivedHRIcommand_parsed['target_action']
        self.get_logger().info("Executing a command...")
        try:
            # template_names = receivedHRIcommand_parsed['actions']
            target_action = receivedHRIcommand_parsed['target_action']
            # template_probs = receivedHRIcommand_parsed['action_probs']
            # receivedHRIcommand_parsed['action_timestamp']
            target_object = receivedHRIcommand_parsed['target_object']
            # object_names = receivedHRIcommand_parsed['objects']
            # object_probs = receivedHRIcommand_parsed['object_probs']
            receivedHRIcommand_parsed['object_classes']
            parameters = receivedHRIcommand_parsed['parameters']

            i = Intent()
            i.target_action = target_action
            i.target_object = target_object  # 'cube_holes_od_0' #s.objects[0].name

            # Parameters

            task = create_template(i.target_action)

        except ValueError as e:
            self._print_error_and_tb(e, "Failed to create template from command, probably missing value in the incoming command msg:")
            return False
        except KeyboardInterrupt:
            self.get_logger().warn("Interrupted called while creating template from a command. Terminating command execution.")
            return False
        except BaseException as e:
            self._print_error_and_tb("Failed to create template from command:")
            return False

        try:
            updated_scene = self.oc.get_updated_scene()
            assert task.match_intent(i, updated_scene), "Could not match template"
            # task.ground(s=self.oc.get_objects_from_onto())

            print(updated_scene)
            # input("check updated scene !")
            task.target_object = target_object
            task.ground_scene(updated_scene)

            # print("********************")
            # for k, v in task.__dict__.items():
            #     print(f"{k}: {v}")
            # print("********************")
        except BaseException as e:
            self._print_error_and_tb(e, "Failed to ground template:")
            return False

        try:
            if self.execution_mode == TaskExecutionMode.BASIC:
                task.execute(self.robot_client, mode=TaskExecutionMode.BASIC)
            elif self.execution_mode == TaskExecutionMode.MVAE:
                task.execute(self.robot_client, mode=TaskExecutionMode.MVAE, mvae=self.mvae)
            else:
                self.get_logger().error(f"Unknown execution mode: {self.execution_mode}")
                return False
        except BaseException as e:
            self._print_error_and_tb(e, "Error executing command:")
            return False

        return True

    def make_robot_done(self):
        """Call this when robot successfully completed the last task
        """
        self._cleanup_after_action()
        self.pclient.robot_failed = False

    def make_robot_fail(self):
        """Call this when robot fails to complete an action
        """
        self._cleanup_after_action()
        self.pclient.robot_failed = True

    def make_robot_fail_to_start(self):
        """Call this when robot fails to start an action (so far, the same as failing to complete)
        """
        self.get_logger().info('Goal rejected :(')
        self._cleanup_after_action()
        self.pclient.robot_failed = True

    def _cleanup_after_action(self):
        self.goal_handle = None
        self.pclient.robot_done = True
        self.status = self.status & ~(self.STATUS_EXECUTING | self.STATUS_SENDING_GOAL)
        # self._set_status(self.STATUS_IDLE)

    def _set_status(self, status):
        self.status = status


def main():
    rclpy.init()
    cl = ControlLogic()
    try:
        n_threads = 4  # nr of callbacks in group, +1 as backup
        mte = MultiThreadedExecutor(num_threads=n_threads, context=rclpy.get_default_context())
        rclpy.spin_once(cl, executor=mte)

        cl.get_logger().info("ready")

        rclpy.spin(cl, executor=mte)
        cl.destroy_node()
    except KeyboardInterrupt:
        print("User requested shutdown.")
    except BaseException as e:
        print(f"Some error had occured: {e}")
        tb.print_exc()
    finally:
        cl.pclient.robot_failed = False
        cl.pclient.robot_done = True
        cl.pclient.ready_for_next_sentence = False


if __name__ == '__main__':
    main()
