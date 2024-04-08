import rclpy
from rclpy.node import Node
from rclpy.time import Time, Duration
from ros2param.api import call_get_parameters
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from geometry_msgs.msg import PoseArray
import message_filters
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import traceback as tb
import curses
import time
import numpy as np
from datetime import datetime
from curses.textpad import Textbox, rectangle
from crow_ontology.crowracle_client import CrowtologyClient
from rdflib.namespace import Namespace, RDF, RDFS, OWL, FOAF, XSD
from rdflib import URIRef, BNode, Literal, Graph
from rdflib.term import Identifier
from rcl_interfaces.srv import GetParameters
from threading import Thread, RLock
from queue import Queue, Full, Empty

from onto_class_templates import OntoCommunicator, BaseClass, ExistingClass, NewClass


ONTO_IRI = "http://imitrob.ciirc.cvut.cz/ontologies/crow"
CROW = Namespace(f"{ONTO_IRI}#")


class OntoLoader(OntoCommunicator):

    def __init__(self):
        self.crowracle = CrowtologyClient()
        self.onto = self.crowracle.onto

    def query(self, query_string) -> str:
        response = self.onto.query(query_string)
        return response


if __name__ == "__main__":
    ol = OntoLoader()
    BaseClass.set_onto_communicator(ol)

    class TangibleObject(ExistingClass):

        def __init__(self):
            print("Initiated")
            super().__init__()

    to = TangibleObject()
    print(to.uri)
