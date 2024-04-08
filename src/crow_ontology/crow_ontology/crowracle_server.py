import os
import argparse
import sys
import time
from importlib.util import find_spec
from typing import ClassVar, List
from uuid import uuid4

import rclpy
from rclpy.exceptions import ParameterAlreadyDeclaredException
from rclpy.exceptions import InvalidParameterException
from rclpy.exceptions import InvalidParameterValueException
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter

import yaml

from crow_ontology.backends.fuseki import FusekiBackend
from crow_ontology.backends.alchemy import AlchemyBackend

from crow_utils.crow_config import get_config_file, get_lib

from knowl import DBConfig, OntologyAPI
from rcl_interfaces.msg import ParameterType
from rclpy.node import ParameterDescriptor
from std_srvs.srv import Trigger


DB_PARAM_NAMES = ["database_host", "database_port", "database_uri", "database_name", "database_store"]
DB_PARAM_MAP = {
    "database_host": "host",
    "database_port": "port",
    "database_name": "database",
    "database_uri": "baseURL",
    "database_store": "store"
}
DB_PARAM_DESC = {
    "database_host": "Host address of the database server.",
    "database_port": "Port on which the database server is running.",
    "database_name": "Name of the database (e.g. an SQL DB name).",
    "database_uri": "Ontology base string / base namespace.",
    "database_store": "Store type, can be alchemy or fuseki."
}

class CrowtologyServer():
    """
    """
    # crowNSString = "http://imitrob.ciirc.cvut.cz/ontologies/crow#"
    # CROW = Namespace(crowNSString)
    # OWLR = Namespace("http://www.lesfleursdunormal.fr/static/_downloads/owlready_ontology.owl#")
    BACKUP_ON_CLEAR: ClassVar[bool] = False
    CLEAR_ON_START: ClassVar[bool] = True  # if clearing is False, backup has no effect (i.e. only bkp if clearing)
    ALLOW_CLIENTS_CLEARING_DATA: ClassVar[bool] = True  # if true, ROS service to clear/reset the DB will be created

    def __init__(self, config_path=None, base_onto_path="onto_draft_04.owl", mode="ros"):
        """
        Args:
            config_path (_type_, optional): _description_. Defaults to None.
            base_onto_path (_type_, optional): _description_. Defaults to None.
            mode (str, optional): "ros" or "local" (non-ros) mode. Defaults to "ros".
        """
        self.__mode = mode
        if self.mode == "ros":  # "ros"
            rclpy.init()
            self._node = rclpy.create_node(node_name="ontology_server")
            self.log("Setting up 'ontology_server' node.")
        else:  # "local"
            self._node = None

        if config_path is None:
            config_path = get_config_file("crow_ontology/db_config.yaml")

        self.modulePath = os.path.dirname(config_path)  # find_spec("crow_ontology").submodule_search_locations[0]

        if os.path.exists(base_onto_path):
            # self.base_onto_path = os.path.join(self.modulePath, "..", "data", "onto_draft_03.owl")
            self.base_onto_path = base_onto_path
        else:  # base path likely not absolute, try looking into data folder
            self.base_onto_path = get_config_file(os.path.join("crow_ontology", "data", base_onto_path))

        self.log(f"Using configuration file: {config_path}")
        self.log(f"Using base ontology file: {self.base_onto_path}")

        with open(config_path, 'r') as f:  # load the config
            self._cfg = yaml.safe_load(f)
        print(config_path)
        self._dbconf = DBConfig.factory(config_path)
        self._onto = OntologyAPI(self._dbconf)


        if "store" not in self._cfg:
            self._cfg["store"] = "alchemy"  # backwards compatibility for cfgs without store option
            self.log("No info about store type found in the config, assuming the default 'SQLAlchemy' store.")

        if self._cfg["store"] == "fuseki":
            self.log("Store type set to 'jena fuseki'.")
            self.backend = FusekiBackend(self._cfg, self._dbconf, self.log)
            self.backend.start()

        elif self._cfg["store"] == "alchemy":
            self.log("Store type set to 'SQLAlchemy'.")
            self.backend = AlchemyBackend(self._cfg, self._dbconf, self.log)
            self.backend.start()
        else:
            raise ValueError(f'Unknown store type {self._cfg["store"]}!')

        time.sleep(2)  # Wait for ontology server to start

        if self.CLEAR_ON_START:
            self.log("Clearing ontology (ClEAR_ON_START set to True)...")
            self.clear_data()

        self.start_param_server()

        self.log("Ontology DB is running.")

    def clear_data(self):
        # self.log("aaa")
        # self.log(f"Len onto: {self.onto}")
        if len(self.onto) > 0:
            if self.BACKUP_ON_CLEAR:
                self.log("Trying to backup existing data (BACKUP_ON_CLEAR is set to True)...")
                try:
                    bkp_path = os.path.join(self.modulePath, "..", "data", "backup", f"bkp_{uuid4()}.owl")
                    self.onto.graph.serialize(bkp_path, format="xml")
                    self.log("Backup complete.")
                except Exception as e:
                    self.log(f"Tried backing up the ontology but failed because: {e}")

            self.backend.clear(self.onto)
            self.log("Ontology cleared.")
        else:
            self.log("Ontology empty, clearing not performed.")

        self.log(f"Uploading data from {self.base_onto_path} into the database.")
        self.onto.mergeFileIntoDB(self.base_onto_path)
        self.log("Data uploaded.")

    def log(self, msg):
        if self.node is not None:
            self.node.get_logger().info(msg)
        else:
            print(msg)

    def clear_data_service_cb(self, request, response):
        self.log(f"A node has requested data clearing.")
        try:
            self.clear_data()
        except BaseException as e:
            response.success = False
            response.message = str(e)
        else:
            response.success = True
        return response

    def start_param_server(self):
        # self.pserver = ParamServer(start_port=25652, addr="0.0.0.0")

        post_params = {
            "host": ("database_host", "Host address of the database server."),
            "port": ("database_port", "Port on which the database server is running."),
            "database": ("database_name", "Name of the database (e.g. an SQL DB name)."),
            "baseURL": ("database_uri", "Ontology base string / base namespace."),
            "store": ("database_store", "Store type, can be alchemy or fuseki.")
        }

        parameters = []

        self.log("Creating server with params:")

        for k, v in self._cfg.items():

            if k not in post_params:
                continue

            parameter, description = post_params[k]

            descriptor = ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description=description
                )

            parameters.append((parameter, v, descriptor))

        if self.mode == "ros":
            try:
                declared_params: List[Parameter] = self.node.declare_parameters(
                    namespace=None,
                    parameters=parameters
                )
            except ParameterAlreadyDeclaredException as e:
                self.log(f"A parameter has already been declared!\n{e}")
                raise
            except InvalidParameterException as e:
                self.log(f"A parameter name is invalid!\n{e}")
                raise
            except InvalidParameterValueException as e:
                self.log(f"A registered callback rejects any parameter!\n{e}")
                raise
            except TypeError as e:
                self.log(f"A parameter does not match the annotated type!\n{e}")
                raise

            for param in declared_params:
                self.log(f"\t{param.name}: {param.value}")

            if self.ALLOW_CLIENTS_CLEARING_DATA:
                self.clearing_service = self.node.create_service(Trigger, "reset_database", self.clear_data_service_cb)

    @property
    def onto(self):
        return self._onto

    @property
    def mode(self):
        return self.__mode

    @property
    def node(self):
        if self.mode == "ros":
            return self._node

    def destroy(self):
        #if self.pserver is not None:
        #    self.pserver.destroy()

        if self.node is not None:
            self.node.destroy_node()

        self.backend.stop()


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--onto-path", "-p", default=None, type=str, help="Path to the ontology file to be loaded. If none specified, the path defined in this script will be used.")
    parser.add_argument("--do-not-clear", action="store_true", help="Do not clear the database on startup.")
    args, rest = parser.parse_known_args()
    if len(rest) > 0:
        print(f"These unknown args were provided: {rest}. If you are running this in ROS, there will be some ROS-related args. That is normal. Otherwise, check if you didn't misspell something.")
    CrowtologyServer.CLEAR_ON_START = not args.do_not_clear
    return args


def main_ros():
    args = parse_args()
    try:
        if args.onto_path is not None:
            cs = CrowtologyServer(mode="ros", base_onto_path=args.onto_path)
        else:
            cs = CrowtologyServer(mode="ros")
    except BaseException as e:
        print(f"Error during CrowtologyServer initialization:\n{e}")
    else:
        try:
            rclpy.spin(cs.node)
        except KeyboardInterrupt as ke:
            print("User requested shutdown.")
        finally:
            cs.destroy()


def main():  # run from console as standalone python script
    # TODO Process args
    cs = CrowtologyServer(mode="local")
    input("Hit enter to exit (makes sense, right?)...")  # so that the script does not exit
    cs.destroy()


if __name__ == "__main__":
    main()
