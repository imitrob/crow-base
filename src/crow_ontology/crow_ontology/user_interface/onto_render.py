import rclpy
from rclpy.node import Node
import curses
from curses.textpad import Textbox, rectangle
from crow_ontology.crowracle_client import CrowtologyClient
from rdflib.namespace import Namespace, RDF, RDFS, OWL, FOAF
from rdflib import URIRef, BNode, Literal, Graph
from rdflib.term import Identifier
import time
from npyscreen.wgmultiline import MultiLine
import npyscreen
import os
from threading import Thread
import numpy as np
import open3d as o3d
from crow_utils.crow_config import get_lib_location
from glob import iglob
import yourdfpy
from copy import deepcopy
import argparse


CROW = Namespace("http://imitrob.ciirc.cvut.cz/ontologies/crow#")


class ORenderer():

    def __init__(self, local_mode=False):
        super().__init__()
        self.crowracle = CrowtologyClient(local_mode=local_mode)
        self.onto = self.crowracle.onto

        obj_directory = get_lib_location("models/urdf")
        axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)

        self.models = {}
        for filename in iglob("*.urdf", root_dir=obj_directory):
            basename, ext = os.path.splitext(filename)
            if basename.endswith("_aff"):
                continue
            filename = os.path.join(obj_directory, filename)
            print(filename)
            # mesh = o3d.io.read_triangle_mesh(filename, True, True)
            urdf = yourdfpy.urdf.URDF.load(filename)
            mesh_name, trimesh = urdf.scene.geometry.popitem()
            print(mesh_name)
            mesh = trimesh.as_open3d
            # o3d.visualization.draw_geometries([mesh, axes])
            self.models[basename] = mesh

    def run(self):
        objects = self.crowracle.list_objects_with_pcl()

        found_models = []
        for obj in objects:
            if obj["det_name"] in self.models:
                model = deepcopy(self.models[obj["det_name"]])
                model.translate(obj["loc"])
                found_models.append(model)
            else:
                print(f"Did not find model for object: {obj}")

        if len(found_models) > 0:
            o3d.visualization.draw_geometries(found_models)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--local-mode", "--local", "-l", action="store_true", help="Start in local (non-ros) mode")
    args, rest = parser.parse_known_args()

    oren = ORenderer(local_mode=args.local_mode)
    try:
        oren.run()
    except KeyboardInterrupt:
        print("User requested shutdown.")


if __name__ == '__main__':
    main()
