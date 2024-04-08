import rdflib
from rdflib import URIRef, BNode, Literal
from rdflib.term import Identifier
from rdflib.namespace import Namespace, RDF, RDFS, OWL, FOAF
from collections import defaultdict
from collections.abc import Iterable
from itertools import chain
from rdflib.extras.infixowl import classOrIdentifier
import pygraphviz as pgv
import yaml
from warnings import warn
import cv2
import os
import argparse
from owlrl import DeductiveClosure, OWLRL_Semantics
import re
import numpy as np
import hashlib


ONTO_IRI = "http://imitrob.ciirc.cvut.cz/ontologies/crow"
CROW = Namespace(f"{ONTO_IRI}#")


# if __name__ == '__main__':
#     # %%ArgParser
#     parser = argparse.ArgumentParser()
#     parser.add_argument("build_name")
#     parser.add_argument("--onto_file", "-o", default="onto_draft_03.owl")
#     # args = parser.parse_args(["build_dog.yaml"])
#     args = parser.parse_args(["crow_reasoning/build_snake_fixed.yaml", "-o", "crow_ontology/data/onto_draft_03.owl"])
#     # args = parser.parse_args(["sub_build_dog.yaml", "-o", "../onto_draft_03.owl"])
#     # os.chdir(r".\code\crow\ontology\assembly")
#     # os.chdir(r".\assembly")
#     # args = parser.parse_args()

#     # %%Initialization
#     # build_name = "build_dog.yaml"
#     build_name = args.build_name
#     # onto_file = "onto_draft_03.owl"
#     # onto_file = "dog_build.owl"
#     onto_file = args.onto_file
#     print(onto_file)
#     split_name_re = re.compile(r"([\w\/]+)\.?")
#     # split_name_re = re.compile(r".*\.(\w*)\.(\w*)")

#     # %%Load onto

#     onto = rdflib.Graph()
#     onto.load(onto_file)
#     onto.bind("crow", CROW)


class OntoBuilder(object):
    split_name_re = re.compile(r"([\w\/]+)\.?")

    def __init__(self, onto):
        self.onto = onto

    # %%Functions
    def getBaseNameAndNS(self, obj_name, build_name=None):
        if "." in obj_name or build_name is None:
            # if obj_name.count(".") > 1:
            *nspaces, obj_name = self.split_name_re.findall(obj_name)
        else:
            nspaces = []

        NS = Namespace(URIRef(ONTO_IRI) + ("/" + build_name if build_name is not None else "") + (("/" + "/".join(nspaces)) if len(nspaces) > 0 else "") + "#")
        return obj_name, NS

    # def getUnqualifiedName


    def getQualifiedName(self, obj_name, build_name=None):
        obj_name, BUILD = self.getBaseNameAndNS(obj_name, build_name)
        return BUILD[obj_name]


    def checkIValidType(self, onto_type):
        try:
            next(self.onto.triples((onto_type, RDF.type, OWL.Class)))
        except StopIteration as e:
            return False
        else:
            return True


    def checkConnectionObject(self, conn_name, obj_name, build_name):
        obj_name, BUILD = self.getBaseNameAndNS(obj_name, build_name)
        try:
            obj_onto_name, _, obj_type = next(self.onto.triples((BUILD[obj_name], RDF.type, None)))
        except StopIteration as e:
            warn(f"Object {obj_name} used in operation {conn_name} does not exist! Maybe you forgot to define it in 'objects' section?")
            return False
        else:
            superClasses = list(self.onto.transitive_objects(obj_type, RDFS.subClassOf))
            if not((CROW.WorkMaterial in superClasses) or (CROW.AssemblyGraph in superClasses)):
                warn(f"Object {obj_name} used in operation {conn_name} is not a sub-class of WorkMaterial or AssemblyGraph!")
                return False
            else:
                return True


    def checkRelationOperation(self, rel_type, op_name, build_name):
        op_name, BUILD = self.getBaseNameAndNS(op_name, build_name)
        try:
            _, _, op_type = next(self.onto.triples((BUILD[op_name], RDF.type, None)))
        except StopIteration as e:
            warn(f"Operation {op_name} used in relation of type {rel_type} does not exist! Maybe you forgot to define it in 'operations' section?")
            return False
        else:
            superClasses = list(self.onto.transitive_objects(op_type, RDFS.subClassOf))
            if CROW.AssemblyOperation not in superClasses:
                warn(f"Operation {op_name} used in relation of type {rel_type} is not a subclass of AssemblyOperation!")
                return False
            else:
                return True


    def checkOrderOperation(self, order_type, op_name, build_name):
        op_name, BUILD = self.getBaseNameAndNS(op_name, build_name)
        try:
            _, _, op_type = next(self.onto.triples((BUILD[op_name], RDF.type, None)))
        except StopIteration as e:
            warn(f"Operation {op_name} used in order hint of type {order_type} does not exist! Maybe you forgot to define it in 'operations' section?")
            return False
        else:
            superClasses = list(self.onto.transitive_objects(op_type, RDFS.subClassOf))
            if CROW.AssemblyOperation not in superClasses:
                warn(f"Operation {op_name} used in order hint of type {order_type} is not a subclass of AssemblyOperation!")
                return False
            else:
                return True


    def checkReference(self, rel_type, ref_name, build_name):
        ref_name, BUILD = self.getBaseNameAndNS(ref_name, build_name)
        try:
            _, _, ref_type = next(self.onto.triples((BUILD[ref_name], RDF.type, None)))
        except StopIteration as e:
            warn(f"Reference {ref_name} used in relation of type {rel_type} does not exist!")
            return False
        else:
            superClasses = list(self.onto.transitive_objects(ref_type, RDFS.subClassOf))
            if CROW.LocalizedThing not in superClasses:
                warn(f"Reference {ref_name} used in relation of type {rel_type} must be localized! (I.e. subclass of LocalizedThing)")
                return False
            else:
                return True


    def sub_clusterize(self, in_g, out_g, top_name):
        for cf in in_g.subgraphs():
            nodes = [f"{top_name}.{n}" for n in cf.nodes()]
            d = out_g.add_subgraph(nodes, name=cf.name)
            self.sub_clusterize(cf, d, top_name)
            for k, v in cf.node_attr.items():
                d.node_attr[k] = v
            for k, v in cf.edge_attr.items():
                d.edge_attr[k] = v
            for k, v in cf.graph_attr.items():
                d.graph_attr[k] = v


    # %%Build graph
    def buildGraph(self, build_name, recipe_name=None, isBaseBuild=None):
        # %%Load YAML
        base_filename, _ = os.path.splitext(build_name)
        _, base_name = os.path.split(base_filename)

        with open(build_name, "r") as f:
            recipe = yaml.safe_load(f)

        # initialize graph
        graph = pgv.AGraph(strict=False,
                        splines="ortho", ranksep=0.6)
        graph.node_attr['style'] = 'filled'

        # add recipe name
        assembly_name = recipe["assembly_name"]
        if isBaseBuild is None:
            isBaseBuild = recipe_name is None
        if recipe_name is None:
            recipe_name = assembly_name
        BUILD = Namespace(f"{ONTO_IRI}/{recipe_name}#")
        self.onto.bind(recipe_name.replace("/", "_"), BUILD)
        recipe_onto_name = BUILD[recipe_name]
        graph.add_node(recipe_name, shape="note", fillcolor="white")
        self.onto.add((recipe_onto_name, RDF.type, CROW.AssemblyGraph))

        # Add objects
        if "imports" in recipe:
            for sub_build, import_path in recipe["imports"].items():
                g, g_name, a_name, _ = self.buildGraph(import_path, recipe_name + "/" + sub_build)
                sub_name = g_name[g_name.find("/") + 1:].replace('/', '.')
                c = graph.add_subgraph(g, name=f'cluster_{sub_build}')
                for k, v in g.node_attr.items():
                    c.node_attr[k] = v
                for k, v in g.edge_attr.items():
                    c.edge_attr[k] = v
                for k, v in g.graph_attr.items():
                    c.graph_attr[k] = v
                h = hashlib.new("shake_256")
                h.update(a_name.encode())
                np.random.seed(int(h.hexdigest(4), 16))
                # color = "#" + h.hexdigest(3) + "33"
                color = hex(np.sum([x << (i * 8) for i, x in enumerate(np.random.randint(0, 255, (3, )))])).replace("0x", "#") + "22"
                c.graph_attr.update(style='filled', color=color)
                c.graph_attr.update(label=f'{sub_build} ({a_name})')

                for n in g.nodes():
                    if str(n) == g_name:
                        continue
                    new_nodename = f"{sub_build}.{n}"
                    c.add_node(new_nodename, **n.attr)
                    n = c.get_node(new_nodename)
                    # n.attr["label"] = f"{sub_build}.{n.attr['label']}"
                for e in g.edges():
                    u, v = [f"{sub_build}.{w}" for w in tuple(e)]
                    c.add_edge(u, v, **e.attr)
                for triple in self.onto.triples((self.getQualifiedName(g_name.replace("/", ".") + f".{g_name}"), None, None)):
                    self.onto.add((recipe_onto_name, triple[1], triple[2]))
                self.sub_clusterize(g, c, sub_build)

        print(f"Parsing build {recipe_name}")
        # Add objects
        if "objects" in recipe:
            for entity, props in recipe["objects"].items():
                node_type = props["type"]
                onto_type = CROW[node_type]
                onto_name = BUILD[entity]
                if self.checkIValidType(onto_type):
                    superClasses = list(self.onto.transitive_objects(CROW[props["type"]], RDFS.subClassOf))
                    if CROW.WorkMaterial in superClasses:
                        self.onto.add((onto_name, RDF.type, onto_type))
                        self.onto.add((recipe_onto_name, CROW.usesMaterial, onto_name))
                        if CROW.Workpiece in superClasses:
                            graph.add_node(entity, label=entity + f"\n{props['type']}", shape="box", fillcolor="azure2")
                        elif CROW.Consumable in superClasses:
                            graph.add_node(entity, label=entity + f"\n{props['type']}", shape="box", fillcolor="gray")
                    else:
                        warn("Type {node_type} for object {entity} is not a subclass of WorkMaterial!")
                else:
                    warn(f"Object class {node_type} for object {entity} not found in Ontology!")

        # Add connections
        if "operations" in recipe:
            for entity, props in recipe["operations"].items():
                node_type = props["type"]
                onto_type = CROW[node_type]
                onto_name = BUILD[entity]
                if self.checkIValidType(onto_type):
                    superClasses = list(self.onto.transitive_objects(onto_type, RDFS.subClassOf))
                    if CROW.AssemblyOperation in superClasses:
                        label = entity + f"\n{node_type[:node_type.index('Connection')]}"
                        graph.add_node(entity, label=label, shape="oval", fillcolor="lavender")
                        self.onto.add((onto_name, RDF.type, onto_type))
                        self.onto.add((recipe_onto_name, CROW.hasAssemblyElement, onto_name))
                        if CROW.AssemblyBinaryConnection in superClasses:
                            # if CROW.InsertConnection in superClasses:
                            #     a = props["shaft"]
                            #     b = props["hole"]
                            # else:
                            a = props["provider"]
                            b = props["consumer"]

                            # check onto
                            if self.checkConnectionObject(entity, a, recipe_name) and self.checkConnectionObject(entity, b, recipe_name):
                                self.onto.add((onto_name, CROW.affordanceProvider, self.getQualifiedName(a, recipe_name)))
                                self.onto.add((onto_name, CROW.affordanceConsumer, self.getQualifiedName(b, recipe_name)))
                                graph.add_edge(a, entity, dir="forward", arrowhead="onormal", arrowsize=0.7)
                                graph.add_edge(entity, b, dir="forward", arrowhead="onormal", arrowsize=0.7)
                        else:
                            warn("Cannot process other than binary connections, yet!")
                    else:
                        warn(f"Operation type {node_type} for {entity} is not a subclass of AssemblyOperation!")
                else:
                    warn(f"Operation type {node_type} for {entity} operation not found in Ontology!")

        # Add relations
        if "relations" in recipe:
            for i, props in enumerate(recipe["relations"]):
                node_type = props["type"]
                onto_type = CROW[node_type]
                if self.checkIValidType(onto_type):
                    name = str(next(self.onto.triples((onto_type, CROW.representation, None)))[2])
                    uname = name + str(i)
                    superClasses = list(self.onto.transitive_objects(onto_type, RDFS.subClassOf))
                    if CROW.Relation in superClasses:
                        onto_name = BUILD[uname]
                        graph.add_node(uname, label=name, shape="circle", color="red", fillcolor="indianred1")
                        self.onto.add((onto_name, RDF.type, onto_type))
                        self.onto.add((recipe_onto_name, CROW.definesRelation, onto_name))

                        if CROW.UnorderedRelation in superClasses:
                            operations = props["operations"]
                            for op in operations:
                                if self.checkRelationOperation(node_type, op, recipe_name):
                                    self.onto.add((onto_name, CROW.relatesOperation, self.getQualifiedName(op, recipe_name)))
                                    graph.add_edge(op, uname, color="red")

                        if CROW.RelationWithReference in superClasses:
                            ref = props["reference"]
                            if self.checkReference(node_type, ref, recipe_name):
                                graph.add_edge(ref, uname, color="red", style="dashed")
                                self.onto.add((onto_name, CROW.hasSpatialReference, self.getQualifiedName(ref, recipe_name)))
                    else:
                        warn(f"Relation of type {props['type']} is not a subclass of Relation!")
                else:
                    warn(f"Relation of type {props['type']} not found in Ontology!")

        # Add order_hints
        if "order_hints" in recipe:
            for i, props in enumerate(recipe["order_hints"]):
                node_type = props["type"]
                onto_type = CROW[node_type]
                if self.checkIValidType(onto_type):
                    name = str(next(self.onto.triples((CROW[props["type"]], CROW.representation, None)))[2])
                    uname = name + str(i)
                    superClasses = list(self.onto.transitive_objects(CROW[props["type"]], RDFS.subClassOf))
                    if CROW.Order in superClasses:
                        onto_name = BUILD[node_type + str(i)]
                        graph.add_node(uname, label=name, shape="square", color="darkgreen", fillcolor="honeydew")
                        self.onto.add((onto_name, RDF.type, onto_type))
                        self.onto.add((recipe_onto_name, CROW.definesOrder, onto_name))
                        if CROW.SequentialOrder in superClasses:
                            first = props["first"]
                            then = props["then"]
                            if self.checkOrderOperation(node_type, first, recipe_name) and self.checkOrderOperation(node_type, then, recipe_name):
                                arrowHead = "open"
                                graph.add_edge(first, uname, color="darkgreen", fillcolor="honeydew", arrowhead=arrowHead, dir="forward")
                                graph.add_edge(uname, then, color="darkgreen", fillcolor="honeydew", arrowhead=arrowHead, dir="forward")
                                self.onto.add((onto_name, CROW.firstOp, self.getQualifiedName(first, recipe_name)))
                                self.onto.add((onto_name, CROW.thenOp, self.getQualifiedName(then, recipe_name)))
                        else:  # assumes parallel or selection order
                            operations = props["operations"]
                            if all([self.checkOrderOperation(node_type, op, recipe_name) for op in operations]):
                                if CROW.ParallelOrder in superClasses:
                                    arrowHead = "open"
                                elif CROW.SelectionOrder in superClasses:
                                    arrowHead = "ediamond"
                                else:
                                    raise Exception(f"Unknown order hint type: {superClasses}")
                                for op in operations:
                                    graph.add_edge(uname, op, color="darkgreen", fillcolor="honeydew", arrowhead=arrowHead, dir="forward")
                                    self.onto.add((onto_name, CROW.ordersOperation, self.getQualifiedName(op, recipe_name)))
                    else:
                        warn(f"Unknown order hint type: {node_type}!")

                else:
                    warn(f"Order hint of type {props['type']} not found in Ontology!")

        # DeductiveClosure(OWLRL_Semantics).expand(onto)
        # %%Output

        # image_file = base_filename + ".png"
        # graph.layout(prog='dot')
        # graph.draw(image_file, prog="dot")

        return graph, recipe_name, assembly_name, base_filename
