import rclpy
from rclpy.node import Node
import curses
from curses.textpad import Textbox, rectangle
from crow_ontology.crowracle_client import CrowtologyClient
from rdflib.namespace import Namespace, RDF, RDFS, OWL, FOAF
from rdflib import URIRef, BNode, Literal, Graph
from rdflib.term import Identifier
import time

CROW = Namespace("http://imitrob.ciirc.cvut.cz/ontologies/crow#")


class OntoTester(Node):

    def __init__(self, node_name="onto_tester"):
        super().__init__(node_name)
        self.crowracle = CrowtologyClient(node=self)
        self.onto = self.crowracle.onto
        # self.get_logger().info(self.onto)

    def test_object(self):
        self.get_logger().info("Making an object")
        time.sleep(7)
        o = self.onto.makeEntity(CROW.Cube, {CROW.hasColor: CROW.COLOR_GREEN, CROW.x: Literal(13)})
        self.get_logger().info(f"Object has color: {o.hasColor}")
        self.get_logger().info("Changing color to red.")
        o.hasColor = CROW.COLOR_RED
        self.get_logger().info(f"Object has color: {o.hasColor}")
        self.get_logger().info("=====")
        self.get_logger().info(f"Object has x: {o.x}")
        self.get_logger().info("Changing x.")
        o.x = Literal(77)
        self.get_logger().info(f"Object has x: {o.x}")

        self.get_logger().info("=====")
        print(o)
        for s, p, o in o.list:
            self.get_logger().info(f"Property {p} has value {o}")

    def start(self):
        self.get_logger().info("Initialize!!!")
        stdscr = curses.initscr()
        curses.noecho()
        stdscr.addstr(0, 0, "Enter IM message: (hit Ctrl-G to send)")
        editwin = curses.newwin(5,30, 2,1)

        rectangle(stdscr, 1,0, 1+5+1, 1+30+1)
        stdscr.refresh()

        box = Textbox(editwin)

        # Let the user edit until Ctrl-G is struck.
        box.edit()

        # Get resulting contents
        message = box.gather()
        curses.endwin()

        self.get_logger().info("And the message is:")
        self.get_logger().info(message)


def main():
    rclpy.init()
    ot = OntoTester()
    ot.test_object()
    rclpy.spin(ot)


if __name__ == "__main__":
    main()
