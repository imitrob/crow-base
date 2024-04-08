
import numpy as np
import rclpy, time, sys
from rclpy.node import Node

from crow_msgs.msg import FilteredPose

from crow_ontology.crowracle_client import CrowtologyClient
import matplotlib.pyplot as plt
from rdflib import URIRef

REFRESH_RATE = 1.0

class DrawerProcessingNode(Node):
    """

    Args:
        Node (_type_): _description_
    """
    def __init__(self):
        super().__init__("drawer_processing_node")
        self.crowracle = CrowtologyClient(node=self)
        self.onto = self.crowracle.onto
        
    def update_drawer(self):
        objects = self.crowracle.getTangibleObjectsProps()
        
        drawer_socket_location = None
        drawer_socket_uri = None
        drawer_cabinet_location = None
        drawer_cabinet_uri = None

        for obj in objects:
            if "drawer" in str(obj['uri']):
                drawer_socket_location = obj['absolute_location']
                drawer_socket_uri = obj['uri']
            if "drawer_cabinet" in str(obj['uri']):
                drawer_cabinet_location = obj['absolute_location']
                drawer_cabinet_uri = obj['uri']

        if drawer_socket_uri is None:
            print("drawer not found in the scene!")
            return

        print(f"Drawer identified as: {drawer_socket_uri}")

        openness_level = self.decide(drawer_socket_location, drawer_cabinet_location)
        print(f"openness_level: {openness_level}")
        
        self.crowracle.update_drawer_openness_level(drawer_socket_uri, openness_level)
        print("drawer data updated")

        ## For testing
        # drawer_data = self.crowracle.read_drawer_openness_level()
        # print(f"DEBUG: returned drawer_data {drawer_data}")
        # found_drawer = None
        # found_drawer_opened = None
        # for drawer_name, drawer_opened in drawer_data:
        #     if drawer_socket_uri in str(drawer_name):
        #         found_drawer = drawer_name
        #         found_drawer_opened = drawer_opened
        # assert found_drawer is not None, "Drawer not found"
        # assert np.round(float(found_drawer_opened),3) == np.round(float(openness_level),3), f"{found_drawer_opened} != {openness_level}, Not successfull"
        # print("Successfull!")

    def decide(self, drawer_socket_loc, drawer_cabinet_loc):
        """Returns if drawer is opened (1.0) or closed (0.0) or between (0.-1.)

        Args:
            drawer_socket_loc (Pose or None): Pose of the socket of the drawer (read as ontology object)
            drawer_cabinet_loc (Pose or None): Pose of the shell of the drawer (read as ontology object)

        Returns:
            Float: 0.0 (Closed) - 1.0 (Opened)
        """    

        # if shell not detected - return Error
        if drawer_cabinet_loc is None:
            return -1.0
        # if socket not detected - return Closed (We assume drawer part is always on the scene)
        if drawer_socket_loc is None:
            return 0.0
        
        # Euclidean distance 
        diff_dist = np.linalg.norm(np.array(drawer_socket_loc) - np.array(drawer_cabinet_loc))
        return self.model_dist(diff_dist)

    @staticmethod
    def sigmoid(x, center=0.2, tau=40):
        return 1 / (1 + np.exp((center-x)*(tau)))

    def model_dist(self, dist):
        # Tuning
        CENTER = 0.1
        TAU = 50
        return self.sigmoid(dist, center=CENTER, tau=TAU)
        
    def model_plot(self):
        x = np.linspace(0, 0.5, 100)
        y = [100*self.model_dist(x_) for x_ in x]
        print(x)
        print(y)
        plt.plot(x,y)
        plt.xlabel("distance [m]")
        plt.ylabel("opened [%]")
        plt.grid()
        plt.show()    

    def test_assign_drawer_openness_level(self, openness_level: float):
        print()
        print("Test test_assign_drawer_openness_level started")
        assert isinstance(openness_level, float) and (0.0 <= openness_level <= 1.0)
        # Load any visible drawer (socket part)
        objects = self.crowracle.getTangibleObjects()

        drawer_socket_uri = None
        for obj in objects:
            if "drawer" in str(obj):
                drawer_socket_uri = obj
        
        assert drawer_socket_uri is not None, "Add drawer to the scene and try again"
        print(f"drawer identified as: {drawer_socket_uri}")
        # change the openness_level
        self.crowracle.update_drawer_openness_level(drawer_socket_uri, openness_level)

        # read the value back
        drawer_data = self.crowracle.read_drawer_openness_level()
        print(f"DEBUG: returned drawer_data {drawer_data}")
        found_drawer = None
        found_drawer_opened = None
        for drawer_name, drawer_opened in drawer_data:
            if drawer_socket_uri in str(drawer_name):
                found_drawer = drawer_name
                found_drawer_opened = drawer_opened
        assert found_drawer is not None, "Drawer not found"
        assert float(found_drawer_opened) == float(openness_level), f"{found_drawer_opened} != {openness_level}, Not successfull"
        print("Successfull!")

def main():
    rclpy.init()
    dpn = DrawerProcessingNode()
    while rclpy.ok():
        time.sleep(1/REFRESH_RATE)
        dpn.update_drawer()

if __name__ == '__main__':
    if len(sys.argv) > 1 and 'test' in sys.argv[1]:
        print("Running test: Model Plot")
        rclpy.init()
        dpn = DrawerProcessingNode()
        dpn.model_plot()
    elif len(sys.argv) > 1 and 'write_onto' in sys.argv[1]:
        print(f"Running test: Write openness_level to: {sys.argv[2]}")
        rclpy.init()
        dpn = DrawerProcessingNode()
        dpn.test_assign_drawer_openness_level(float(sys.argv[2]))
    else:
        main()
