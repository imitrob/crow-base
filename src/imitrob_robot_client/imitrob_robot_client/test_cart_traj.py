import rclpy
# from imitrob_robot_client.robot_client import RobotActionClient
from robot_client import RobotActionClient
from time import sleep


def run():
    rclpy.init()
    node = rclpy.create_node("test1")
    # node = None
    print("Init")
    racl = RobotActionClient(node)
    print("Created a robot action client")
    racl.start()



    racl._trajectory_skills.trajectory_cart(p = [[0.5, 0.1, 0.3]], q=[[1.0, 0.0, 0.0, 0.0]])

    # sleep(20)
    # print("then")
    # choose_and_run_action(racl, **args.__dict__)
    # print("end")
    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    run()
