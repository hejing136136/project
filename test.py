# from utils.fr5_robot_control import RobotControl
# import time

# robot = RobotControl("192.168.58.2")
# reset_pos = [90.000, -120.000, 120.000, -180.000, -90.000, -90.000]
# desc_pos1 = [160.000, -290.000, 600.000, -90.000, -90.000, 180.000]
# robot.reset(reset_pos)
# print(robot.getTCPPose())
# time.sleep(2)
# # robot.moveL(desc_pos1)
# # time.sleep(2)
# robot.stop()
# # import pyrealsense2 as rs

import rclpy 
from rclpy.node import Node
from std_msgs.msg import String

rclpy.init(args=None)
ros_node = Node("watermelon_detector_node")
publisher = ros_node.create_publisher(
    String,
    "/chatter",
    10
)


def publish_status(status):
    msg = String()
    msg.data = status
    publisher.publish(msg)
    last_status = status
    print(f"ROS Published: {status}")

if __name__ == '__main__':
    detected = None
    while True:
        flag = input()
        if flag == 'w':
            detected = True
        elif flag == 'n':
            detected = False

        if detected:
            publish_status('watermelon')
        else:
            publish_status("notwatermelon")

    ros_node.destroy_node()
    rclpy.shutdown()