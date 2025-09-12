from math import sqrt, sin, cos
import rclpy

from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Point

class SimpleKinematics(Node):
    def __init__(self, node_name:str):
        super().__init__(node_name)
        self.create_subscription(Pose, '/turtle1/pose', self._sub_turtle1_pose_cb, 10)
        self.create_subscription(Pose, '/turtle2/pose', self._sub_turtle2_pose_cb, 10)

        self._turtle1_pose = Pose()

        self._turtle2_pose = Pose()

        self._tx = 0.0
        self._ty = 0.0

    def _sub_turtle1_pose_cb(self, msg:Pose):
        self._turtle1_pose = msg

        self._tx = self._turtle1_pose.x
        self._ty = self._turtle1_pose.y

        self.get_logger().info(f"Translation-Vector Turtle1: Tx={self._tx:.2f}, Ty={self._ty:.2f}%n")
        
    def _sub_turtle2_pose_cb(self, msg:Pose):
        self._turtle2_pose = msg

        self._tx = self._turtle2_pose.x - self._turtle1_pose.x
        self._ty = self._turtle2_pose.y - self._turtle1_pose.y

        self._theta = self._turtle2_pose.theta - self._turtle1_pose.theta
        self._td = sqrt(self._tx**2 + self._ty**2)

        rotation_matrix = [
            [cos(self._theta), -sin(self._theta)],
            [sin(self._theta), cos(self._theta)]]
        
        self.get_logger().info(f"Rotation-Matrix: {rotation_matrix}")
        self.get_logger().info(f"Translation-Vector Turtle2: Tx={self._tx:.2f}, Ty={self._ty:.2f}")
        self.get_logger().info(f"Distance Turtle1 to Turtle2: {self._td:.2f}")

def main():
    try:
        rclpy.init()
        node = SimpleKinematics("simple_kinematics")
        rclpy.spin(node)

    except KeyboardInterrupt as e:
        print("Sie haben STRG+C gedrückt!")

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()