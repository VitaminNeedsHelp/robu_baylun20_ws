from rclpy.node import Node
import rclpy

import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
from scipy.spatial.transform import Rotation as R

class CameraToWorldTransformation(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)
        self._tf2_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self._tf2_buffer, self)

        self._timer_transform = self.create_timer(1.0, self._transform_camera_to_world)

    def _transform_camera_to_world(self):
        try:
            transform = self._tf2_buffer.lookup_transform("world", "camera_link", rclpy.time.Time())
            point_camera = PointStamped()
            point_camera.header.frame_id = "camera_link"
            point_camera.point.x = 0.1
            point_camera.point.y = 0.2
            point_camera.point.z = 0.3
            
            point_world=tf2_geometry_msgs.do_transform_point(point_camera, transform)

            self.get_logger().info(f"Weltkoordinaten von P-Camera nach P-Welt: x={point_world.point.x:.1f}, y={point_world.point.y:.1f}, z={point_world.point.z:.1f}")
        except tf2_ros.LookupException:
            self.get_logger().warn(f"Buffer nicht gefüllt!")
        except tf2_ros.ExtrapolationException:
            self.get_logger().warn(f"Transformationsausnahme aufgetreten!")


    def destroy_node(self):
        return super().destroy_node()

def main():
    node = None
    try:
        rclpy.init()
        try:
            node = CameraToWorldTransformation("camera_to_world")
        except Exception as e:
            print(f"Fehler beim Erstellen des Nodes: {e}")
            return

        rclpy.spin(node)

    except KeyboardInterrupt:
        print("Sie haben STRG+C gedrückt!")

    finally:
        if node is not None:
            if rclpy.ok():
                node.get_logger().info(f"Node {node.get_name()} wird beendet!")
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()