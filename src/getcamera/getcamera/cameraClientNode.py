from rclpy.node import Node
import rclpy
from camerainterface.srv import Camera
from cv_bridge import CvBridge
import cv2

class cameraClientNode(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)
        self._client_camera = self.create_client(Camera,'get_photo')
        while not self._client_camera.wait_for_service(10.0):
            self.get_logger().warn("Warte auf Service get_photo")
        self.get_logger().info("Service get_photo verfügbar")

    def send_request(self):
        return self._client_camera.call_async(Camera.Request())

    def destroy_node(self):
        return super().destroy_node()

def main():
    node = None
    try:
        rclpy.init()
        try:
            node = cameraClientNode("camera_client_node")
        except Exception as e:
            print(f"Fehler beim Erstellen des Nodes: {e}")
            return
        future = node.send_request()
        rclpy.spin_until_future_complete(node,future)
        img = CvBridge().imgmsg_to_cv2(future.result().image)
        cv2.imshow("Mein Bild", img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        

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