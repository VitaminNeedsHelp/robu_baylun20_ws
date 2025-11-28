from rclpy.node import Node
from sensor_msgs.msg import Image
import rclpy
import os
import cv2
from cv_bridge import CvBridge
from camerainterface.srv import Camera


class cameraServiceNode(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)
        self._bridge = CvBridge()
        self.declare_parameter("resolution", [300,300])
        self._resolution = self.get_parameter("resolution").get_parameter_value().integer_array_value
        self.declare_parameter("device", 0)
        self._device = self.get_parameter("device").get_parameter_value().integer_value
        
        self._image_pub = self.create_publisher(Image,"/image_raw",10)
        self._srv_get_photo = self.create_service(Camera,'get_photo',self._srv_get_photo_cb)
        self._timer_photo = self.create_timer(10.0, self._timer_photo_cb)
    

    def _srv_get_photo_cb(self,request, response):
        if self._device > 0:
            self.get_logger().warn("Only device 0 is supported")
            return response
        image_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),"images", f"titanic.jpeg")
        img = cv2.imread(image_path)
        response.image = self._bridge.cv2_to_imgmsg(cv2.resize(img,self._resolution))
        response.msg = "Foto"
        self.get_logger().info(f"Service request answered: returned Photo")
        return response

    def _timer_photo_cb(self):
        if self._device > 0:
            self.get_logger().warn("Only device 0 is supported")
            return
        image_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),"images", f"titanic.jpeg")
        img = cv2.imread(image_path)
        self._image_pub.publish(self._bridge.cv2_to_imgmsg(img,encoding="bgr8"))
        self.get_logger().info(f"Published picture")

        
    def destroy_node(self):
        return super().destroy_node()

def main():
    node = None
    try:
        rclpy.init()
        try:
            node = cameraServiceNode("camera_service_node")
        except Exception as e:
            print(f"Error creating node: {e}")
            return

        rclpy.spin(node)

    except KeyboardInterrupt:
        print("You pressed CTRL+C!")

    finally:
        if node is not None:
            if rclpy.ok():
                node.get_logger().info(f"Node {node.get_name()} closing!")
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()