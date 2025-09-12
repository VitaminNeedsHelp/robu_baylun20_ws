from rclpy.node import Node
import rclpy
from std_msgs.msg import Float32, String

class TempMonitorNode(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.declare_parameter('threshold', 27.0)

        self.create_subscription(Float32, 'temperature', self._sub_temp_callback, 10)

        self._pub_temp_alert = self.create_publisher(String, 'temp_alert', 10)


    def _sub_temp_callback(self, msg: Float32):
        threshold = self.get_parameter('threshold').get_parameter_value().double_value
        if msg.data > threshold:
            alert_msg = String()
            alert_msg.data = f"Warnung! Hohe Temperatur erkannt: {msg.data:.2f}°C"
            self._pub_temp_alert.publish(alert_msg)
            self.get_logger().warn(alert_msg.data)
        else:
            self.get_logger().info(f"Temperatur: {msg.data:.2f}°C")

    def destroy_node(self):
        return super().destroy_node()

def main():
    node = None
    try:
        rclpy.init()
        try:
            node = TempMonitorNode("temp_monitor")
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