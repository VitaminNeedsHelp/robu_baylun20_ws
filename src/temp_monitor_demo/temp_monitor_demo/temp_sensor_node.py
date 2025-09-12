from rclpy.node import Node
import rclpy
from std_msgs.msg import Float32
import random


class TempSensorNode(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)
        self._pub_temperature = self.create_publisher(Float32, 'temperature', 10)
        self.declare_parameter('hz', 2.0)
        self.declare_parameter('mean', 25.0)
        self.declare_parameter('amp', 5.0)

        self._hz = self.get_parameter('hz').get_parameter_value().double_value
        self._mean = self.get_parameter('mean').get_parameter_value().double_value
        self._amp = self.get_parameter('amp').get_parameter_value().double_value

        self.add_on_set_parameters_callback(self._on_param_cb)

        self._timer = self.create_timer(1.0 / self._hz, self._timer_temp_cb)

    def _on_param_cb(self, params):
        self.get_logger().info("Parameteränderung erkannt!")

    def _timer_temp_cb(self):
        val = Float32()

        self._hz = self.get_parameter('hz').get_parameter_value().double_value
        self._mean = self.get_parameter('mean').get_parameter_value().double_value
        self._amp = self.get_parameter('amp').get_parameter_value().double_value

        val.data = self._mean + random.uniform(-self._amp, self._amp)

        self._pub_temperature.publish(val)


    def destroy_node(self):
        return super().destroy_node()

def main():
    node = None
    try:
        rclpy.init()
        try:
            node = TempSensorNode("temp_sensor")
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