from rclpy.node import Node
import rclpy
from std_msgs.msg import Float32
from rclpy.parameter import Parameter
import random
from rcl_interfaces.msg import SetParametersResult

class TempSensorNode(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)
        self._pub_temperature = self.create_publisher(Float32, "temperature", 10)
        
        self.declare_parameter("hz", 2.0)
        self.declare_parameter("mean", 25.0)
        self.declare_parameter("amp", 5.0)


        self._hz = self.get_parameter("hz").get_parameter_value().double_value
        self._mean = self.get_parameter("mean").get_parameter_value().double_value
        self._amp = self.get_parameter("amp").get_parameter_value().double_value
        
        self.add_on_set_parameters_callback(self._on_param_set_cb)

        self._timer_temperature = self.create_timer(1.0/self._hz, self._timer_temperature_cb)
        

    def _on_param_set_cb(self, param: list[Parameter]):
        for p in param:
            if p.name == "hz":
                self._hz = p.value
                self.get_logger().info(f"Hz wurde auf {self._hz} gesetzt")
            elif p.name == "mean":
                self._mean = p.value
                self.get_logger().info(f"Mean wurde auf {self._mean} gesetzt")
            elif p.name == "amp":
                self._amp = p.value
                self.get_logger().info(f"Amp wurde auf {self._amp} gesetzt")
            else:
                self.get_logger().warn(f"Unbekannter Parameter: {p.name}")
        return SetParametersResult(successful=True)

    def _timer_temperature_cb(self):
        val = Float32()

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