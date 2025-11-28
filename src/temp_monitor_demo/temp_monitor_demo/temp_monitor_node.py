from rclpy.node import Node
import rclpy
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float32, String

class TempMonitorNode(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.declare_parameter("threshold", 27.0)

        self.create_subscription(Float32, "temperature", self._sub_temperature_cb, 10)

        self.add_on_set_parameters_callback(self._on_param_set_cb)
        
        self._pub_temperature_alert = self.create_publisher(String, "temperature_alert", 10)

    def _on_param_set_cb(self, param: list[Parameter]):
        for p in param:
            if p.name == "threshold":
                self._threshold = p.value
                self.get_logger().info(f"Threshold wurde auf {self._threshold} gesetzt")
        return SetParametersResult(successful=True)
    

    
    def _sub_temperature_cb(self, msg:Float32):
        val_temp = msg.data

        self.get_logger().info(f"Aktuelle Temperatur: {val_temp:.2f}")
        
        if val_temp > self.get_parameter("threshold").get_parameter_value().double_value:
            self.get_logger().warn(f"Temperaturwert ist zu hoch: {val_temp:.2f}")
            str_alert = String()
            str_alert.data = "Temperaturalarm! Aktuelle Temperatur: {:.2f}".format(val_temp)
            self._pub_temperature_alert.publish(str_alert)

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