from rclpy.node import Node
import rclpy
import cv2


def test_track_detection():
    frame = cv2.imread("src/awdbot/awdbot_line_follower/test/test-bild.png")
    # cv2.imshow("Foto alda", frame)
    # cv2.waitKey(0)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # cv2.imshow("50 Graustufen", gray)
    # cv2.waitKey(0)

    mask = cv2.inRange(gray, 0, 100)
    # cv2.imshow("Nobody cared till i put on the mask", mask)
    # cv2.waitKey(0)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        for contour in contours:
            if cv2.contourArea(contour) > 100:
                frame_cnt = cv2.drawContours(frame, [contour], 0, (0, 0, 255), 2)
                # cv2.imshow("WARTEEEE ICH MUSS NOCH MEINE CONTOUR VERBLENDEN", frame_cnt)
                # cv2.waitKey(0)
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                frame_cnt = cv2.circle(frame_cnt, (cx, cy), 10, (255, 0, 0), 2)

    cv2.imshow("Frame mit Konturen und Mittelpunkten", frame_cnt)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


class LineFollower(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)

    def destroy_node(self):
        return super().destroy_node()


def main():
    node = None
    try:
        rclpy.init()
        try:
            node = LineFollower("line_follower")
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
    test_track_detection()
