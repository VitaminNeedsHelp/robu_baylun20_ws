#Exercise Title:    Remote Control for the TurtleBot3 Burger
#Group:             ?
#Class:             ?
#Date:              ?

import rclpy #ROS2 Python API
from geometry_msgs.msg import Twist
import rclpy.logging
from rclpy.qos import QoSProfile #Quality of Service Profile

import os   #OS functions
import select 
import sys #System functions
import time 

import termios
import tty


msg = """
Excercise:  ?
Group:      ?
Class:      ?
Date:       ?
"""

e = """
Communications Failed
"""

#Physikalische Grenzen des Roboters
MAX_LIN_VEL = 2.2          #m/s
MAX_ANG_VEL = 4.8          #rad/s

LIN_VEL_STEP_SIZE = 0.1    #m/s
ANG_VEL_STEP_SIZE = 0.2     #rad/s

def get_key():
    old_settings = termios.tcgetattr(sys.stdin)
    ts = time.time()
    key = ''
    try:
        tty.setraw(sys.stdin.fileno())
        while True:
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                key += os.read(sys.stdin.fileno(), 1).decode("utf-8")
            else:
                break
        return key
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main():
    
    rclpy.init() #Initialize ROS2

    qos = QoSProfile(depth=10) #Quality of Service Profile with a queue depth of 10 messages
    
    node = rclpy.create_node('remotectrl') #Create a ROS2 node with the name 'remotectrl'
    pub = node.create_publisher(Twist, 'turtle1/cmd_vel', qos) #Create a publisher for the topic 'cmd_vel' with the message type Twist  

    vel = Twist()

    try:
        while(1):
            key = get_key()
            if key != '':
                str = "String: " + key.replace(chr(0x1B), '^') + ", Code:"
                for c in key:
                    str += " %d" % (ord(c))
                #print(str)

                if ord(key[0]) == 0x03:
                    print("CTRL-C, exiting...")
                    break
                elif key == chr(0x1B):
                    print("ESC")
                    vel.angular.x = 0.0
                    vel.angular.y = 0.0
                    vel.angular.z = 0.0
                    vel.linear.x = 0.0
                    vel.linear.y = 0.0
                    vel.linear.z = 0.0
                    node.get_logger().info("Robot stopped")
                elif key == "\x1b[A":
                    print("Forwards")
                    vel.linear.x += LIN_VEL_STEP_SIZE
                    if vel.linear.x > MAX_LIN_VEL:
                        vel.linear.x = MAX_LIN_VEL
                        node.get_logger().info("Forwad-Velocity at maximum")
                elif key == "\x1b[B":
                    print("Backwards")
                    vel.linear.x -= LIN_VEL_STEP_SIZE
                    if vel.linear.x < -MAX_LIN_VEL:
                        vel.linear.x = -MAX_LIN_VEL
                        node.get_logger().info("Backward-Velocity at maximum")
                elif key == "\x1b[C":
                    print("Right")
                    vel.angular.z -= ANG_VEL_STEP_SIZE
                    if vel.angular.z < -MAX_ANG_VEL:
                        vel.angular.z = -MAX_ANG_VEL
                        node.get_logger().info("Rotation right at maximum")
                elif key == "\x1b[D":
                    print("Left")
                    vel.angular.z += ANG_VEL_STEP_SIZE
                    if vel.angular.z > MAX_ANG_VEL:
                        vel.angular.z = MAX_ANG_VEL
                        node.get_logger().info("Rotation left at maximum")
                
                pub.publish(vel)

                    
    except Exception as e:
        print(e)

    finally:
        vel.angular.z = 0.0
        vel.linear.x = 0.0
        pub.publish(vel)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()