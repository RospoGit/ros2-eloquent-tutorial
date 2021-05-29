#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# Inherit from Node obj
class MyNode(Node):
    def __init__(self):
        # Create and define the name
        super().__init__("py_test")
        self.get_logger().info("Hello ROS2")

        # Variable
        self.counter_= 0

        # Call a timer at 2hz aka 0.5 sec
        self.create_timer(0.5, self.timer_callback)


    def timer_callback(self):
        self.counter_ += 1
        self.get_logger().info("Hello timer: "+ str(self.counter_))

def main(args=None):
    # Initialize ros2 node communication
    rclpy.init(args=args)

    # Call the node created (ie constructor)
    node = MyNode()

    # Allow the node to remain alive and not return
    rclpy.spin(node)

    #shutdown the communication
    rclpy.shutdown()

if __name__=="__main__":
    main()