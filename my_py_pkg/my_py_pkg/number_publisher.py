#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64

# Inherit from Node obj
class NumberPublisherNode(Node):
    def __init__(self):
        super().__init__("number_publisher")

        # declare parameter and set its default value. I can change that
        # from the terminal when launching the node:
        # ros2 run my_py_pkg number_publisher --ros-args -p number_to_publish:=5
        self.declare_parameter("number_to_publish", 4)
        self.declare_parameter("publish_frequency", 1.0)

        self.number_ = self.get_parameter("number_to_publish").value
        self.publish_frequency_ = self.get_parameter("publish_frequency").value

        # create_publisher(data_type_of_topic, topic_name, queue_size)
        self.publisher_ = self.create_publisher(Int64, "number", 10)
        self.timer_ = self.create_timer(1.0 / self.publish_frequency_, self.publish_number)
        # For debugging
        self.get_logger().info("Number Publisher has been started.")

    def publish_number(self):
        num = Int64()
        num.data = self.number_
        self.publisher_.publish(num)

def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()