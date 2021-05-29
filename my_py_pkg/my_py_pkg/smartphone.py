#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Inherit from Node obj
class SmartphoneNode(Node):
    def __init__(self):
        super().__init__("smartphone")
        # create_subscription(data_type_of_topic, topic_name, Callback, queue_size)
        # The callback is created only AFTER the spin. In the first run only the node of the creation
        # of the node is done
        self.subscriber_ = self.create_subscription(String, "robot_news", self.callback_robot_news, 10)
        self.get_logger().info("smartphone has been started.")

    def callback_robot_news(self, msg):
        self.get_logger().info(msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = SmartphoneNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()