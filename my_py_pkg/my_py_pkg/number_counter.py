#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
from std_srvs.srv import SetBool


# Inherit from Node obj
class NumberCounterNode(Node):
    def __init__(self):
        super().__init__("number_counter")
        self.counter = Int64()
        self.counter.data = 0
        # create_publisher(data_type_of_topic, topic_name, queue_size)
        self.publisher_ = self.create_publisher(Int64, "number_count", 10)
        self.subscriber_ = self.create_subscription(Int64, "number", self.count_number, 10)
        # self.timer_ = self.create_timer(0.5, self.publish_number)
        self.server_ = self.create_service(SetBool, "reset_counter", self.callback_reset_counter)
        # For debugging
        self.get_logger().info("Number Counter has been started")
    
    def count_number(self, num):
        self.counter.data += num.data
        # self.get_logger().info("Number Counter: "+str(self.counter.data))
        self.publisher_.publish(self.counter)
        self.get_logger().info("Counter: "+str(self.counter.data))

    def callback_reset_counter(self, request, response):
        if (request.data):
            self.counter.data = 0
            response.success = True
            response.message = "Counter has been initialized."
        else:
            response.success = False
            response.message = "Counter has NOT been initialized."
        return response

def main(args=None):
    rclpy.init(args=args)
    node = NumberCounterNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()