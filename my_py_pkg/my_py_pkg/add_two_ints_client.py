#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
from functools import partial

class AddTwoIntsClientNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("add_two_ints_client") # MODIFY NAME
        self.call_add_two_ints_server(6,7)
        self.call_add_two_ints_server(2,3)


    # FROM HERE, WE CAN ALWAYS USE THE SAME CODE FOR THE SERVICES
    # We don't put the service in the __init__ but we create a new function
    # so that we can call it multiple times
    def call_add_two_ints_server(self, a, b):
        client = self.create_client(AddTwoInts, "add_two_ints")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Add Two Ints...")
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        future = client.call_async(request)
        # When the future is complete, callback is called. We have to do like this, 
        # because we can't spin here (we already spin in the main)
        # NB: The callback is called with the Future object as its only argument.
        # Partial(func, specific_param) set the param of the func to specific_parm
        # (it goes from left to right OR I can explicitly assign them)
        future.add_done_callback(partial(self.callback_call_add_two_ints, a=a, b=b))
    
    def callback_call_add_two_ints(self, future, a, b):
        try:
            response = future.result()
            self.get_logger().info(
                str(a)+"+"+str(b)+"="+str(response.sum))
        except Exception as e:
            self.get_logger().error("Service call failed %r"%(e,))

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsClientNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
