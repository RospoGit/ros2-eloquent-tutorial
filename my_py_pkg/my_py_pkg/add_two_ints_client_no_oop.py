#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

def main(args=None):
    rclpy.init(args=args)
    node = Node("add_two_ints_no_oop") # MODIFY NAME

    client = node.create_client(AddTwoInts, "add_two_ints")
    
    # I wait for the server to be ready. client.wait_for_service returns False if
    # timeout expires, True if the service is ready.
    while not client.wait_for_service(1.0):
        node.get_logger().warn("Waiting for Server Add Two Ints...")

    # AddTwoInts is a service type!
    request = AddTwoInts.Request()
    request.a = 3
    request.b = 5

    # call_async makes a service request and asyncronously get the result,
    # otw it could wait forever (aka deadlock).
    # It returns a future object, which is an obj that will be set in the future.
    # I continuously spin until I receive a response.
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    # future could contain some exception. So it's better to use try-catch
    try:
        response = future.result()
        node.get_logger().info(
            str(request.a)+"+"+str(request.b)+"="+str(response.sum))
    except Exception as e:
        node.get_logger().error("Service call failed %r"%(e,))

    rclpy.shutdown()


if __name__ == "__main__":
    main()
