#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import SetLed
from functools import partial


class BatteryNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("battery") # MODIFY NAME
        self.last_change_time = self.get_current_time()
        self.low_on_battery = False
        self.get_logger().info("Battery node has been launched.")
        self.create_timer(0.1, self.callback_change_battery_charge)

    def get_current_time(self):
        secs, nanosecs = self.get_clock().now().seconds_nanoseconds()
        return secs + (nanosecs/1000000000.0)

    def callback_change_battery_charge(self):
        time_now = self.get_current_time()
        if (not(self.low_on_battery) and (time_now - self.last_change_time) > 4.0):
            self.low_on_battery = True
            self.get_logger().info("Battery is empty! Charging...")
            self.call_set_led(True, 3)
            self.last_change_time = time_now
        elif (self.low_on_battery and (time_now - self.last_change_time) > 6.0):
            self.low_on_battery = False
            self.get_logger().info("Battery charged!")
            self.call_set_led(False, 3)
            self.last_change_time = time_now
        

    def call_set_led(self, battery_low, led_number):
        client_ = self.create_client(SetLed, "set_led")
        while not client_.wait_for_service(1):
            self.get_logger().warn("Waiting for Server Set Led...")
        request = SetLed.Request()
        request.led_number = led_number
        request.led_state = battery_low

        future = client_.call_async(request)
        future.add_done_callback(
            partial(self.callback_set_led, led_number=led_number, led_state = battery_low))

    def callback_set_led(self, future, led_number, led_state):
        try:
            response = future.result()
            if (response):
                self.get_logger().info("Led has been updated.")
            else:
                self.get_logger().info("Led has NOT been updated.")
        except Exception as e:
            self.get_logger().error("Service call failed %r"%(e, ))

def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
