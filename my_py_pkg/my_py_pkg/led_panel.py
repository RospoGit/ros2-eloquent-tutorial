#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import LedStates
from my_robot_interfaces.srv import SetLed


class LedPanelNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("led_panel") # MODIFY NAME
        self.declare_parameter("led_states", [False, False, False])
        self.led_states_ = self.get_parameter("led_states").value
        self.server_ = self.create_service(SetLed, "set_led", self.callback_set_led)
        self.publisher_ = self.create_publisher(LedStates, "led_states", 10)
        self.timer_ = self.create_timer(4.0, self.callback_led_states)
        self.get_logger().info("Led Panel node has been launched.")
        self.battery_state_ = LedStates()
        self.battery_state_.led_states = self.led_states_

    def callback_led_states(self):
        self.publisher_.publish(self.battery_state_)

    def callback_set_led(self, request, response):
        if (request.led_number >= 0 and request.led_number <= 3):
            self.battery_state_.led_states[request.led_number-1] = request.led_state
            response.success = True
        else:
            response.success = False
        self.callback_led_states()
        return response

def main(args=None):
    rclpy.init(args=args)
    node = LedPanelNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
