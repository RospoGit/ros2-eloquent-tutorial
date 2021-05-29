#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import Kill
from my_robot_interfaces.msg import AvailableTurtles
from geometry_msgs.msg import Twist
from functools import partial


class TurtleControllerNode(Node):  # MODIFY NAME
    def __init__(self):
        super().__init__("turtle_controller")  # MODIFY NAME

        # Set controlled turtle
        self.declare_parameter("controlled_turtle", "turtle1")
        self.controlled_turtle_ = self.get_parameter("controlled_turtle").value

        # Subscribe to controlled turtle pose
        self.controlled_turtle_pose_sub_ = self.create_subscription(
            Pose, str(self.controlled_turtle_)+"/pose", self.sub_get_controlled_turtle_pose, 10)
        self.controlled_pose_ = Pose()

        # Subscribe to list of available turtles
        self.available_turtles_sub_ = self.create_subscription(
            AvailableTurtles, "available_turtles", self.sub_get_target_pose, 10)
        self.target_pose_ = Pose()

        # Publish controlled turtle velocities
        self.cmd_vel_pub_ = self.create_publisher(
            Twist, str(self.controlled_turtle_)+"/cmd_vel", 10)
        self.commands_timer_ = self.create_timer(0.01, self.turtle_controller)

        # Call kill client
        self.check_if_caught_timer_ = self.create_timer(
            0.01, self.check_if_target_reached)
        self.target_name_ = ""

    def sub_get_controlled_turtle_pose(self, msg):
        self.controlled_pose_ = msg

    def sub_get_target_pose(self, msg):
        distance_from_controlled_turtle = []
        if not msg.name:
            self.get_logger().info("No available target.")
        else:
            for index, turtle in enumerate(msg.name):
                distance_from_controlled_turtle.append(
                    self.compute_distance(self.controlled_pose_.x, self.controlled_pose_.y,
                                          msg.x[index], msg.y[index]))
            minimum_distance = min(distance_from_controlled_turtle)
            minimum_index = distance_from_controlled_turtle.index(
                minimum_distance)
            self.target_pose_.x = msg.x[minimum_index]
            self.target_pose_.y = msg.y[minimum_index]
            self.target_name_ = msg.name[minimum_index]

    @staticmethod
    def compute_distance(x1, y1, x2, y2):
        deltaY = y2 - y1
        deltaX = x2 - x1
        distance = math.sqrt(math.pow(deltaX, 2)+math.pow(deltaY, 2))
        return distance

    @staticmethod
    def compute_angle(x1, y1, x2, y2):
        deltaY = y2 - y1
        deltaX = x2 - x1
        angle = math.atan2(deltaY, deltaX)
        return angle

    def turtle_controller(self):
        command_twist = Twist()
        if self.target_name_ != "":
            distance = self.compute_distance(self.controlled_pose_.x, self.controlled_pose_.y,
                                             self.target_pose_.x, self.target_pose_.y)
            angle = self.compute_angle(self.controlled_pose_.x, self.controlled_pose_.y,
                                       self.target_pose_.x, self.target_pose_.y)
            deltaAngle = angle - self.controlled_pose_.theta
            # Normalize
            if deltaAngle >= math.pi:
                deltaAngle -= 2*math.pi
            elif deltaAngle <= -math.pi:
                deltaAngle += 2*math.pi
            # Check if I have already reached it
            if distance < 0.1:
                command_twist.linear.x = 0.0
                command_twist.angular.z = 0.0
            else:
                command_twist.linear.x = 2*distance
                command_twist.angular.z = 6*deltaAngle
        else:
            command_twist.linear.x = 0.0
            command_twist.angular.z = 0.0
        self.cmd_vel_pub_.publish(command_twist)

    def call_kill_and_update_server(self, target):
        client = self.create_client(Kill, "kill_and_update")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server kill_and_update...")
        request = Kill.Request()
        request.name = target
        future = client.call_async(request)
        future.add_done_callback(
            partial(self.callback_kill_and_update, name=target))

    def callback_kill_and_update(self, future, name):
        try:
            response = future.result()
            self.get_logger().info("Turtle "+str(name)+" has been killed.")
        except Exception as e:
            self.get_logger().error("Service call to \kill_and_update failed %r" % (e,))

    def check_if_target_reached(self):
        if self.target_name_ != "":
            distance = self.compute_distance(self.controlled_pose_.x, self.controlled_pose_.y,
                                             self.target_pose_.x, self.target_pose_.y)
            if distance < 0.1:
                self.call_kill_and_update_server(self.target_name_)
                self.target_name_ = ""
        else:
            return


def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()  # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
