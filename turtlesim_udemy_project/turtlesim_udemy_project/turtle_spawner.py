#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn, Kill
import random, math
from my_robot_interfaces.msg import AvailableTurtles
from functools import partial


class TurtleSpawnerNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("turtle_spawner") # MODIFY NAME

        # Call spawn server
        self.spawn_timer_ = self.create_timer(2.0, self.call_spawn_server)

        # List of available turtles
        self.available_turtles_ = AvailableTurtles()

        # Kill server (used to call kill service from turtlesim)
        self.kill_server_ = self.create_service(Kill, "kill_and_update", self.server_kill_and_update)

        # Publish list of available turtles
        self.avail_turtles_pub_ = self.create_publisher(AvailableTurtles, "available_turtles", 10)
        self.turtle_msg_timer_ = self.create_timer(0.1, self.publish_available_turtles)

    def call_spawn_server(self):
        client = self.create_client(Spawn, "spawn")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server spawn...")
        request = Spawn.Request()
        request.x, request.y, request.theta = self.generate_random_position(0, 11, 1, 1)
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_spawn, x=request.x, y=request.y))

    def callback_spawn(self, future, x, y):
        try:
            response = future.result()
            self.get_logger().info("Turtle "+response.name+" has been spawned at: "+str(x)+" "+str(y))
            self.add_turtle_to_list(response.name, x, y)
        except Exception as e:
            self.get_logger().error("Service call to \spawn failed %r"%(e,))

    @staticmethod
    def generate_random_position(min_value, max_value, step, degree_step):
        x = random.randrange(min_value, max_value, step)
        y = random.randrange(min_value, max_value, step)
        theta = random.randrange(0, 360, degree_step)
        theta = theta*2*math.pi/360
        if theta > math.pi:
            theta -= 2*math.pi
        return float(x), float(y), theta

    def add_turtle_to_list(self, name, x, y):
        self.available_turtles_.name.append(str(name))
        self.available_turtles_.x.append(x)
        self.available_turtles_.y.append(y)

    def remove_turtle_from_list(self, name):
        try:
            index = self.available_turtles_.name.index(str(name))
            del self.available_turtles_.name[index]
            del self.available_turtles_.x[index]
            del self.available_turtles_.y[index]
        except Exception as e:
            self.get_logger().info("Error: %r"%(e,))
            self.get_logger().info(str(self.available_turtles_.name))
        

    def server_kill_and_update(self, request, response):
        self.remove_turtle_from_list(request.name)
        self.call_turtlesim_kill_service(request.name)
        return response

    def call_turtlesim_kill_service(self, name):
        client = self.create_client(Kill, "kill")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server kill...")
        request = Kill.Request()
        request.name = name
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_kill, name=name))

    def callback_kill(self, future, name):
        try:
            response = future.result()
            self.get_logger().info(
                "Turtle "+str(name)+" has been killed.")
        except Exception as e:
            self.get_logger().error("Service call to \kill failed %r"%(e,))

    def publish_available_turtles(self):
        msg = AvailableTurtles()
        msg = self.available_turtles_
        self.avail_turtles_pub_.publish(msg)




def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawnerNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
