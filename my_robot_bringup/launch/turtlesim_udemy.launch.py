from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    controller_node = Node(
        package="turtlesim_udemy_project",
        node_executable="turtle_controller",
        parameters=[
                {"controlled_turtle": "turtle1"}
            ],
        output="screen",
        emulate_tty=True
    )
    
    spawner_node = Node(
        package="turtlesim_udemy_project",
        node_executable="turtle_spawner",
        output="screen",
        emulate_tty=True
    )

    turtlesim_node = Node(
        package="turtlesim",
        node_executable="turtlesim_node",
        output="screen",
        emulate_tty=True
    )

    ld.add_action(controller_node)
    ld.add_action(spawner_node)
    ld.add_action(turtlesim_node)
    return ld