from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    name_list = ["giskard", "bb8", "daneel", "jander","c3po"]
    node_list =[]

    for robot_name in name_list:
        node_list.append(Node(
            package="my_py_pkg",
            node_executable="robot_news_station",
            node_name="robot_news_station_"+robot_name,
            parameters=[
                {"robot_name": robot_name}
            ],
            # I should add this to print the DEBUG of the nodes after the launch from
            # the launch file. In FOXY I don't need those.
            output="screen",
            emulate_tty=True
        ))

    smartphone_node = Node(
        package="my_py_pkg",
        node_executable="smartphone",
        output="screen",
        emulate_tty=True
    )



    for node in node_list:
        ld.add_action(node)
    ld.add_action(smartphone_node)
    return ld