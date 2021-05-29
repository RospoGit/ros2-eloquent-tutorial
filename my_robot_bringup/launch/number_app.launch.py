from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # This is the node launcher
    ld = LaunchDescription()

    # We define the tuple which is used to rename the topic name
    remapping_topic_name = ("number","my_number")
    
    # Start node from launch file
    number_publisher_node = Node(
        package="my_py_pkg",
        # Number of the executable is not node name or python file name.
        # It can be seen in the setup.py file. In FOXY is just "executable" and "name"
        node_executable="number_publisher",
        # We can rename the node name (remapping)
        node_name="my_number_publisher",
        # Rename the topic name
        remappings=[
            #("number", "my_number")
            remapping_topic_name
        ],
        # Add parameters
        parameters=[
            {"number_to_publish": 8},
            {"publish_frequency": 5.0}
        ]
    )

    counter_node = Node(
        package="my_cpp_pkg",
        node_executable="number_counter",
        node_name="my_number_counter",
        remappings=[
            #("number", "my_number")
            remapping_topic_name,
            ("number_count", "my_number_count")
        ]
        
    ) 

    ld.add_action(number_publisher_node)
    ld.add_action(counter_node)
    return ld