import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch_ros.actions import Node

parameters_file = 'src/pgcd/launch/simple_example_params.yaml'

def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='pgcd',
            #node_namespace='dummy_sender',
            node_executable='component.py',
            node_name='dummy_sender',
            parameters=[
                parameters_file
            ],
        ),
        Node(
            package='pgcd',
            #node_namespace='dummy_receiver',
            node_executable='component.py',
            node_name='dummy_receiver',
            parameters=[
                parameters_file
            ],
        ),
        Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            arguments=['-1', '0', '0', '0', '0', '0', 'world', 'dummy_sender']
        ),
        Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            arguments=['1', '0', '0', '0', '0', '0', 'world', 'dummy_receiver']
        )
    ])

