import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch_ros.actions import Node

parameters_file = 'src/pgcd/launch/simple_example_params.yaml'

def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['-1', '0', '0', '0', '0', '0', 'world', 'dummy_sender']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['1', '0', '0', '0', '0', '0', 'world', 'dummy_receiver']
        ),
        Node(
            package='pgcd',
            executable='component.py',
            name='dummy_sender',
            output = 'screen',
            emulate_tty = True,
            parameters=[
                parameters_file
            ],
        ),
        Node(
            package='pgcd',
            executable='component.py',
            name='dummy_receiver',
            output = 'screen',
            emulate_tty = True,
            parameters=[
                parameters_file
            ],
        )
    ])

