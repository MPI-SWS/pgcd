import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch_ros.actions import Node

parameters_file = 'xp_sorting/params.yaml'

def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'franka']
        ),
        Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'cart']
        ),
        Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'carrier']
        ),
        Node(
            package='pgcd',
            node_executable='component.py',
            node_name='franka',
            output = 'screen',
            emulate_tty = True,
            parameters=[
                parameters_file
            ],
        ),
        Node(
            package='pgcd',
            node_executable='component.py',
            node_name='arm',
            output = 'screen',
            emulate_tty = True,
            parameters=[
                parameters_file
            ],
        ),
        Node(
            package='pgcd',
            node_executable='component.py',
            node_name='carrier',
            output = 'screen',
            emulate_tty = True,
            parameters=[
                parameters_file
            ],
        ),
        Node(
            package='pgcd',
            node_executable='component.py',
            node_name='sensor',
            output = 'screen',
            emulate_tty = True,
            parameters=[
                parameters_file
            ],
        ),
        Node(
            package='pgcd',
            node_executable='component.py',
            node_name='cart',
            output = 'screen',
            emulate_tty = True,
            parameters=[
                parameters_file
            ],
        )
    ])
