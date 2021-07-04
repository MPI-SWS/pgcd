import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch_ros.actions import Node

parameters_file = 'experiments/recovery_test_05/params.yaml'

def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'dummy1']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'dummy2']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'dummy3']
        ),
        Node(
            package='pgcd',
            executable='component.py',
            name='dummy1',
            output = 'screen',
            emulate_tty = True,
            parameters=[
                parameters_file
            ],
        ),
        Node(
            package='pgcd',
            executable='component.py',
            name='dummy2',
            output = 'screen',
            emulate_tty = True,
            parameters=[
                parameters_file
            ],
        ),
        Node(
            package='pgcd',
            executable='component.py',
            name='dummy3',
            output = 'screen',
            emulate_tty = True,
            parameters=[
                parameters_file
            ],
        )
    ])
