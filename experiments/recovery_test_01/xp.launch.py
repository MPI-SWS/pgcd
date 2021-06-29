import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch_ros.actions import Node

parameters_file = 'experiments/recovery_test_01/params.yaml'

def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'dummy']
        ),
        Node(
            package='pgcd',
            executable='component.py',
            name='dummy',
            output = 'screen',
            emulate_tty = True,
            parameters=[
                parameters_file
            ],
        )
    ])
