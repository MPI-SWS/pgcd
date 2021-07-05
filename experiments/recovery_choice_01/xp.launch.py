import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch_ros.actions import Node

parameters_file = 'experiments/recovery_choice_01/params.yaml'

def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'crane'] # x y z yaw pitch roll
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'carrier']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'franka']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'cart']
        ),
        Node(
            package='pgcd',
            executable='component.py',
            name='crane',
            output = 'screen',
            emulate_tty = True,
            parameters=[
                parameters_file
            ],
        ),
        Node(
            package='pgcd',
            executable='component.py',
            name='carrier',
            output = 'screen',
            emulate_tty = True,
            parameters=[
                parameters_file
            ],
        ),
        Node(
            package='pgcd',
            executable='component.py',
            name='franka',
            output = 'screen',
            emulate_tty = True,
            parameters=[
                parameters_file
            ],
        ),
        Node(
            package='pgcd',
            executable='component.py',
            name='cart',
            output = 'screen',
            emulate_tty = True,
            parameters=[
                parameters_file
            ],
        ),
        Node(
            package='pgcd',
            executable='component.py',
            name='arm',
            output = 'screen',
            emulate_tty = True,
            parameters=[
                parameters_file #TODO frame
            ],
        )
    ])
