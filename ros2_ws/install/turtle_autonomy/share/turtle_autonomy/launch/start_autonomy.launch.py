from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtle_autonomy',
            executable='odometry_node',
            name='odom_node'
        ),
        Node(
            package='turtle_autonomy',
            executable='autonomy_node',
            name='auto_node',
            parameters=[
                {'goal_x': 1.0},
                {'goal_y': 9.5}
            ]
        ),
    ])
