from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['$(find-pkg-share articubot_one)/urdf/articubot_one.urdf'],
        ),
        Node(
            package='gazebo_ros',
            executable='gzserver',
            output='screen',
            arguments=['-s', 'libgazebo_ros_factory.so'],
        ),
        Node(
            package='gazebo_ros',
            executable='gzclient',
            output='screen',
        )
    ])
