from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    world_path = os.path.join(
        get_package_share_directory('articubot_one'),
        'worlds',
        'obstacles.world'
    )

    robot_description_path = os.path.join(
        get_package_share_directory('articubot_one'),
        'urdf',
        'articubot.urdf'
    )

    return LaunchDescription([
        Node(
            package='ros_gz_sim',
            executable='gz_sim',
            arguments=['-r', world_path],
            output='screen'
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', 'my_bot', '-file', robot_description_path],
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'use_sim_time': True, 'robot_description': open(robot_description_path).read()}],
            output='screen'
        )
    ])
