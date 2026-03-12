from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

## configura spawing the controller a ser usado
def generate_launch_description():
    namespace = LaunchConfiguration('robot_namespace')

    controller_node = Node(
        package='phi_p3dx_navigation',
        executable='wall_follow',
        name='wall_follow',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': True,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('robot_namespace', default_value=''),
        controller_node
    ])
