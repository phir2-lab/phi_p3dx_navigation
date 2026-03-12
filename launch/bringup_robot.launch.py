from os.path import join
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

## configura spawing do robô real
def generate_launch_description():
    pkg_phi_aria = get_package_share_directory('phi_aria')

    port_arg = DeclareLaunchArgument(
        'port', default_value='192.168.1.11:10002',
        description='')

    namespace_arg = DeclareLaunchArgument(
        'robot_namespace', default_value='',
        description='pioneer3dx')

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='false',
        description='Abrir RViz2 automaticamente')

    port      = LaunchConfiguration('port')
    namespace = LaunchConfiguration('robot_namespace')

    phi_aria_node = Node(
        package='phi_aria',
        executable='phi_p3dx',
        name='phi_aria',
        namespace=namespace,
        output='screen',
        parameters=[{
            'port':       port,
            'odom_frame':       'odom',
            'base_link_frame':  'base_link',
            'sonar_frame':      'base_link',
            'laser_frame':      'lidar_link',
            'publish_aria_lasers': False,
        }]
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(pkg_navigation, "launch", "bringup_navigation.launch.py")),
        launch_arguments={
            'robot_namespace': namespace
        }.items()
    )

    
    return LaunchDescription([
        port_arg,
        namespace_arg,

        phi_aria_node,
        navigation_launch,
    ])
