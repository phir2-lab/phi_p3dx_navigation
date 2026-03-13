from os.path import join
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

## configura spawing do ambiente 2d "mobilesim"
def generate_launch_description():
    pkg_description = get_package_share_directory('phi_p3dx_description')
    pkg_navigation = get_package_share_directory('phi_p3dx_navigation')

    map_file = join(pkg_description, 'map', 'obstacles.map')

    port_arg = DeclareLaunchArgument(
        'port', default_value='localhost:8101',
        description='')

    namespace_arg = DeclareLaunchArgument(
        'robot_namespace', default_value='',
        description='pioneer3dx')

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='')

    port      = LaunchConfiguration('port')
    namespace = LaunchConfiguration('robot_namespace')
    use_rviz  = LaunchConfiguration('use_rviz')

    mobilesim = ExecuteProcess(
        cmd=['MobileSim', '-m', map_file],
        output='screen',
    )

    phi_aria_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='phi_p3dx_aria',
                executable='phi_p3dx',
                name='phi_p3dx_aria',
                namespace=namespace,
                output='screen',
                parameters=[{
                    'port':                port,
                    'odom_frame':          'odom',
                    'base_link_frame':     'base_link',
                    'sonar_frame':         'base_link',
                    'laser_frame':         'lidar_link',
                    'publish_aria_lasers': True,
                }]
            ),
        ],
    )

    navigation = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    join(pkg_navigation, 'launch', 'bringup_navigation.launch.py')
                ),
                launch_arguments={
                    'robot_namespace': namespace
                }.items()
            ),
        ],
    )
    

    rviz_launch = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    join(pkg_navigation, 'launch', 'bringup_rviz.launch.py')
                ),
                condition=IfCondition(use_rviz),
                launch_arguments={
                    'robot_namespace': namespace,
                    'use_sim_time': 'false',
                }.items(),
            ),
        ],
    )

    return LaunchDescription([
        port_arg,
        namespace_arg,
        use_rviz_arg,

        mobilesim,
        phi_aria_node,
        navigation,
        rviz_launch,
    ])
