from os.path import join
from launch.conditions import IfCondition
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

## o arquivo main para subir o robô no gazebo 3D
def generate_launch_description():
    pkg_navigation = get_package_share_directory("phi_p3dx_navigation")
    pkg_description = get_package_share_directory("phi_p3dx_description")

    world_name = LaunchConfiguration("world_name")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    yaw = LaunchConfiguration("yaw")
    namespace = LaunchConfiguration("robot_namespace")
    use_rviz = LaunchConfiguration("use_rviz")

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(pkg_navigation, "launch", "bringup_env.launch.py")),
        launch_arguments={'world_name': world_name}.items()
    )
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(pkg_navigation, "launch", "bringup_spawn_gz.launch.py")),
        launch_arguments={
            'x': x,
            'y': y,
            'yaw': yaw,
            'robot_namespace': namespace
        }.items()
    )

    rviz_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(join(pkg_navigation, "launch", "bringup_rviz.launch.py")),
                condition=IfCondition(use_rviz),
                launch_arguments={
                    'robot_namespace': namespace,
                    'use_sim_time': 'true'
                }.items()
            )
        ]
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(pkg_navigation, "launch", "bringup_navigation.launch.py")),
        launch_arguments={
            'robot_namespace': namespace
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument("world_name", default_value="obstacle2x2"),
        DeclareLaunchArgument("robot_namespace", default_value=""),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.5"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        
        world_launch,
        robot_launch,
        rviz_launch,
        navigation_launch
    ])