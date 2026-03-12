from os.path import join
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

## configura spawing do gazebo 3D
def generate_launch_description():
    pkg_model_bringup = get_package_share_directory("model_bringup")
    pkg_model_description = get_package_share_directory("model_description")
    
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    yaw = LaunchConfiguration("yaw")
    namespace = LaunchConfiguration("robot_namespace")
    
    bridge_config = join(pkg_model_bringup, 'config', 'model_bridge.yaml')

    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([pkg_model_description, 'urdf', 'p3dx', 'pioneer3dx.xacro']),
        ' robot_namespace:=', namespace
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{'robot_description': robot_description_content}],
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        namespace=namespace,
        arguments=[
            '-topic', 'robot_description',
            '-name', 'pioneer3dx',
            '-allow_renaming', 'true',
            '-x', x, 
            '-y', y, 
            '-Y', yaw, 
            '-z', '0.2'
        ]
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        namespace=namespace,
        parameters=[{'config_file': bridge_config}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument("robot_namespace", default_value=""),
        
        robot_state_publisher,
        spawn_entity,
        bridge
    ])