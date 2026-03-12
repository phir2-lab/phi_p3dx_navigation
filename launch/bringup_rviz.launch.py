from os.path import join
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

## configura spawing de rviz
def generate_launch_description():
    pkg_description = get_package_share_directory("phi_p3dx_description")

    urdf_file = join(pkg_description, "urdf", "p3dx", "pioneer3dx.xacro")

    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=join(pkg_description, "rviz", "rviz.rviz"),
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false"
    )

    namespace_arg = DeclareLaunchArgument(
        "robot_namespace",
        default_value="",
        description="pioneer3dx"
    )

    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("robot_namespace")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            {"use_sim_time": use_sim_time}
        ],
    )

    robot_state_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
            output='screen'
        )

    joint_state_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        rviz_config_arg,
        use_sim_time_arg,
        namespace_arg,
        robot_state_node,
        joint_state_node,
        rviz_node,
    ])