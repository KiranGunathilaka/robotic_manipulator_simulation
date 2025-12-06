import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )

    # --- Paths ---

    desc_share = get_package_share_directory('gp7_robot')
    sim_share = get_package_share_directory('gazebo_sim')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    urdf_path = os.path.join(desc_share, 'urdf', 'gp7_robot.urdf')
    world_path = os.path.join(sim_share, 'worlds', 'gp7_world.sdf')

    # --- URDF / TF ---
    robot_description = ParameterValue(
        Command(['cat', ' ', urdf_path]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }]
    )

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_basefootprint_baselink',
        arguments=[
            '0', '0', '0',
            '0', '0', '0',
            'base_footprint',
            'base_link'
        ]
    )

    # --- Gazebo (ros_gz_sim) ---
    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r ', world_path]
        }.items()
    )

    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_gp7_robot',
        output='screen',
        arguments=[
            '-world', 'gp7_world',
            '-topic', 'robot_description',
            '-name', 'gp7_robot',
            '-x', '.0',
            '-y', '0.0',
            '-z', '0.0',
            '-Y', '0.0'
        ],
        respawn=True,
        respawn_delay=1.0
    )

    return LaunchDescription([
        use_sim_time_arg,
        gz_server,
        robot_state_publisher_node,
        static_tf_node,
        spawn_robot_node,
    ])
