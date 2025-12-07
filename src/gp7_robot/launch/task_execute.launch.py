import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
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
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
            '-Y', '0.0'
        ]
    )

    # --- Controller Spawners ---
    # Spawn joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager'
        ],
        output='screen'
    )

    # Spawn arm_controller
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager', '/controller_manager'
        ],
        output='screen'
    )

    # Spawn gripper_controller
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'gripper_controller',
            '--controller-manager', '/controller_manager'
        ],
        output='screen'
    )

    # --- Pick and Place Controller Node ---
    pick_place_controller_node = Node(
        package='gp7_robot',
        executable='pick_place_controller',
        name='pick_place_controller',
        output='screen',
        emulate_tty=True,
    )

    # --- Event Handlers for Sequential Loading ---
    # Step 1: Load joint_state_broadcaster after robot is spawned
    load_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot_node,
            on_exit=[
                TimerAction(
                    period=2.0,  # Wait 2 seconds after robot spawn
                    actions=[joint_state_broadcaster_spawner]
                )
            ]
        )
    )

    # Step 2: Load arm_controller after joint_state_broadcaster
    load_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                TimerAction(
                    period=1.0,  # Wait 1 second
                    actions=[arm_controller_spawner]
                )
            ]
        )
    )

    # Step 3: Load gripper_controller after arm_controller
    load_gripper_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[
                TimerAction(
                    period=1.0,  # Wait 1 second
                    actions=[gripper_controller_spawner]
                )
            ]
        )
    )

    # Step 4: Start pick and place controller after all controllers are loaded
    start_pick_place = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gripper_controller_spawner,
            on_exit=[
                TimerAction(
                    period=3.0,  # Wait 3 seconds for controllers to stabilize
                    actions=[pick_place_controller_node]
                )
            ]
        )
    )

    return LaunchDescription([
        use_sim_time_arg,
        gz_server,
        robot_state_publisher_node,
        static_tf_node,
        spawn_robot_node,
        load_joint_state_broadcaster,
        load_arm_controller,
        load_gripper_controller,
        start_pick_place,
    ])