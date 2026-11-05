import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (EnvironmentVariable, LaunchConfiguration,
                                  PathJoinSubstitution, PythonExpression)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml

# Hand-fitted map->arena for rb3302_map, used when there is no calibration
# file (see calibrate_arena).
FALLBACK_ARENA = {'arena_x': 0.380, 'arena_y': 1.643, 'arena_yaw': -2.031}


def arena_tf(context):
    """Publish map->arena from explicit args, the calibration file, or the
    hand-fitted fallback, in that order."""
    arena_file = LaunchConfiguration('arena_file').perform(context)
    if not arena_file:
        map_yaml = LaunchConfiguration('map').perform(context)
        arena_file = os.path.splitext(map_yaml)[0] + '_arena.yaml'
    values, source = dict(FALLBACK_ARENA), 'hand-fitted fallback'
    if os.path.isfile(arena_file):
        with open(arena_file) as f:
            values.update(yaml.safe_load(f) or {})
        source = arena_file
    for key in FALLBACK_ARENA:
        override = LaunchConfiguration(key).perform(context)
        if override:
            values[key], source = override, 'launch arguments'
    map_frame = LaunchConfiguration('map_frame').perform(context)
    return [
        LogInfo(msg=f'arena frame from {source}: x {values["arena_x"]} '
                    f'y {values["arena_y"]} yaw {values["arena_yaw"]}'),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='arena_tf_publisher',
            output='screen',
            arguments=[
                '--x', str(values['arena_x']),
                '--y', str(values['arena_y']),
                '--yaw', str(values['arena_yaw']),
                '--frame-id', map_frame,
                '--child-frame-id', LaunchConfiguration('arena_frame'),
            ],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
    ]


def turn_controller_node(context):
    """turn_rate_controller, with the base's measured turning correction
    from <map dir>/turn_calibration.yaml (see calibrate_turning) when that
    file exists; otherwise the node's built-in defaults."""
    map_yaml = LaunchConfiguration('map').perform(context)
    calib = os.path.join(os.path.dirname(map_yaml), 'turn_calibration.yaml')
    params = [{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    if os.path.isfile(calib):
        params.append(calib)
        source = calib
    else:
        source = 'built-in defaults (run calibrate_turning)'
    # The turn controller reads wall_repulsion's output (see the velocity
    # chain in generate_launch_description).
    turn_input = 'cmd_vel_repelled'
    return [
        LogInfo(msg=f'turn calibration from {source}'),
        Node(
            package='planning_rpp',
            executable='turn_rate_controller',
            name='turn_rate_controller',
            output='screen',
            parameters=params,
            remappings=[
                ('cmd_vel_smoothed', turn_input),
                ('cmd_vel', LaunchConfiguration('cmd_vel_topic')),
                ('odom', LaunchConfiguration('odom_topic')),
            ],
        ),
    ]


def generate_launch_description():
    share = get_package_share_directory('planning_rpp')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    autostart = LaunchConfiguration('autostart')
    base_frame = LaunchConfiguration('base_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    map_frame = LaunchConfiguration('map_frame')
    scan_topic = LaunchConfiguration('scan_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    raw_odom_topic = LaunchConfiguration('raw_odom_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    stiction_boost = LaunchConfiguration('stiction_boost')
    turn_controller = LaunchConfiguration('turn_controller')
    collision_monitor = LaunchConfiguration('collision_monitor')
    wall_repulsion = LaunchConfiguration('wall_repulsion')
    # Velocity chain: velocity_smoother -> [collision_monitor] ->
    # wall_repulsion -> turn_rate_controller / stiction_boost -> /cmd_vel.
    # wall_repulsion reads the collision monitor's output when it runs, else
    # the smoother's; with wall_repulsion:=false it passes commands through.
    repulsion_input_topic = PythonExpression([
        "'cmd_vel_safe' if '", collision_monitor,
        "'.lower() == 'true' else 'cmd_vel_nav_smoothed'"])
    set_initial_pose = LaunchConfiguration('set_initial_pose')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites={
                'use_sim_time': use_sim_time,
                'base_frame_id': base_frame,
                'odom_frame_id': odom_frame,
                'amcl.ros__parameters.global_frame_id': map_frame,
                'bt_navigator.ros__parameters.global_frame': map_frame,
                'local_costmap.local_costmap.ros__parameters.global_frame': odom_frame,
                'global_costmap.global_costmap.ros__parameters.global_frame': map_frame,
                'behavior_server.ros__parameters.global_frame': map_frame,
                'behavior_server.ros__parameters.local_frame': odom_frame,
                'robot_base_frame': base_frame,
                'odom_topic': odom_topic,
                'scan_topic': scan_topic,
                'topic': scan_topic,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution(
                [EnvironmentVariable('HOME'), 'rb3302_final_proj_code', 'student_ws', 'maps', 'rb3302_map.yaml'])),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [share, 'config', 'nav2_params.yaml'])),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution(
                [share, 'rviz', 'navigation.rviz'])),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('raw_odom_topic', default_value='/odom/raw'),
        DeclareLaunchArgument('imu_topic', default_value='/imu_data_ros'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument(
            'stiction_boost', default_value='true',
            description='Boost stalled in-place turns (false = pass-through); '
                        'only used when turn_controller is false'),
        DeclareLaunchArgument(
            # Off by default: its emergency stop zeroes every command,
            # pivots included, so a robot that stops with its front near a
            # wall cannot turn away and stays stuck until a spin recovery
            # (seen at square F, 2026-10-28). RPP's own collision check and
            # the costmaps still keep paths off walls.
            'collision_monitor', default_value='false',
            description='Emergency stop from the raw scan (true = on)'),
        DeclareLaunchArgument(
            'wall_repulsion', default_value='true',
            description='Steer away from walls within 5 cm of the robot, from '
                        'the live scan (false = pass-through)'),
        DeclareLaunchArgument(
            'turn_controller', default_value='true',
            description='Closed-loop yaw-rate control from the IMU gyro '
                        '(false = use stiction_boost instead)'),
        DeclareLaunchArgument('set_initial_pose', default_value='false'),
        DeclareLaunchArgument('initial_pose_x', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_yaw', default_value='0.0'),
        # Arena frame: origin at the SW inner corner of the 4x6 course (outer
        # corner of square S), X = East (A->F), Y = North (S->A). Measured by
        # calibrate_arena into <map>_arena.yaml next to the map; arena_x/y/yaw
        # override it.
        DeclareLaunchArgument('arena_frame', default_value='arena'),
        DeclareLaunchArgument(
            'arena_file', default_value='',
            description='map->arena calibration (default: <map>_arena.yaml)'),
        DeclareLaunchArgument('arena_x', default_value=''),
        DeclareLaunchArgument('arena_y', default_value=''),
        DeclareLaunchArgument('arena_yaw', default_value=''),
        OpaqueFunction(function=arena_tf),
        Node(
            package='planning_rpp',
            executable='odom_direction_fix',
            name='odom_direction_fix',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                ('odom_in', raw_odom_topic),
                ('odom_out', '/odom/raw_signed'),
                ('cmd_vel', cmd_vel_topic),
            ],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'frequency': 50.0,
                'sensor_timeout': 0.30,
                'two_d_mode': True,
                'transform_timeout': 0.10,
                'print_diagnostics': True,
                'publish_tf': True,
                'publish_acceleration': False,
                'map_frame': map_frame,
                'odom_frame': odom_frame,
                'base_link_frame': base_frame,
                'world_frame': odom_frame,
                # Wheel odometry with its direction fixed (see
                # odom_direction_fix): the base reports reversing as forward.
                'odom0': '/odom/raw_signed',
                'odom0_config': [
                    False, False, False,
                    False, False, False,
                    True, False, False,
                    False, False, False,
                    False, False, False,
                ],
                'odom0_queue_size': 10,
                'odom0_differential': False,
                'odom0_relative': False,
                'imu0': imu_topic,
                'imu0_config': [
                    False, False, False,
                    False, False, True,
                    False, False, False,
                    False, False, True,
                    False, False, False,
                ],
                'imu0_queue_size': 50,
                'imu0_differential': False,
                'imu0_relative': True,
                'imu0_remove_gravitational_acceleration': False,
                # No outlier rejection (robot_localization's default). With
                # a threshold of 5 the EKF threw away the gyro whenever the
                # robot started a fast pivot (a sudden ~1.2 rad/s against a
                # confident 0 estimate), so /odom read ~0 rad/s while the
                # robot spun, and the heading froze until it caught up.
            }],
            remappings=[('odometry/filtered', odom_topic)],
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[configured_params, {'yaml_filename': map_yaml_file}],
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
                configured_params,
                {
                    'set_initial_pose': set_initial_pose,
                    'initial_pose': {
                        'x': initial_pose_x,
                        'y': initial_pose_y,
                        'z': 0.0,
                        'yaw': initial_pose_yaw,
                    },
                },
            ],
            remappings=[('scan', scan_topic)],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': ['map_server', 'amcl'],
            }],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[configured_params],
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[configured_params],
            remappings=[('cmd_vel', 'cmd_vel_nav_raw'), ('odom', odom_topic)],
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[configured_params],
            remappings=[
                ('cmd_vel', 'cmd_vel_nav_raw'),
                ('cmd_vel_smoothed', 'cmd_vel_nav_smoothed'),
            ],
        ),
        Node(
            package='planning_rpp',
            executable='wall_repulsion',
            name='wall_repulsion',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'enabled': ParameterValue(wall_repulsion, value_type=bool),
                'base_frame': base_frame,
                'influence_dist': 0.05,
                # Real robot: 20 x 20 cm about its turning centre.
                'footprint_front': 0.10,
                'footprint_back': 0.10,
                'footprint_half_width': 0.10,
            }],
            remappings=[
                ('cmd_vel_in', repulsion_input_topic),
                ('cmd_vel_out', 'cmd_vel_repelled'),
                ('scan', scan_topic),
            ],
        ),
        OpaqueFunction(function=turn_controller_node,
                       condition=IfCondition(turn_controller)),
        Node(
            package='planning_rpp',
            executable='stiction_boost',
            name='stiction_boost',
            output='screen',
            condition=UnlessCondition(turn_controller),
            parameters=[{
                'use_sim_time': use_sim_time,
                'enabled': ParameterValue(stiction_boost, value_type=bool),
            }],
            remappings=[
                ('cmd_vel_smoothed', 'cmd_vel_repelled'),
                ('cmd_vel', cmd_vel_topic),
                ('odom', odom_topic),
            ],
        ),
        Node(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            name='collision_monitor',
            output='screen',
            condition=IfCondition(collision_monitor),
            parameters=[configured_params],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_collision_monitor',
            output='screen',
            condition=IfCondition(collision_monitor),
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': ['collision_monitor'],
            }],
        ),
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[configured_params],
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[configured_params],
            # Spin / back-up go through the velocity smoother and turn
            # controller like normal driving, so slow spin commands are not
            # lost in the base's ~0.4 rad/s pivot dead zone.
            remappings=[('cmd_vel', 'cmd_vel_nav_raw')],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[configured_params],
            remappings=[('odom', odom_topic)],
        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[configured_params],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': [
                    'planner_server',
                    'controller_server',
                    'smoother_server',
                    'behavior_server',
                    'bt_navigator',
                    'waypoint_follower',
                    'velocity_smoother',
                ],
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(rviz),
        ),
    ])
