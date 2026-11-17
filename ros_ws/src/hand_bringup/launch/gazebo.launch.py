"""Mouse- or CSV-controlled xArm 6 in Gazebo Classic, one controller manager.

Gazebo is the first stage where commanded tool velocity and achieved tool
velocity can diverge, because the arm has inertia and the controller has to
track a trajectory rather than teleport to it.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder
from uf_ros_lib.uf_robot_utils import generate_ros2_control_params_temp_file, load_yaml


def launch_setup(context):
    # Resolve our viewer option before the included vendor launch sets its own
    # show_rviz=false. Included launch configurations otherwise share scope.
    show_rviz = LaunchConfiguration('show_rviz').perform(context)
    input_mode = LaunchConfiguration('input_mode').perform(context)
    # Hard-code the simulation hardware plugin. Gazebo owns ros2_control;
    # launching the fake Servo stack here would create a competing manager.
    controller_file = generate_ros2_control_params_temp_file(
        os.path.join(get_package_share_directory('xarm_controller'),
                     'config', 'xarm6_controllers.yaml'),
        update_rate=150, use_sim_time=True, robot_type='xarm',
    )
    config = MoveItConfigsBuilder(
        context=context, dof=6, robot_type='xarm',
        ros2_control_plugin='gazebo_ros2_control/GazeboSystem',
        ros2_control_params=controller_file,
    ).to_moveit_configs()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('xarm_gazebo'), 'launch',
            '_robot_beside_table_gazebo.launch.py',
        )),
        launch_arguments={
            'dof': '6', 'robot_type': 'xarm', 'gz_type': 'gazebo',
            'moveit_config_dump': yaml.safe_dump(config.to_dict()),
            'load_controller': 'true', 'show_rviz': 'false',
        }.items(),
    )

    # Match the fake demo's topic names, but timestamp all ROS messages with
    # Gazebo's /clock. Mouse speed itself uses wall time, so pausing Gazebo
    # cannot cause division by zero or distort the user's drag measurement.
    servo_config = load_yaml('xarm_moveit_servo', 'config/xarm_moveit_servo_config.yaml')
    servo_config.update({
        'use_gazebo': True,
        'move_group_name': 'xarm6',
        'command_out_topic': '/xarm6_traj_controller/joint_trajectory',
    })
    servo = Node(
        package='moveit_servo', executable='servo_node_main', name='servo_server',
        output='screen', parameters=[
            config.robot_description, config.robot_description_semantic,
            config.robot_description_kinematics, config.joint_limits,
            {'moveit_servo': servo_config, 'use_sim_time': True},
        ],
    )
    prepare = Node(
        package='hand_bringup', executable='prepare_sim', output='screen',
        parameters=[{'use_sim_time': True}],
    )
    mouse = Node(
        package='hand_teleop', executable='mouse_teleop_node', output='screen',
        parameters=[config.robot_description, {
            'use_sim_time': True,
            'max_speed': LaunchConfiguration('max_speed'),
            'mouse_gain': LaunchConfiguration('mouse_gain'),
        }],
    )
    # Resolve the CSV path here so a typo fails the launch rather than leaving
    # a node waiting for a file that will never appear.
    csv_path = os.path.abspath(LaunchConfiguration('csv_path').perform(context))
    if input_mode == 'csv' and not os.path.isdir(os.path.dirname(csv_path)):
        raise ValueError(f'csv_path directory does not exist: {csv_path}')
    csv_vision = Node(
        package='hand_vision', executable='palm_csv_node', output='screen',
        parameters=[{
            'csv_path': csv_path,
            'replay': LaunchConfiguration('csv_replay'),
            'use_sim_time': True,
        }],
    )
    # Sample timing comes from the tracker's own clock inside the message, and
    # staleness from wall time, so pausing Gazebo cannot distort either. A pause
    # freezes joint feedback stamps, which the guard rejects as stale.
    csv_teleop = Node(
        package='hand_teleop', executable='palm_teleop_node', output='screen',
        parameters=[config.robot_description, {
            'use_sim_time': True,
            'max_speed': LaunchConfiguration('max_speed'),
            'scale_y': LaunchConfiguration('scale_y'),
            'scale_z': LaunchConfiguration('scale_z'),
            'scale_x': LaunchConfiguration('scale_x'),
        }],
    )
    # Exactly one input source, so two publishers cannot fight over the arm.
    selected = [mouse] if input_mode == 'mouse' else [csv_vision, csv_teleop]
    # RViz displays Gazebo's measured joint states through robot_state_publisher.
    # It remains usable on desktops where Gazebo Classic's native client hangs.
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        condition=IfCondition(show_rviz),
        arguments=['-d', os.path.join(
            get_package_share_directory('hand_bringup'), 'config', 'mouse.rviz',
        )],
        parameters=[config.robot_description, {'use_sim_time': True}],
    )

    def after_ready(event, context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='Gazebo initialization failed'))]
        return selected

    return [
        RegisterEventHandler(OnProcessExit(target_action=prepare, on_exit=after_ready)),
        gazebo, servo, prepare, rviz,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('input_mode', default_value='mouse',
                              choices=['mouse', 'csv'],
                              description='Select the arm velocity input source'),
        DeclareLaunchArgument('max_speed', default_value='0.25',
                              description='Maximum total tool speed in m/s'),
        DeclareLaunchArgument('mouse_gain', default_value='0.001',
                              description='Meters of tool travel per mouse pixel'),
        DeclareLaunchArgument('csv_path', default_value='/workspace/palm_center.csv',
                              description='Palm CSV written by hand_tracking_demo.py'),
        DeclareLaunchArgument('csv_replay', default_value='false',
                              choices=['true', 'false'],
                              description='Replay the whole CSV instead of '
                                          'following new rows only'),
        DeclareLaunchArgument('scale_y', default_value='0.01',
                              description='Tool metres per cm of hand motion across frame; 0.01 is 1:1'),
        DeclareLaunchArgument('scale_z', default_value='0.01',
                              description='Tool metres per cm of hand motion up the frame; 0.01 is 1:1'),
        DeclareLaunchArgument('scale_x', default_value='0.0',
                              description='Tool metres per cm of hand distance from the camera; '
                                          '0 disables depth, 0.01 tracks 1:1'),
        DeclareLaunchArgument('gui', default_value='false', choices=['true', 'false'],
                              description='Also show the native Gazebo client window'),
        DeclareLaunchArgument('show_rviz', default_value='true', choices=['true', 'false'],
                              description='Show live Gazebo joint feedback in RViz'),
        OpaqueFunction(function=launch_setup),
    ])
