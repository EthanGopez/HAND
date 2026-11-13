"""Mouse-only Servo control of a physical, unprefixed Lite6."""

import ipaddress
import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder
from uf_ros_lib.uf_robot_utils import generate_ros2_control_params_temp_file, load_yaml


def launch_setup(context):
    robot_ip = LaunchConfiguration('robot_ip').perform(context)
    # Validate before constructing any action that can connect to hardware.
    if robot_ip != 'auto':
        address = ipaddress.IPv4Address(robot_ip)
        if address.is_unspecified or address.is_multicast or address.is_loopback:
            raise ValueError('robot_ip must be the physical controller IPv4 address')
    settings = {}
    for name in ('max_speed', 'mouse_gain'):
        value = float(LaunchConfiguration(name).perform(context))
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f'{name} must be finite and greater than zero')
        settings[name] = value

    if robot_ip == 'auto':
        raise ValueError('This Lite6 launch requires an explicit robot_ip; '
                         'the existing discovery helper identifies xArm 6 only')

    controller_file = generate_ros2_control_params_temp_file(
        os.path.join(get_package_share_directory('xarm_controller'),
                     'config', 'lite6_controllers.yaml'),
        update_rate=150, use_sim_time=False, robot_type='lite',
    )
    config = MoveItConfigsBuilder(
        context=context, dof=6, robot_type='lite', robot_ip=robot_ip,
        controllers_name='controllers', limited=True,
        ros2_control_plugin='uf_robot_hardware/UFRobotSystemHardware',
        ros2_control_params=controller_file,
    ).to_moveit_configs()
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('xarm_controller'), 'launch',
            '_ros2_control.launch.py',
        )),
        launch_arguments={
            'robot_description': yaml.safe_dump(config.robot_description),
            'ros2_control_params': controller_file,
        }.items(),
    )
    state = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen', parameters=[config.robot_description, {'use_sim_time': False}],
    )
    servo_config = load_yaml('xarm_moveit_servo', 'config/xarm_moveit_servo_config.yaml')
    servo_config.update({
        'use_gazebo': False,
        'move_group_name': 'lite6',
        'command_out_topic': '/lite6_traj_controller/joint_trajectory',
    })
    servo = Node(
        package='moveit_servo', executable='servo_node_main', name='servo_server',
        output='screen', parameters=[
            config.robot_description, config.robot_description_semantic,
            config.robot_description_kinematics, config.joint_limits,
            {'moveit_servo': servo_config, 'use_sim_time': False},
        ],
    )
    spawners = [Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=[name, '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '90'],
    ) for name in ('joint_state_broadcaster', 'lite6_traj_controller')]
    ready = Node(package='hand_bringup', executable='prepare_hardware', output='screen',
                 parameters=[{'use_sim_time': False}])
    watchdog = Node(
        package='hand_bringup', executable='hardware_watchdog', output='screen',
        parameters=[{'use_sim_time': False}],
    )
    mouse = Node(
        package='hand_teleop', executable='mouse_teleop_node', output='screen',
        parameters=[config.robot_description,
                    dict(settings, use_sim_time=False, feedback_topic='/xarm/joint_states')],
    )
    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        condition=IfCondition(LaunchConfiguration('show_rviz')),
        arguments=['-d', os.path.join(get_package_share_directory('hand_bringup'),
                                     'config', 'mouse.rviz')],
        parameters=[config.robot_description, {'use_sim_time': False}],
    )

    def after_ready(event, context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='Hardware readiness check failed'))]
        return [watchdog, mouse]

    def stop_stack(event, context):
        return [EmitEvent(event=Shutdown(reason='Mouse control, watchdog, or Servo exited'))]

    def spawner_exit(event, context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='Hardware controller failed to start'))]
        return []

    handlers = [
        RegisterEventHandler(OnProcessExit(target_action=ready, on_exit=after_ready)),
        RegisterEventHandler(OnProcessExit(target_action=mouse, on_exit=stop_stack)),
        RegisterEventHandler(OnProcessExit(target_action=watchdog, on_exit=stop_stack)),
        RegisterEventHandler(OnProcessExit(target_action=servo, on_exit=stop_stack)),
    ] + [RegisterEventHandler(OnProcessExit(target_action=node, on_exit=spawner_exit))
         for node in spawners]
    return handlers + [control, state, servo, *spawners, ready, rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_ip',
                              description='Physical Lite6 controller IPv4 address'),
        DeclareLaunchArgument('max_speed', default_value='0.05',
                              description='Maximum total mouse tool speed in m/s'),
        DeclareLaunchArgument('mouse_gain', default_value='0.0001',
                              description='Meters of tool travel per mouse pixel'),
        DeclareLaunchArgument('show_rviz', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=launch_setup),
    ])
