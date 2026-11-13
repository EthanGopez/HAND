from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
    RegisterEventHandler, TimerAction,
)
from launch.conditions import LaunchConfigurationEquals
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    vision_source = LaunchConfiguration('vision_source')
    max_speed = LaunchConfiguration('max_speed')
    mouse_gain = LaunchConfiguration('mouse_gain')

    # Simulated xArm 6 + MoveIt Servo (fake hardware, no robot IP, ever)
    servo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('xarm_moveit_servo'), 'launch', 'xarm_moveit_servo_fake.launch.py',
        ])),
        launch_arguments={'dof': '6'}.items(),
    )

    # Stop Servo while the ready trajectory executes, then restart it. Waiting
    # for the action result avoids competing hold commands and startup races.
    prepare = Node(package='hand_bringup', executable='prepare_sim', output='screen')
    ready_pose = TimerAction(period=6.0, actions=[prepare])

    vision = Node(
        package='hand_vision', executable='vision_node', output='screen',
        condition=LaunchConfigurationEquals('input_mode', 'hand'),
        parameters=[{'source': vision_source}],
    )

    teleop = Node(
        package='hand_teleop', executable='teleop_node', output='screen',
        condition=LaunchConfigurationEquals('input_mode', 'hand'),
        parameters=[{'max_speed': max_speed}],
    )

    # Select exactly one input source so mouse and hand commands cannot compete.
    # Both wait for the initial joint-space move out of the singular home pose.
    mouse = Node(
        package='hand_teleop', executable='mouse_teleop_node', output='screen',
        condition=LaunchConfigurationEquals('input_mode', 'mouse'),
        parameters=[{'max_speed': max_speed, 'mouse_gain': mouse_gain}],
    )

    def after_ready(event, context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='Simulation initialization failed'))]
        return [teleop, mouse]

    return LaunchDescription([
        DeclareLaunchArgument('input_mode', default_value='hand',
                              choices=['hand', 'mouse'],
                              description='Select the arm velocity input source'),
        DeclareLaunchArgument('vision_source', default_value='fake',
                              description="'fake' or the camera URL"),
        DeclareLaunchArgument('max_speed', default_value='0.05',
                              description='Teleop hard speed cap in m/s'),
        DeclareLaunchArgument('mouse_gain', default_value='0.001',
                              description='Meters of tool travel per mouse pixel'),
        servo,
        RegisterEventHandler(OnProcessExit(target_action=prepare, on_exit=after_ready)),
        ready_pose,
        vision,
    ])
