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
    csv_path = LaunchConfiguration('csv_path')
    csv_replay = LaunchConfiguration('csv_replay')
    scale_y = LaunchConfiguration('scale_y')
    scale_z = LaunchConfiguration('scale_z')
    scale_x = LaunchConfiguration('scale_x')

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
        parameters=[{'max_speed': max_speed, 'mouse_gain': mouse_gain,
                     'allow_recovery': LaunchConfiguration('allow_recovery')}],
    )

    # CSV input: the tracking demo (or a recording of it) supplies the hand
    # positions, so nothing here opens a camera. Two nodes, same split as the
    # live hand mode: one reads positions, one turns them into velocities.
    palm_vision = Node(
        package='hand_vision', executable='palm_csv_node', output='screen',
        condition=LaunchConfigurationEquals('input_mode', 'csv'),
        parameters=[{'csv_path': csv_path, 'replay': csv_replay}],
    )

    palm_teleop = Node(
        package='hand_teleop', executable='palm_teleop_node', output='screen',
        condition=LaunchConfigurationEquals('input_mode', 'csv'),
        parameters=[{'max_speed': max_speed, 'scale_y': scale_y,
                     'scale_z': scale_z, 'scale_x': scale_x,
                     'allow_recovery': LaunchConfiguration('allow_recovery')}],
    )

    def after_ready(event, context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='Simulation initialization failed'))]
        return [teleop, mouse, palm_vision, palm_teleop]

    return LaunchDescription([
        DeclareLaunchArgument('allow_recovery', default_value='false',
                              choices=['true', 'false'],
                              description='Enable experimental gated recovery in simulation'),
        DeclareLaunchArgument('input_mode', default_value='hand',
                              choices=['hand', 'mouse', 'csv'],
                              description='Select the arm velocity input source'),
        DeclareLaunchArgument('vision_source', default_value='fake',
                              description="'fake' or the camera URL"),
        DeclareLaunchArgument('max_speed', default_value='0.05',
                              description='Teleop hard speed cap in m/s'),
        DeclareLaunchArgument('mouse_gain', default_value='0.001',
                              description='Meters of tool travel per mouse pixel'),
        DeclareLaunchArgument('csv_path', default_value='palm_center.csv',
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
                              description='Tool metres per cm of hand distance from the '
                                          'camera; 0 disables depth, 0.01 tracks 1:1'),
        servo,
        RegisterEventHandler(OnProcessExit(target_action=prepare, on_exit=after_ready)),
        ready_pose,
        vision,
    ])
