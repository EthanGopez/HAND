"""xArm6 in Gazebo, driven through MoveIt Servo by exactly one HAND node.

The stock xarm6_moveit_gazebo launch has no Servo in it (the xarm Servo launch files
cover only fake and real hardware), so this adds a Servo node configured for Gazebo and
then the node that feeds it. Simulation only: nothing here takes a robot IP.

    ros2 launch hand_bringup gazebo_tracker.launch.py                 # ee_target_tracker
    ros2 launch hand_bringup gazebo_tracker.launch.py max_speed:=0.1
    ros2 launch hand_bringup gazebo_tracker.launch.py driver:=teleop  # hand_teleop instead
    ros2 launch hand_bringup gazebo_tracker.launch.py driver:=none    # Servo only
"""
import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder
from uf_ros_lib.uf_robot_utils import generate_ros2_control_params_temp_file, load_yaml

DOF = '6'
ROBOT_TYPE = 'xarm'
XARM_TYPE = f'{ROBOT_TYPE}{DOF}'  # move group and controller prefix, "xarm6"
DRIVERS = ('tracker', 'teleop', 'none')


def servo_node(context):
    """MoveIt Servo for the Gazebo arm: same kinematics and limits as move_group."""
    ros2_control_params = generate_ros2_control_params_temp_file(
        os.path.join(get_package_share_directory('xarm_controller'), 'config',
                     f'{XARM_TYPE}_controllers.yaml'),
        update_rate=1000, use_sim_time=True, robot_type=ROBOT_TYPE)
    moveit_config = MoveItConfigsBuilder(
        context=context,
        controllers_name='fake_controllers',
        dof=DOF,
        robot_type=ROBOT_TYPE,
        hw_ns='xarm',
        limited=True,
        ros2_control_plugin='gazebo_ros2_control/GazeboSystem',
        ros2_control_params=ros2_control_params,
    ).to_moveit_configs()
    robot_description = {}
    robot_description.update(moveit_config.robot_description)
    robot_description.update(moveit_config.robot_description_semantic)
    robot_description.update(moveit_config.robot_description_kinematics)
    robot_description.update(moveit_config.joint_limits)
    robot_description.update(moveit_config.planning_pipelines)

    servo = load_yaml('xarm_moveit_servo', 'config', 'xarm_moveit_servo_config.yaml')
    servo['move_group_name'] = XARM_TYPE
    servo['command_out_topic'] = f'/{XARM_TYPE}_traj_controller/joint_trajectory'
    # The config ships at 15 Hz output. Match the 50 Hz command stream and publish a
    # trajectory as soon as a twist arrives instead of waiting for the next period.
    servo['publish_period'] = 0.02
    servo['low_latency_mode'] = True
    return Node(
        package='moveit_servo',
        executable='servo_node_main',
        name='servo_server',  # command topic is /servo_server/delta_twist_cmds
        output='screen',
        parameters=[
            {'moveit_servo': servo},
            robot_description,
            {'use_sim_time': True},
            # xarm's low_pass_filter_coeff is ignored by this Servo; the smoothing plugin
            # reads this one.
            {'butterworth_filter_coeff':
                float(LaunchConfiguration('servo_filter_coeff').perform(context))},
        ],
    )


def launch_setup(context, *args, **kwargs):
    driver = LaunchConfiguration('driver').perform(context)
    if driver not in DRIVERS:
        raise RuntimeError(f"driver must be one of {DRIVERS}, got '{driver}'")

    actions = [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('xarm_moveit_config'), 'launch',
             'xarm6_moveit_gazebo.launch.py']))),
        servo_node(context),
        # Moves the arm off its singular start pose, then starts Servo. It waits for
        # Gazebo's controller on its own, so it can start with everything else.
        Node(package='hand_bringup', executable='prepare_arm', output='screen'),
    ]
    # Only one node may publish /servo_server/delta_twist_cmds, so exactly one driver
    # is ever started here.
    if driver == 'tracker':
        actions.append(Node(
            package='hand_ee_target_tracker',
            executable='ee_target_tracker',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'publish_twist': True,
                # launch arguments arrive as strings; the node's parameters are doubles
                'max_speed': float(LaunchConfiguration('max_speed').perform(context)),
                'max_angular_speed':
                    float(LaunchConfiguration('max_angular_speed').perform(context)),
                'stale_after': float(LaunchConfiguration('stale_after').perform(context)),
            }],
        ))
    elif driver == 'teleop':
        actions.append(Node(
            package='hand_teleop',
            executable='teleop_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('driver', default_value='tracker',
                              description='Which node drives Servo: tracker, teleop or none'),
        DeclareLaunchArgument('servo_filter_coeff', default_value='1.01',
                              description="Servo's butterworth_filter_coeff; see the launch file"),
        DeclareLaunchArgument('max_speed', default_value='0.05',
                              description='Tracker linear speed cap, m/s'),
        DeclareLaunchArgument('max_angular_speed', default_value='0.5',
                              description='Tracker angular speed cap, rad/s'),
        DeclareLaunchArgument('stale_after', default_value='1.0',
                              description='Seconds without a goal before the tracker holds'),
        OpaqueFunction(function=launch_setup),
    ])
