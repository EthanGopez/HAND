"""Headless Lite6 Servo integration test. Fake hardware is mandatory."""
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder
from uf_ros_lib.uf_robot_utils import generate_ros2_control_params_temp_file, load_yaml

POSE_DEGREES = [204.42974751901605, -11.143871979962466, 21.808630095310633,
                21.060790975538644, 32.95440599050424, -155.56641027220473]


def generate_launch_description():
    if os.environ.get('ROS_DOMAIN_ID') != '218' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Use run_fake_servo_test.py: isolated localhost-only domain 218 is required')
    import math
    params = generate_ros2_control_params_temp_file(
        os.path.join(get_package_share_directory('xarm_controller'), 'config',
                     'lite6_controllers.yaml'), update_rate=150, robot_type='lite')
    config = MoveItConfigsBuilder(
        dof=6, robot_type='lite', robot_ip='', limited=True,
        ros2_control_plugin='uf_robot_hardware/UFRobotFakeSystemHardware',
        ros2_control_params=params).to_moveit_configs()
    robot = ET.fromstring(config.robot_description['robot_description'])
    # Simulation-only proposal: change J1 alone to the vendor's +/-2*pi
    # range. Preserve every other restricted planning limit.
    joint1_limits = robot.find("joint[@name='joint1']/limit")
    joint1_limits.set('lower', str(-2*math.pi))
    joint1_limits.set('upper', str(2*math.pi))
    controls = robot.findall('ros2_control')
    assert len(controls) == 1
    assert controls[0].find('hardware/plugin').text == 'uf_robot_hardware/UFRobotFakeSystemHardware'
    for index, degrees in enumerate(POSE_DEGREES, start=1):
        name = f'joint{index}'
        angle = math.radians(degrees)
        limits = robot.find(f"joint[@name='{name}']/limit")
        assert float(limits.get('lower')) <= angle <= float(limits.get('upper'))
        interface = controls[0].find(f"joint[@name='{name}']/state_interface[@name='position']")
        initial = interface.find("param[@name='initial_value']")
        if initial is None:
            initial = ET.SubElement(interface, 'param', name='initial_value')
        initial.text = str(angle)
    description = {'robot_description': ET.tostring(robot, encoding='unicode')}
    servo = load_yaml('xarm_moveit_servo', 'config/xarm_moveit_servo_config.yaml')
    servo.update(move_group_name='lite6', use_gazebo=False,
                 command_out_topic='/lite6_traj_controller/joint_trajectory')
    return LaunchDescription([
        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[description, params], output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[description], output='screen'),
        Node(package='moveit_servo', executable='servo_node_main', name='servo_server',
             parameters=[description, config.robot_description_semantic,
                         config.robot_description_kinematics, config.joint_limits,
                         {'moveit_servo': servo, 'use_sim_time': False}], output='screen'),
        *[Node(package='controller_manager', executable='spawner',
               arguments=[name, '--controller-manager-timeout', '30'], output='screen')
          for name in ('joint_state_broadcaster', 'lite6_traj_controller')],
    ])
