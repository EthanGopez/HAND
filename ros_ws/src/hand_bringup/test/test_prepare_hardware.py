"""Startup regression checks without ROS hardware or network connections."""

from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import pytest
from sensor_msgs.msg import JointState

from hand_bringup import prepare_hardware as prepare


def completed(result):
    future = MagicMock()
    future.done.return_value = True
    future.result.return_value = result
    return future


def test_ready_startup_never_sends_a_joint_goal():
    node = MagicMock()
    node.get_clock.return_value.now.return_value.nanoseconds = 10_000_000_000
    message = JointState()
    message.name = [f'joint{i}' for i in range(1, 7)]
    message.position = [0.1] * 6
    message.header.stamp.sec = 10

    def subscribe(message_type, topic, callback, qos):
        callback(message)
        return MagicMock()

    node.create_subscription.side_effect = subscribe
    controllers = MagicMock()
    controllers.call_async.return_value = completed(SimpleNamespace(controller=[
        SimpleNamespace(name=name, state='active') for name in prepare.CONTROLLERS
    ]))
    start = MagicMock()
    start.call_async.return_value = completed(SimpleNamespace(success=True, message=''))

    def client(service_type, name):
        # Whitelist readiness/start calls; any motion/other client fails the test.
        return {'/controller_manager/list_controllers': controllers,
                '/servo_server/start_servo': start}[name]

    node.create_client.side_effect = client
    node.create_publisher.side_effect = AssertionError('Initializer must not publish motion')
    with patch.object(prepare, 'rclpy') as ros, \
            patch('rclpy.action.ActionClient', side_effect=AssertionError('No startup goals')):
        ros.create_node.return_value = node
        ros.ok.return_value = True
        prepare.main()
    start.call_async.assert_called_once()
    node.create_publisher.assert_not_called()
    ros.try_shutdown.assert_called_once()


def test_interrupt_cleanup_is_idempotent():
    with patch.object(prepare, 'rclpy') as ros:
        ros.ok.return_value = True
        ros.spin_once.side_effect = KeyboardInterrupt
        with pytest.raises(SystemExit) as error:
            prepare.main()
        assert error.value.code == 130
    ros.try_shutdown.assert_called_once()
    ros.shutdown.assert_not_called()


def test_readiness_timeout_does_not_report_ready():
    with patch.object(prepare, 'rclpy') as ros, \
            patch.object(prepare.time, 'monotonic', side_effect=[0.0, 100.0]):
        ros.ok.return_value = True
        with pytest.raises(RuntimeError, match='readiness timed out'):
            prepare.main()
    ros.create_node.return_value.get_logger.return_value.info.assert_called_once()
    ros.try_shutdown.assert_called_once()


def test_broadcaster_without_successful_driver_reads_cannot_start_servo():
    node = MagicMock()
    node.get_clock.return_value.now.return_value.nanoseconds = 10_000_000_000
    message = JointState()
    message.name = [f'joint{i}' for i in range(1, 7)]
    message.position = [0.1]*6
    message.header.stamp.sec = 10

    def subscribe(message_type, topic, callback, qos):
        if topic == '/joint_states':
            callback(message)
        return MagicMock()

    node.create_subscription.side_effect = subscribe
    with patch.object(prepare, 'rclpy') as ros, \
            patch.object(prepare.time, 'monotonic', side_effect=[0.0, 0.0, 100.0]):
        ros.create_node.return_value = node
        ros.ok.return_value = True
        with pytest.raises(RuntimeError, match='readiness timed out'):
            prepare.main()
    node.create_client.return_value.call_async.assert_not_called()
