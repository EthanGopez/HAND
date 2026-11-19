"""Check measured-pose gating after fake-controller initialization."""

from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import pytest

from hand_bringup import prepare_sim as prepare


def completed(result):
    future = MagicMock()
    future.done.return_value = True
    future.result.return_value = result
    return future


@pytest.mark.parametrize('blocked', [False, True])
def test_servo_only_restarts_after_measured_pose_passes(blocked):
    node = MagicMock()
    services = {}

    def client(service_type, name):
        service = MagicMock()
        service.call_async.return_value = completed(SimpleNamespace(success=True))
        services[name] = service
        return service

    node.create_client.side_effect = client
    handle = MagicMock()
    handle.accepted = True
    handle.get_result_async.return_value = completed(SimpleNamespace(result=SimpleNamespace(
        error_code=prepare.FollowJointTrajectory.Result.SUCCESSFUL)))
    with patch.object(prepare, 'rclpy') as ros, \
            patch.object(prepare, 'ActionClient') as action, \
            patch.object(prepare, 'StartupPose') as monitor, \
            patch.object(prepare.time, 'monotonic', side_effect=[0., 11.]):
        ros.create_node.return_value = node
        ros.ok.return_value = True
        action.return_value.send_goal_async.return_value = completed(handle)
        monitor.return_value.assess.return_value = SimpleNamespace(
            blocked=blocked, reason='Near singularity')
        if blocked:
            with pytest.raises(RuntimeError, match='Measured ready pose rejected'):
                prepare.main()
        else:
            prepare.main()
    assert '/servo_server/stop_servo' in services
    assert ('/servo_server/start_servo' in services) is not blocked
    ros.shutdown.assert_called_once()
