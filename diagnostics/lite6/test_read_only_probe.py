"""Simulated responses only; no socket connects to any host."""
import struct
from unittest.mock import MagicMock

import pytest

from read_only_probe import QUERIES, query


def transport(packet):
    sock = MagicMock()
    remaining = bytearray(packet)

    def recv(count):
        size = min(count, 3)
        chunk = bytes(remaining[:size])
        del remaining[:size]
        return chunk

    sock.recv.side_effect = recv
    return sock


@pytest.mark.parametrize('name,function,payload', QUERIES)
def test_only_read_functions_with_fragmented_responses(name, function, payload):
    if function == 42:
        count = 1 if not payload else payload[0]
        data = struct.pack('<' + 'f'*(count*7), *([0.25]*(count*7)))
    elif function == 1:
        data = b'6,9,LI1003,DL1000,v1.11.4\0'.ljust(40, b'\0')
    else:
        data = b'\0'
    response = bytes([function, 0]) + data
    sock = transport(struct.pack('!HHH', 1, 2, len(response)) + response)
    result = query(sock, 2, function, payload)
    assert result['status_byte'] == 0
    packet = sock.sendall.call_args.args[0]
    assert packet[6] in (1, 13, 15, 42)
    assert packet[7:] == payload


def test_motion_function_rejected_before_send():
    sock = MagicMock()
    with pytest.raises(ValueError, match='Only the documented read queries'):
        query(sock, 2, 11, b'\x01')
    sock.sendall.assert_not_called()


def test_timeout_does_not_manufacture_joint_angles():
    sock = MagicMock()
    sock.recv.side_effect = TimeoutError('simulated timeout')
    with pytest.raises(TimeoutError):
        query(sock, 2, 42, b'\x03')


def test_closed_socket_rejected():
    with pytest.raises(ConnectionError):
        query(transport(b''), 2, 42, b'\x03')


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


def test_persistent_reads_are_paced_and_use_unique_ids(monkeypatch):
    import read_only_probe as probe
    clock = FakeClock()
    calls = []

    def reply(sock, protocol, function, payload, **kwargs):
        calls.append((protocol, function, payload, kwargs['transaction_id'], clock()))
        clock.sleep(0.001)
        return {'status_byte': 0, 'positions_degrees': [0.1]*6}

    monkeypatch.setattr(probe, 'query', reply)
    result = probe.sample_joint_states(object(), 3, 1, 10, clock, clock.sleep)
    assert result['samples'] == 10
    assert result['achieved_hz'] == 10
    assert [call[3] for call in calls] == list(range(1, 11))
    assert all(call[:3] == (3, 42, b'\x03') for call in calls)
    assert all(b[4]-a[4] >= 0.099 for a, b in zip(calls, calls[1:]))


def test_persistent_read_timeout_aborts_without_retry(monkeypatch):
    import read_only_probe as probe
    from unittest.mock import Mock
    clock = FakeClock()
    request = Mock(side_effect=TimeoutError('injected timeout'))
    monkeypatch.setattr(probe, 'query', request)
    result = probe.sample_joint_states(object(), 3, 1, 150, clock, clock.sleep)
    assert result['samples'] == 0
    assert result['failed_sample'] == 1
    assert 'timeout' in result['error']
    request.assert_called_once()


@pytest.mark.parametrize('seconds,hz', [(6,150), (3,151), (float('nan'),10), (1,0)])
def test_persistent_limits_checked_before_query(monkeypatch, seconds, hz):
    import read_only_probe as probe
    from unittest.mock import Mock
    request = Mock()
    monkeypatch.setattr(probe, 'query', request)
    with pytest.raises(ValueError):
        probe.sample_joint_states(object(), 3, seconds, hz)
    request.assert_not_called()


def test_transaction_id_changes_wire_request():
    data = struct.pack('<7f', *([0.25]*7))
    packet = struct.pack('!HHHBB', 17, 3, len(data)+2, 42, 0) + data
    sock = transport(packet)
    query(sock, 3, 42, b'\x01', transaction_id=17)
    assert struct.unpack('!H', sock.sendall.call_args.args[0][:2])[0] == 17
