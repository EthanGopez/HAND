"""Exercise discovery with simulated replies; never probe the real LAN."""

from unittest.mock import MagicMock, patch
import socket
import struct

import pytest

from hand_bringup import discover_xarm as discovery


def response(version='6,6,AX1300,CB1300,v2.5.0', transaction=1, protocol=2):
    text = version.encode().ljust(40, b'\0')
    payload = b'\x01\x00' + text
    return struct.pack('!HHH', transaction, protocol, len(payload)) + payload


def identify(packet):
    sock = MagicMock()
    # TCP is a byte stream: intentionally fragment all reads.
    data = bytearray(packet)

    def recv(size):
        count = min(size, 3)
        chunk = bytes(data[:count])
        del data[:count]
        return chunk

    sock.recv.side_effect = recv
    sock.__enter__.return_value = sock
    with patch.object(discovery.socket, 'create_connection', return_value=sock):
        result = discovery.identify_xarm6('192.0.2.1')
    sock.sendall.assert_called_once_with(b'\x00\x01\x00\x02\x00\x01\x01')
    return result


def test_read_only_identification_with_fragmented_reply():
    assert identify(response())


@pytest.mark.parametrize('packet', [
    response('6,9,AX1300,CB1300,v2.5.0'),
    response('6,12,AX1300,CB1300,v2.5.0'),
    response('7,3,AX1300,CB1300,v2.5.0'),
    response('unrelated service'),
    response(transaction=2), response(protocol=0), response()[:12], b'',
])
def test_rejects_wrong_models_services_and_incomplete_replies(packet):
    assert not identify(packet)


def test_unreachable_host_is_not_a_match():
    with patch.object(discovery.socket, 'create_connection', side_effect=TimeoutError):
        assert not discovery.identify_xarm6('192.0.2.1')


@pytest.mark.parametrize('matches,expected', [
    ([True, False], '192.0.2.1'), ([False, False], 'No xArm 6 found'),
    ([True, True], 'Multiple xArm 6 controllers'),
])
def test_unique_selection(matches, expected):
    with patch.object(discovery, 'local_candidates', return_value=['192.0.2.1', '192.0.2.2']), \
            patch.object(discovery, 'identify_xarm6', side_effect=lambda ip: matches[int(ip[-1]) - 1]):
        if expected.startswith('192.'):
            assert discovery.discover_robot_ip() == expected
        else:
            with pytest.raises(RuntimeError, match=expected):
                discovery.discover_robot_ip()


@pytest.mark.parametrize('mask,allowed', [('255.255.255.0', True), ('255.255.0.0', False)])
def test_local_network_scope(mask, allowed):
    def ioctl(sock, operation, request):
        value = '192.168.1.50' if operation == 0x8915 else mask
        return bytes(20) + socket.inet_aton(value) + bytes(232)

    with patch.object(discovery.socket, 'if_nameindex', return_value=[(1, 'eth0')]), \
            patch.object(discovery.fcntl, 'ioctl', side_effect=ioctl), \
            patch.object(discovery.Path, 'exists', return_value=False):
        if allowed:
            candidates = discovery.local_candidates()
            assert len(candidates) == 253
            assert '192.168.1.50' not in candidates
            assert '192.168.1.0' not in candidates
            assert '192.168.1.255' not in candidates
        else:
            with pytest.raises(RuntimeError, match='No private wired'):
                discovery.local_candidates()
