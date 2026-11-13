"""Bounded LAN discovery using the SDK's read-only GET_VERSION request.

Protocol source: xarm_sdk/cxx/src/xarm/core/instruction/uxbus_cmd_tcp.cc
and uxbus_cmd.cc. Discovery never enables motors or sends motion commands.
"""

from concurrent.futures import ThreadPoolExecutor
import fcntl
import ipaddress
from pathlib import Path
import re
import socket
import struct
import sys
import time


# Transaction 1, private protocol 2, length 1, GET_VERSION function 1.
VERSION_REQUEST = struct.pack('!HHHB', 1, 2, 1, 1)
MAX_HOSTS = 1024


def local_candidates():
    """Read Linux wired IPv4 interfaces; search only private /24 or smaller LANs."""
    addresses = set()
    interfaces = []
    for _, name in socket.if_nameindex():
        if (name.startswith(('lo', 'wl', 'docker', 'br-', 'veth', 'tun', 'tap'))
                or (Path('/sys/class/net') / name / 'wireless').exists()):
            continue
        try:
            request = struct.pack('256s', name.encode()[:15])
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                # Linux SIOCGIFADDR / SIOCGIFNETMASK. No packets are sent.
                addr = socket.inet_ntoa(fcntl.ioctl(sock, 0x8915, request)[20:24])
                mask = socket.inet_ntoa(fcntl.ioctl(sock, 0x891b, request)[20:24])
            interface = ipaddress.IPv4Interface(f'{addr}/{mask}')
        except (OSError, ValueError):
            continue
        addresses.add(interface.ip)
        interfaces.append(interface)
    candidates = set()
    for interface in interfaces:
        network = interface.network
        if (interface.ip.is_private and not interface.ip.is_loopback
                and not interface.ip.is_link_local and network.prefixlen >= 24):
            candidates.update(network.hosts())
    candidates.difference_update(addresses)
    if not candidates:
        raise RuntimeError(
            'No private wired IPv4 LAN with a /24 or smaller subnet is visible. '
            'Configure Ethernet on the arm subnet and expose it to the container, '
            'or supply the controller IP explicitly.')
    if len(candidates) > MAX_HOSTS:
        raise RuntimeError('Too many LAN addresses for automatic discovery; supply robot_ip.')
    return sorted(map(str, candidates), key=ipaddress.IPv4Address)


def identify_xarm6(address, timeout=0.5):
    """Return true only for a matching private-protocol xArm 6 version reply."""
    deadline = time.monotonic() + timeout
    try:
        with socket.create_connection((address, 502), timeout=timeout) as sock:
            sock.sendall(VERSION_REQUEST)

            def receive(count):
                data = bytearray()
                while len(data) < count:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        raise TimeoutError('Controller identification timed out')
                    sock.settimeout(remaining)
                    chunk = sock.recv(count - len(data))
                    if not chunk:
                        raise OSError('Incomplete controller response')
                    data.extend(chunk)
                return bytes(data)

            transaction, protocol, length = struct.unpack('!HHH', receive(6))
            if (transaction, protocol, length) != (1, 2, 42):
                return False
            payload = receive(length)
            if payload[0] != 1 or payload[1] & 0x08:
                return False
            version = payload[2:].split(b'\0', 1)[0].decode('ascii')
            # SDK version fields: axis, device_type, arm_type, control_type, version.
            # xArm 6 is (6,6); Lite6 (6,9) and UF850 (6,12) are distinct models.
            return bool(re.fullmatch(r'6,6,[^,]+,[^,]+,.*\d+\.\d+\.\d+.*', version))
    except (OSError, ValueError, UnicodeError):
        return False


def discover_robot_ip():
    candidates = local_candidates()
    print(f'Looking for an xArm 6 on {len(candidates)} local LAN addresses...',
          file=sys.stderr, flush=True)
    with ThreadPoolExecutor(max_workers=32) as pool:
        found = [address for address, matches in zip(
            candidates, pool.map(identify_xarm6, candidates)) if matches]
    if not found:
        raise RuntimeError(
            'No xArm 6 found. Check Ethernet, power, matching subnets, and container '
            'LAN access. Older firmware may require an explicit controller IP.')
    if len(found) != 1:
        raise RuntimeError('Multiple xArm 6 controllers found: ' + ', '.join(found)
                           + '. Supply the intended controller IP explicitly.')
    print(f'Found xArm 6 at {found[0]}', file=sys.stderr, flush=True)
    return found[0]


if __name__ == '__main__':
    try:
        print(discover_robot_ip())
    except RuntimeError as error:
        sys.exit(str(error))
