"""Identify firmware and compare read queries without SDK startup or motor commands.

Run only with other controller clients stopped. This script sends only
GET_VERSION (1), GET_STATE (13), GET_ERROR (15), and GET_JOINT_POS (42).
It never enables motors, clears faults, changes mode, or sends trajectories.
"""
import argparse
import ipaddress
import json
import math
import socket
import struct
import time


QUERIES = [('version', 1, b''), ('state', 13, b''), ('errors', 15, b''),
           ('legacy_angles', 42, b''), ('positions', 42, b'\x01'),
           ('positions_velocity_effort', 42, b'\x03')]
ALLOWED = {(function, payload) for _, function, payload in QUERIES}


def query(sock, protocol, function, payload, timeout=2.0, transaction_id=1):
    if protocol not in (2, 3) or (function, payload) not in ALLOWED:
        raise ValueError('Only the documented read queries are allowed')
    if not 1 <= transaction_id <= 65535:
        raise ValueError('Transaction ID must be between 1 and 65535')
    deadline = time.monotonic() + timeout
    sock.settimeout(timeout)
    sock.sendall(struct.pack('!HHHB', transaction_id, protocol, len(payload)+1, function) + payload)

    def receive(count):
        data = bytearray()
        while len(data) < count:
            remaining = deadline-time.monotonic()
            if remaining <= 0:
                raise TimeoutError('Response deadline exceeded')
            sock.settimeout(remaining)
            chunk = sock.recv(count-len(data))
            if not chunk:
                raise ConnectionError('Connection closed before complete response')
            data.extend(chunk)
        return bytes(data)

    transaction, reply_protocol, length = struct.unpack('!HHH', receive(6))
    if transaction != transaction_id or reply_protocol != protocol or not 2 <= length <= 256:
        raise ValueError('Unexpected protocol header')
    response = receive(length)
    if response[0] != function:
        raise ValueError('Unexpected function in response')
    result = {'status_byte': response[1]}
    data = response[2:]
    if function == 1:
        result['version'] = data.split(b'\0', 1)[0].decode('ascii')
    elif function == 42:
        count = 1 if not payload else payload[0]
        if len(data) != count*7*4:
            raise ValueError(f'Unexpected joint response size: {len(data)}')
        values = struct.unpack('<' + 'f'*(len(data)//4), data)
        if not all(math.isfinite(v) for v in values):
            raise ValueError('Non-finite joint data')
        result['positions_degrees'] = [math.degrees(v) for v in values[:6]]
    else:
        result['data_hex'] = data.hex()
    return result



def sample_joint_states(sock, protocol, seconds, rate_hz,
                        clock=time.monotonic, sleep=time.sleep):
    """Use one connection, unique request IDs, and no retries or catch-up bursts."""
    if not (math.isfinite(seconds) and 0.1 <= seconds <= 5.0
            and math.isfinite(rate_hz) and 1 <= rate_hz <= 150):
        raise ValueError('Duration must be 0.1–5 seconds; rate must be 1–150 Hz')
    started = clock()
    deadline = started + seconds
    interval = 1.0 / rate_hz
    latencies = []
    result = {'protocol': protocol, 'requested_hz': rate_hz,
              'requested_seconds': seconds, 'samples': 0}
    for index in range(max(1, int(seconds*rate_hz))):
        before = clock()
        if before >= deadline:
            break
        try:
            response = query(sock, protocol, 42, b'\x03',
                             timeout=min(2.0, deadline-before), transaction_id=index+1)
            if response['status_byte'] & 0x48:
                raise ValueError(f"Controller error/invalid status: {response['status_byte']}")
        except (OSError, ValueError) as error:
            result['error'] = str(error)
            result['failed_sample'] = index+1
            break
        latencies.append((clock()-before)*1000)
        result['samples'] += 1
        if result['samples'] == 1:
            result['first_positions_degrees'] = response['positions_degrees']
        result['last_positions_degrees'] = response['positions_degrees']
        # Space from this request's start; a slow response never causes a burst.
        delay = min(max(0.0, interval-(clock()-before)), max(0.0, deadline-clock()))
        if delay:
            sleep(delay)
    elapsed = clock()-started
    result['elapsed_seconds'] = round(elapsed, 4)
    result['achieved_hz'] = round(result['samples']/elapsed, 2) if elapsed else 0.0
    if latencies:
        result['mean_read_ms'] = round(sum(latencies)/len(latencies), 3)
        result['max_read_ms'] = round(max(latencies), 3)
        result['reads_over_period'] = sum(ms > interval*1000 for ms in latencies)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ip', required=True, type=ipaddress.IPv4Address)
    parser.add_argument('--protocol', choices=(2, 3), type=int, default=2)
    parser.add_argument('--stream-seconds', type=float, default=None,
                        help='Read combined joint states on one connection for 0.1–5 seconds')
    parser.add_argument('--rate-hz', type=float, default=150,
                        help='Persistent read rate, 1–150 Hz (no motion commands)')
    args = parser.parse_args()
    if args.ip.is_loopback or args.ip.is_multicast or args.ip.is_unspecified:
        parser.error('Supply the physical controller IPv4 address')
    if args.stream_seconds is not None:
        if not (math.isfinite(args.stream_seconds) and 0.1 <= args.stream_seconds <= 5
                and math.isfinite(args.rate_hz) and 1 <= args.rate_hz <= 150):
            parser.error('Duration must be 0.1–5 seconds; rate must be 1–150 Hz')
        try:
            with socket.create_connection((str(args.ip), 502), timeout=2.0) as sock:
                result = sample_joint_states(sock, args.protocol,
                                             args.stream_seconds, args.rate_hz)
        except OSError as error:
            result = {'error': str(error), 'failed_phase': 'connect', 'samples': 0}
        print(json.dumps(result), flush=True)
        return int('error' in result or result['samples'] == 0)
    failed = False
    for name, function, payload in QUERIES:
        started = time.monotonic()
        row = {'query': name, 'protocol': args.protocol}
        phase = 'connect'
        try:
            # Independent connections isolate each read from a prior timeout.
            with socket.create_connection((str(args.ip), 502), timeout=2.0) as sock:
                phase = 'read_response'
                row.update(query(sock, args.protocol, function, payload))
        except (OSError, ValueError) as error:
            failed = True
            row['error'] = str(error)
            row['failed_phase'] = phase
        row['elapsed_ms'] = round((time.monotonic()-started)*1000, 2)
        print(json.dumps(row), flush=True)
        time.sleep(0.2)
    return int(failed)


if __name__ == '__main__':
    raise SystemExit(main())
