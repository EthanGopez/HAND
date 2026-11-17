"""Incrementally read the palm-centre CSV that the hand tracking demo writes.

`vision_demos/hand_tracking_demo.py` opens its palm CSV with mode 'w' and
flushes after every frame, so the file is a live append-only log that is
truncated whenever the demo restarts. This reader follows such a file without
ever replaying a stale row as if it were new, and rejects any row it cannot
fully trust: a reader that guesses would hand the arm a bogus displacement.

Nothing here imports rclpy, so the parsing rules can be tested without a robot.
"""

import math
import os
from typing import NamedTuple

# Must match PALM_CSV_FIELDS in vision_demos/hand_tracking_demo.py. A file with
# a different header is some other CSV; reading it would be a silent mismatch.
EXPECTED_HEADER = ('timestamp_ms', 'center_x', 'center_y', 'center_z')

# The palm centre is an offset from the image centre expressed through the
# palm-width ruler, so a plausible hand is well inside two metres of frame centre.
MAX_OFFSET_CM = 200.0
# The depth estimate is DEPTH_CONSTANT / palm width in pixels: always positive,
# and unbounded only as the palm shrinks to nothing.
MAX_DEPTH = 1000.0


class PalmSample(NamedTuple):
    """One accepted CSV row."""

    timestamp_ms: int
    x: float      # centimetres right of the image centre
    y: float      # centimetres below the image centre
    z: float      # hand distance from the camera, centimetres


class PalmCsvReader:
    """Follow a palm CSV and hand out only newly appended, valid rows.

    The file need not exist yet: `read()` simply returns nothing until the
    tracking demo creates it, and picks it up on a later call.
    """

    def __init__(self, path, from_start=False, max_rows_per_read=240):
        if max_rows_per_read <= 0:
            raise ValueError('max_rows_per_read must be greater than zero')
        self.path = str(path)
        self.from_start = bool(from_start)
        self.max_rows_per_read = int(max_rows_per_read)
        self.handle = None
        self.identity = None      # (st_dev, st_ino) of the open file
        self.position = 0         # bytes consumed, for truncation detection
        self.partial = ''         # a row the writer has not finished yet
        self.last_timestamp_ms = None
        self.header_ok = False
        self.rejected = 0         # rows dropped since construction
        self.status = 'Waiting for the palm CSV to appear'

    # -- file lifecycle ---------------------------------------------------

    def close(self):
        if self.handle is not None:
            self.handle.close()
        self.handle = None
        self.identity = None
        self.position = 0
        self.partial = ''
        self.header_ok = False
        # A new session restarts the tracker's monotonic clock from a lower
        # value, so the old high-water mark must not reject its first rows.
        self.last_timestamp_ms = None

    def open(self, skip_backlog):
        """Attach to the file, returning whichever existing rows count as new.

        On the first attach the backlog is parsed only far enough to validate
        the header and is then discarded: a demo that has been running for ten
        minutes must not drive the arm through ten minutes of recorded hand
        motion the moment this node starts. On a reattach after the demo
        restarted, the file is new, so its rows are delivered (still capped by
        `max_rows_per_read`).
        """
        try:
            handle = open(self.path, 'rb')
            info = os.fstat(handle.fileno())
        except OSError as error:
            self.status = f'Cannot open {self.path}: {error.strerror or error}'
            return []
        self.close()
        self.handle = handle
        self.identity = (info.st_dev, info.st_ino)
        self.status = f'Following {self.path}'
        samples = self._drain()
        return [] if skip_backlog else samples

    def _rotated(self):
        """Report whether the writer replaced or truncated the file under us."""
        try:
            info = os.stat(self.path)
        except OSError:
            return True
        return ((info.st_dev, info.st_ino) != self.identity
                or info.st_size < self.position)

    # -- reading ----------------------------------------------------------

    def read(self):
        """Return the valid rows appended since the previous call."""
        if self.handle is None:
            # First attach, or a retry after an unusable file was dropped.
            return self.open(skip_backlog=not self.from_start)
        if self._rotated():
            # The demo restarted and truncated or replaced the file. Reopening
            # discards the partial line and the old timestamp high-water mark.
            return self.open(skip_backlog=False)
        return self._drain()

    def _drain(self):
        try:
            chunk = self.handle.read()
            self.position = self.handle.tell()
        except OSError as error:
            self.status = f'Read failed on {self.path}: {error.strerror or error}'
            self.close()
            return []
        if not chunk:
            return []
        # Decode defensively: a half-written multibyte character must not raise.
        self.partial += chunk.decode('utf-8', errors='replace')
        lines = self.partial.split('\n')
        # The final element is whatever follows the last newline, i.e. a row
        # the writer has not finished. Keep it until its newline arrives.
        self.partial = lines.pop()
        if len(self.partial) > 4096:
            # Not a CSV row under construction; treat the file as unusable.
            self.status = f'{self.path} does not look like a palm CSV'
            self.close()
            return []

        samples = []
        for line in lines:
            # csv.DictWriter terminates rows with \r\n by default.
            line = line.strip('\r').strip()
            if not line:
                continue
            if not self.header_ok:
                if tuple(field.strip() for field in line.split(',')) != EXPECTED_HEADER:
                    self.status = (f'{self.path} header is not '
                                   f'{",".join(EXPECTED_HEADER)}')
                    self.close()
                    return []
                self.header_ok = True
                continue
            sample = self.parse(line)
            if sample is None:
                self.rejected += 1
                continue
            self.last_timestamp_ms = sample.timestamp_ms
            samples.append(sample)
        if len(samples) > self.max_rows_per_read:
            # A long backlog is history, not a command. Keep only the newest
            # rows so a stalled consumer cannot be driven by old motion.
            samples = samples[-self.max_rows_per_read:]
        return samples

    def parse(self, line):
        """Turn one CSV row into a sample, or None when it cannot be trusted."""
        fields = line.split(',')
        if len(fields) != len(EXPECTED_HEADER):
            return None
        try:
            timestamp = float(fields[0])
            x, y, z = (float(field) for field in fields[1:])
        except ValueError:
            return None
        if not all(math.isfinite(value) for value in (timestamp, x, y, z)):
            return None
        timestamp_ms = int(timestamp)
        if self.last_timestamp_ms is not None and timestamp_ms <= self.last_timestamp_ms:
            # The tracker's timestamps strictly increase. A repeat or a step
            # backwards means interleaved or stale output, not new motion.
            return None
        if abs(x) > MAX_OFFSET_CM or abs(y) > MAX_OFFSET_CM:
            return None
        if not 0.0 < z <= MAX_DEPTH:
            # Depth is DEPTH_CONSTANT / palm width, which is never <= 0.
            return None
        return PalmSample(timestamp_ms, x, y, z)
