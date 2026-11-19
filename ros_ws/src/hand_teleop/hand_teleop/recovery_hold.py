"""Release-to-arm heartbeat gate for deliberate recovery input."""

import math


class RecoveryHold:
    def __init__(self, timeout=0.2):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('Invalid hold timeout')
        self.timeout = timeout
        self.reset()

    def reset(self):
        self.released = False
        self.last_hold = None

    def update(self, held, now):
        if not math.isfinite(now):
            self.reset()
            return
        if not held:
            self.last_hold = None
            self.released = True
            return
        if self.last_hold is not None and not self.active(now):
            return
        if self.released or self.last_hold is not None:
            self.last_hold = now
            self.released = False

    def active(self, now):
        if self.last_hold is None:
            return False
        if not 0 <= now - self.last_hold <= self.timeout:
            self.reset()
            return False
        return True
