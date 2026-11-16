"""Map palm-centre samples to a base-frame tool velocity.

This is the palm counterpart of `MouseInput`, and follows the same rules,
because they are what make the mouse pad safe rather than what make it a
mouse:

* Position itself commands nothing. Only *change* in position between samples
  does, so an arm never jumps when tracking starts or resumes somewhere new.
* Displacement is accumulated between publication ticks and divided by the
  elapsed time, so the measured speed does not depend on the sample rate.
* Displacement is consumed once. A hand that stops moving sends zero on the
  very next tick even though it is still visible.
* The whole velocity vector is capped, so diagonal motion is no faster than
  single-axis motion.

Units: the CSV's x, y and z are all centimetres, and each `scale_*` is metres of
tool travel per centimetre of hand motion, so 0.01 tracks the hand one-for-one.

The one real difference: elapsed time comes from the tracker's own frame
timestamps rather than wall time, because the samples were measured by the
camera, not by the moment this process happened to read them.

Nothing here imports rclpy.
"""

import math


class PalmInput:
    """Turn successive palm positions into a capped Cartesian velocity."""

    def __init__(self, max_speed, scale_y=0.01, scale_z=0.01, scale_x=0.0,
                 max_jump=6.0, max_depth_jump=1.0, max_gap=0.5, deadband=0.0):
        values = (max_speed, scale_y, scale_z, scale_x, max_jump,
                  max_depth_jump, max_gap, deadband)
        if not all(math.isfinite(value) for value in values):
            raise ValueError('PalmInput settings must all be finite')
        if max_speed <= 0:
            raise ValueError('max_speed must be greater than zero')
        if max_jump <= 0 or max_depth_jump <= 0 or max_gap <= 0 or deadband < 0:
            raise ValueError('max_jump, max_depth_jump and max_gap must be positive')
        self.max_speed = max_speed
        # Metres of tool travel per unit of CSV displacement, per axis.
        self.scale_y = scale_y   # CSV x (image right) -> robot -Y; m per cm
        self.scale_z = scale_z   # CSV y (image down)  -> robot -Z
        self.scale_x = scale_x   # CSV z (depth)       -> robot -X; 0 disables
        self.max_jump = max_jump
        # Depth is a separate threshold in its own units. It is far noisier
        # than the image-plane position, and it must not be able to veto
        # sideways motion that the camera saw perfectly well.
        self.max_depth_jump = max_depth_jump
        self.max_gap = max_gap
        self.deadband = deadband
        self.reference = None    # last accepted (x, y, z)
        self.sample_time = None  # its timestamp, in seconds
        self.dx = self.dy = self.dz = 0.0
        self.elapsed = 0.0
        self.rejected = 0        # whole samples dropped since construction
        self.depth_rejected = 0  # samples whose depth alone was dropped

    @property
    def active(self):
        return self.reference is not None

    def stop(self):
        """Disarm completely: motion must be re-established from a new sample."""
        self.reference = None
        self.sample_time = None
        self.dx = self.dy = self.dz = 0.0
        self.elapsed = 0.0

    def reacquire(self, position, timestamp_s):
        """Accept a position as the new reference without commanding motion."""
        self.reference = position
        self.sample_time = timestamp_s
        self.dx = self.dy = self.dz = 0.0
        self.elapsed = 0.0

    def update(self, x, y, z, timestamp_s):
        """Fold one sample into the pending displacement.

        Returns True when the sample contributed motion, False when it was
        rejected or only re-established the reference.
        """
        if not all(math.isfinite(value) for value in (x, y, z, timestamp_s)):
            self.rejected += 1
            self.stop()
            return False
        position = (x, y, z)
        if self.reference is None:
            self.reacquire(position, timestamp_s)
            return False
        step = timestamp_s - self.sample_time
        if step <= 0:
            if -step > self.max_gap:
                # The source clock restarted, so the old reference is from a
                # different session. Re-reference rather than reject forever.
                self.reacquire(position, timestamp_s)
                return False
            # A repeated or out-of-order frame measures no new motion.
            self.rejected += 1
            return False
        if step > self.max_gap:
            # The hand was lost and came back. Whatever it did in between is
            # not a drag, so re-reference instead of commanding the jump.
            self.reacquire(position, timestamp_s)
            return False
        delta = [component - previous
                 for component, previous in zip(position, self.reference)]
        if max(abs(delta[0]), abs(delta[1])) > self.max_jump:
            # A tracking glitch, a second hand, or a re-detection elsewhere in
            # frame. Any of those would otherwise become a large velocity.
            self.rejected += 1
            self.reacquire(position, timestamp_s)
            return False
        if abs(delta[2]) > self.max_depth_jump:
            # The depth estimate (cm) is DEPTH_CONSTANT / palm width in pixels, so
            # one badly fitted frame swings it wildly. Drop the depth step
            # only; the image-plane motion in this sample is still good.
            delta[2] = 0.0
            self.depth_rejected += 1
        self.dx += delta[0]
        self.dy += delta[1]
        self.dz += delta[2]
        self.elapsed += step
        self.reference = position
        self.sample_time = timestamp_s
        return True

    def velocity(self):
        """Consume the pending displacement and return (vx, vy, vz) in m/s."""
        if self.reference is None or self.elapsed <= 0:
            return 0.0, 0.0, 0.0
        dx, dy, dz, elapsed = self.dx, self.dy, self.dz, self.elapsed
        # Consume once: without new motion the next tick commands zero, even
        # while the hand is still perfectly visible in the frame.
        self.dx = self.dy = self.dz = 0.0
        self.elapsed = 0.0
        if max(abs(dx), abs(dy), abs(dz)) < self.deadband:
            return 0.0, 0.0, 0.0

        # Image right -> -Y and image down -> -Z match the sign convention
        # already used by teleop_node.py, so hand up raises the tool. Depth
        # grows as the hand moves away, so moving toward the camera is +X.
        x = -dz * self.scale_x / elapsed
        y = -dx * self.scale_y / elapsed
        z = -dy * self.scale_z / elapsed

        speed = math.sqrt(x * x + y * y + z * z)
        scale = self.max_speed / max(self.max_speed, speed)
        return x * scale, y * scale, z * scale
