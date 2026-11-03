"""
Tripwire gate for hand tracking. No ROS imports, so it can be tested anywhere.

Tracking only starts once a palm centre enters a circle in the middle of the frame,
so a hand just wandering into view does not start driving the robot. Once active,
a hand anywhere in frame keeps it active; after ``lost_timeout_s`` seconds with no
hand at all it goes back to waiting.
"""

# wrist and the finger base landmarks, averaged to get the palm centre
PALM_INDICES = (0, 1, 5, 9, 13, 17)


def palm_center(landmarks, width, height):
    """Palm centre of one hand in pixels, from normalized MediaPipe landmarks."""
    return (
        int(sum(landmarks[i].x for i in PALM_INDICES) / len(PALM_INDICES) * width),
        int(sum(landmarks[i].y for i in PALM_INDICES) / len(PALM_INDICES) * height),
    )


class TripwireGate:
    """Waits for a palm in the centre zone, then stays active until the hand is gone."""

    def __init__(self, radius_fraction=0.15, lost_timeout_s=5.0):
        if radius_fraction <= 0.0 or lost_timeout_s <= 0.0:
            raise ValueError('radius_fraction and lost_timeout_s must be positive')
        self.radius_fraction = radius_fraction  # of the shorter frame side
        self.lost_timeout_s = lost_timeout_s
        self.active = False
        self.last_seen = None

    def zone(self, width, height):
        """Return the zone's centre pixel and radius in pixels."""
        return (width // 2, height // 2), int(min(width, height) * self.radius_fraction)

    def in_zone(self, point, width, height):
        """Return True if a pixel point is inside the zone."""
        (zone_x, zone_y), radius = self.zone(width, height)
        return (point[0] - zone_x) ** 2 + (point[1] - zone_y) ** 2 <= radius ** 2

    def update(self, hands, width, height, now):
        """
        Advance one frame and return whether tracking is active.

        ``hands`` is the list of detected hands (MediaPipe ``hand_landmarks``) and
        ``now`` is a monotonic time in seconds.
        """
        if self.active:
            if hands:
                self.last_seen = now
            elif now - self.last_seen >= self.lost_timeout_s:
                self.active = False
                self.last_seen = None
            return self.active

        for landmarks in hands:
            if self.in_zone(palm_center(landmarks, width, height), width, height):
                self.active = True
                self.last_seen = now
                break
        return self.active

    def seconds_lost(self, now):
        """How long the hand has been missing while active (0 if visible or waiting)."""
        if not self.active or self.last_seen is None:
            return 0.0
        return now - self.last_seen
