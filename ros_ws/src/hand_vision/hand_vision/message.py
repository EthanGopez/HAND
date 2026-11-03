"""Build HandLandmarks messages. Kept apart from the node so it tests without a camera."""
from geometry_msgs.msg import Point
from hand_interfaces.msg import HandLandmarks

NUM_LANDMARKS = 21
FRAME_ID = 'camera'


def build_message(stamp, landmarks=None, handedness=''):
    """
    Build the HandLandmarks message for one frame.

    With ``landmarks`` None the message is untracked: ``tracked`` false, empty
    handedness and all 21 points zero, as the message definition requires.
    """
    msg = HandLandmarks()
    msg.header.stamp = stamp
    msg.header.frame_id = FRAME_ID
    if landmarks is None:
        msg.tracked = False
        msg.handedness = ''
        msg.landmarks = [Point() for _ in range(NUM_LANDMARKS)]
        return msg
    if len(landmarks) != NUM_LANDMARKS:
        raise ValueError(f'expected {NUM_LANDMARKS} landmarks, got {len(landmarks)}')
    msg.tracked = True
    msg.handedness = handedness
    # x, y normalized to the image (0-1); z is MediaPipe's relative depth, not metres
    msg.landmarks = [Point(x=float(p.x), y=float(p.y), z=float(p.z)) for p in landmarks]
    return msg
