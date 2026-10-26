# One camera frame of hand tracking.
std_msgs/Header header             # stamp = when the frame was captured; frame_id = "camera"
bool tracked                       # false when no hand is visible (landmarks are then all zero)
string handedness                  # "Left", "Right", or "" when not tracked
geometry_msgs/Point[21] landmarks  # MediaPipe order; x, y normalized to the image (0-1), z = relative depth, not metres
