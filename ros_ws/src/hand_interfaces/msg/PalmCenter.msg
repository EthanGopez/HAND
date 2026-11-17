# One tracked palm-center sample, as recorded by vision_demos/hand_tracking_demo.py.
#
# Rows only exist while that demo's tripwire gate is active, so every message
# means "a hand was seen"; silence means "no hand". Consumers must therefore
# treat a gap longer than their own timeout as a stop command, not as a hold.
std_msgs/Header header    # stamp = when this sample was published to ROS; frame_id = "camera"
int64 source_timestamp_ms # tracker's own monotonic frame time; use this for sample-to-sample dt
geometry_msgs/Point center  # all in centimetres: x = right of image centre, y = below image centre,
                            # z = hand distance from the camera (DEPTH_CONSTANT / palm width px;
                            #     inferred from the constant's implied focal length, confirm with a ruler)
