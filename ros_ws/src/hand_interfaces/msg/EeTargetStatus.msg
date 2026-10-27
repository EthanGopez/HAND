# Health of the end-effector target tracker. See docs/interfaces/ee_target_tracker.md
uint8 NO_TARGET=0
uint8 TRACKING=1
uint8 REACHED=2
uint8 STALE=3
uint8 REJECTED=4

std_msgs/Header header             # stamp = when the status was sampled; frame_id = "link_base"
uint8 state                        # one of the constants above
float32 distance                   # metres from the current end-effector pose to the goal; 0 when there is no goal
float32 angle                      # radians between current and goal orientation; 0 when there is no goal
float32 target_age                 # seconds since the goal was received; 0 when there is no goal
bool publishing_twist              # true when this node is driving /servo_server/delta_twist_cmds
