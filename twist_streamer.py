#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import TwistStamped


def main():
    rclpy.init()
    node = rclpy.create_node('twist_streamer')
    pub = node.create_publisher(TwistStamped, '/servo_server/delta_twist_cmds', 10)

    def tick():
        msg = TwistStamped()
        msg.header.stamp = node.get_clock().now().to_msg()   # servo ignores stale commands
        msg.header.frame_id = 'link_base'                     # directions relative to the base
        msg.twist.linear.z = 0.02                             # 2 cm/s upward
        pub.publish(msg)

    node.create_timer(0.05, tick)   # 20 Hz: faster than servo's 15 Hz and well inside the 0.2 s timeout
    rclpy.spin(node)


if __name__ == '__main__':
    main()
