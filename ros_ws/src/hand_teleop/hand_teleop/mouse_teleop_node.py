"""Mouse velocity control for MoveIt Servo, with Shift selecting the X axis."""

import math
import time
import tkinter as tk

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node


class MouseInput:
    """Keep the input mapping independent of ROS and the graphical window.

    Accumulate pixel displacement between publication ticks, then divide by
    elapsed wall time to measure drag velocity. Pointer location itself has
    no effect. All three output axes are relative to the robot's base.
    """

    def __init__(self, max_speed, mouse_gain=0.0001):
        if not math.isfinite(max_speed) or max_speed <= 0:
            raise ValueError('max_speed must be finite and greater than zero')
        if not math.isfinite(mouse_gain) or mouse_gain <= 0:
            raise ValueError('mouse_gain must be finite and greater than zero')
        self.max_speed = max_speed
        self.mouse_gain = mouse_gain  # meters per pixel of pointer travel
        self.active = False
        self.horizontal = 0.0
        self.vertical = 0.0
        self.shift = False
        self.pointer = None
        self.sample_time = None

    def start(self, x, y, now):
        # Clicking only establishes a reference; it must never cause a jump.
        self.stop()
        self.active = True
        self.pointer = (x, y)
        self.sample_time = now

    def move(self, x, y):
        if self.active and self.pointer is not None:
            self.horizontal += x - self.pointer[0]
            self.vertical += y - self.pointer[1]
            self.pointer = (x, y)

    def select_shift(self, pressed):
        if self.shift != pressed:
            # Do not apply drag measured in Z mode to X after a mode switch.
            self.horizontal = self.vertical = 0.0
        self.shift = pressed

    def stop(self):
        # Disarm completely: returning to the pad must require another click.
        self.active = False
        self.horizontal = self.vertical = 0.0
        self.shift = False
        self.pointer = self.sample_time = None

    def velocity(self, now):
        if not self.active or self.sample_time is None:
            return 0.0, 0.0, 0.0
        elapsed = now - self.sample_time
        if elapsed <= 0:
            return 0.0, 0.0, 0.0
        if elapsed > 0.2:
            # A long GUI stall is a stop, never a deferred movement command.
            self.stop()
            return 0.0, 0.0, 0.0
        self.sample_time = now

        # Right maps to -Y, matching the existing hand teleop convention.
        # Up maps to +Z normally, or +X while Shift is held. The unused axis
        # is explicitly zero, so switching modes cannot retain its old speed.
        horizontal = -self.horizontal * self.mouse_gain / elapsed
        vertical = -self.vertical * self.mouse_gain / elapsed
        # Consume displacement once. Without new motion, the next tick sends
        # zero even if the button remains held and the pointer is off-center.
        self.horizontal = self.vertical = 0.0
        x = vertical if self.shift else 0.0
        y = horizontal
        z = 0.0 if self.shift else vertical

        # Cap the length of the entire velocity vector. A diagonal command
        # therefore has the same maximum speed as a single-axis command.
        speed = math.sqrt(x*x + y*y + z*z)
        scale = self.max_speed / max(self.max_speed, speed)
        return x * scale, y * scale, z * scale


class MouseTeleop(Node):
    """Publish stamped Cartesian velocities and display the mouse control pad."""

    WIDTH = 500
    HEIGHT = 400

    def __init__(self):
        super().__init__('mouse_teleop')
        max_speed = self.declare_parameter('max_speed', 0.25).value
        mouse_gain = self.declare_parameter('mouse_gain', 0.001).value
        self.controls = MouseInput(max_speed, mouse_gain)
        self.pub = self.create_publisher(
            TwistStamped, '/servo_server/delta_twist_cmds', 10
        )
        self.root = tk.Tk()
        self.root.title('xArm mouse control — hold left click to move')
        self.root.resizable(False, False)
        self.root.geometry('+20+220')  # Keep the control pad beside the robot view.
        tk.Label(
            self.root,
            text='Hold left click: horizontal → Y, vertical → Z\n'
                 'Hold Shift too: vertical → X (up = +X)\n'
                 'Drag faster = move faster. Hold still / release = stop.',
            padx=12, pady=10,
        ).pack()
        self.canvas = tk.Canvas(
            self.root, width=self.WIDTH, height=self.HEIGHT,
            bg='#20252c', highlightthickness=0,
        )
        self.canvas.pack()
        cx, cy = self.WIDTH / 2, self.HEIGHT / 2
        self.canvas.create_line(cx, 0, cx, self.HEIGHT, fill='#607080')
        self.canvas.create_line(0, cy, self.WIDTH, cy, fill='#607080')
        self.marker = self.canvas.create_oval(
            cx-6, cy-6, cx+6, cy+6, fill='#56c8a0', outline='',
        )
        self.status = tk.StringVar(value='STOPPED — click and hold inside the pad')
        tk.Label(self.root, textvariable=self.status, padx=8, pady=10).pack()

        self.closed = False
        self.shift_keys = set()
        self.last_tick = time.monotonic()
        self.canvas.bind('<ButtonPress-1>', self.press)
        self.canvas.bind('<Motion>', self.motion)
        self.root.bind('<ButtonRelease-1>', self.stop)
        self.canvas.bind('<Leave>', self.stop)
        self.root.bind('<FocusOut>', self.stop)
        self.root.bind('<KeyPress-Shift_L>', self.shift_press)
        self.root.bind('<KeyPress-Shift_R>', self.shift_press)
        self.root.bind('<KeyRelease-Shift_L>', self.shift_release)
        self.root.bind('<KeyRelease-Shift_R>', self.shift_release)
        self.root.bind('<Escape>', self.close)
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.root.after(50, self.tick)
        self.get_logger().info(
            f'Mouse control ready, total speed cap {max_speed} m/s. '
            'Hold left click; Shift selects X instead of Z.'
        )

    def motion(self, event):
        # Coordinates increase downward; signs are handled by MouseInput.
        # Sum event deltas rather than differentiating individual events:
        # networked desktops can batch events or deliver uneven intervals.
        self.controls.select_shift(bool(event.state & 0x0001))
        self.controls.move(event.x, event.y)
        self.canvas.coords(
            self.marker, event.x-6, event.y-6, event.x+6, event.y+6
        )

    def press(self, event):
        self.canvas.focus_set()
        self.controls.start(event.x, event.y, time.monotonic())
        self.motion(event)

    def shift_press(self, event):
        self.shift_keys.add(event.keysym)
        self.controls.select_shift(True)

    def shift_release(self, event):
        self.shift_keys.discard(event.keysym)
        self.controls.select_shift(bool(self.shift_keys))

    def publish(self, velocity):
        msg = TwistStamped()
        # Servo checks freshness. Each message needs a new ROS timestamp,
        # even when the pointer has not moved since the previous update.
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'link_base'
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = velocity
        # Angular velocity remains zero: this demo only translates the tool.
        self.pub.publish(msg)

    def stop(self, event=None):
        self.controls.stop()
        self.shift_keys.clear()
        self.publish((0.0, 0.0, 0.0))
        self.status.set('STOPPED — click and hold inside the pad')

    def tick(self):
        if not rclpy.ok():
            self.close()
            return
        now = time.monotonic()
        # If the GUI stalls, disarm before publishing again. Otherwise a held
        # command could resume after Servo's own 0.2-second timeout stopped it.
        if now - self.last_tick > 0.2:
            self.stop()
        self.last_tick = now
        rclpy.spin_once(self, timeout_sec=0.0)
        velocity = self.controls.velocity(now)
        self.publish(velocity)
        mode = 'Y / X (Shift)' if self.controls.shift else 'Y / Z'
        state = 'ACTIVE' if self.controls.active else 'STOPPED'
        self.status.set(
            f'{state} | {mode} | '
            f'X {velocity[0]:+.3f}  Y {velocity[1]:+.3f}  Z {velocity[2]:+.3f} m/s'
        )
        # A 20 Hz stream keeps commands fresh for Servo's 0.2-second timeout.
        # Sharing one event loop means a frozen GUI cannot keep sending motion.
        self.root.after(50, self.tick)

    def close(self, event=None):
        if not self.closed:
            self.closed = True
            if rclpy.ok():
                self.stop()
            self.root.destroy()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MouseTeleop()
        node.root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
