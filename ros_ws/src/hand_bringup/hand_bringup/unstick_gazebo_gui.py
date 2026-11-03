"""
unstick_gazebo_gui: wake gzclient if it deadlocks on startup, then raise its window.

Under x86 emulation (Apple Silicon running the amd64 dev container) gzclient reliably
stalls early in startup on a lost wakeup: it sits idle at ~0% CPU and never opens its
window, sometimes for many minutes. Stopping and continuing the process (what attaching
a debugger does) frees it within seconds. This does that until the window exists.

The window is found with xwininfo (package x11-utils). If that is missing it kicks blindly
for a minute instead. If wmctrl is installed the window is also maximized and raised.
"""
import os
import signal
import subprocess
import sys
import time

KICK_EVERY = 5.0        # s between kicks while the window is missing
FIND_TIMEOUT = 120.0    # s to wait for gzclient to be started at all
GIVE_UP = 300.0         # s of kicking before giving up
BLIND_KICKS_FOR = 60.0  # s of kicking when xwininfo is not available
WINDOW_TITLE = 'Gazebo'


def gzclient_pid():
    """Return the pid of a running gzclient, or None."""
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        try:
            with open(f'/proc/{entry}/comm') as comm:
                if comm.read().strip() == 'gzclient':
                    return int(entry)
        except OSError:
            continue
    return None


def window_is_up():
    """Return True/False for whether the Gazebo window exists, or None if xwininfo is missing."""
    env = dict(os.environ, DISPLAY=os.environ.get('DISPLAY', ':1'))
    try:
        tree = subprocess.run(['xwininfo', '-root', '-tree'], env=env, capture_output=True,
                              text=True, timeout=10).stdout
    except FileNotFoundError:
        return None
    except subprocess.TimeoutExpired:
        return False
    return f'"{WINDOW_TITLE}"' in tree


def kick(pid):
    """Stop and continue a process: enough to free a lost wakeup."""
    os.kill(pid, signal.SIGSTOP)
    time.sleep(0.3)
    os.kill(pid, signal.SIGCONT)


def raise_window():
    """Maximize and raise the Gazebo window if wmctrl is available."""
    env = dict(os.environ, DISPLAY=os.environ.get('DISPLAY', ':1'))
    for args in (['-r', WINDOW_TITLE, '-b', 'add,maximized_vert,maximized_horz'],
                 ['-a', WINDOW_TITLE]):
        try:
            subprocess.run(['wmctrl'] + args, env=env, timeout=10, check=False)
        except (FileNotFoundError, subprocess.TimeoutExpired):
            return


def main():
    start = time.monotonic()
    pid = None
    while pid is None and time.monotonic() - start < FIND_TIMEOUT:
        pid = gzclient_pid()
        time.sleep(1.0)
    if pid is None:
        print('unstick_gazebo_gui: gzclient was never started; nothing to do')
        return 1

    kicking_since = time.monotonic()
    kicks = 0
    while time.monotonic() - kicking_since < GIVE_UP:
        up = window_is_up()
        if up:
            print(f'unstick_gazebo_gui: Gazebo window is up after {kicks} kick(s)')
            raise_window()
            return 0
        if up is None and time.monotonic() - kicking_since > BLIND_KICKS_FOR:
            print('unstick_gazebo_gui: xwininfo not found (install x11-utils); '
                  'stopped kicking blindly')
            return 0
        if gzclient_pid() != pid:
            print('unstick_gazebo_gui: gzclient exited')
            return 1
        kick(pid)
        kicks += 1
        time.sleep(KICK_EVERY)
    print('unstick_gazebo_gui: the Gazebo window never appeared')
    return 1


if __name__ == '__main__':
    sys.exit(main())
