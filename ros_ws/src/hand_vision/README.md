# `hand_vision`

Planned owner: Vision, Host, and Arm Control.

This package turns hand tracking into timestamped ROS messages. It contains two
nodes with different inputs and different maturity.

| Node | Input | Publishes |
| --- | --- | --- |
| `vision_node` | A synthetic circling hand (`source:=fake`); a real camera is not implemented | `/hand/landmarks` (`hand_interfaces/msg/HandLandmarks`) |
| `palm_csv_node` | The palm CSV written by [`vision_demos/hand_tracking_demo.py`](../../../vision_demos/hand_tracking_demo.py) | `/hand/palm_center` (`hand_interfaces/msg/PalmCenter`) |

## `palm_csv_node`

The tracking demo is a standalone OpenCV/MediaPipe program, not a ROS node. It
writes one CSV row per tracked frame and flushes after every frame, so the file
is a live log. This node follows that file and republishes each new row. It
performs no mapping to robot axes; [`hand_teleop`](../hand_teleop/README.md)
does that.

Run the tracking demo and this node together, from the same working directory
so the relative path matches:

```bash
# terminal 1 — writes palm_center.csv
cd /workspace && python3 vision_demos/hand_tracking_demo.py

# terminal 2 — republishes new rows
source /opt/ros/humble/setup.bash && source ros_ws/install/setup.bash
ros2 run hand_vision palm_csv_node --ros-args -p csv_path:=/workspace/palm_center.csv
```

### Parameters

| Parameter | Default | Meaning |
| --- | --- | --- |
| `csv_path` | `palm_center.csv` | File to follow, relative to the node's working directory |
| `poll_hz` | `60.0` | How often the file is checked; the demo writes at about 25 Hz |
| `replay` | `false` | `true` reads the file from the beginning and re-emits it at its recorded timing |
| `replay_speed` | `1.0` | Replay multiplier; only used when `replay` is true |
| `frame_id` | `camera` | `header.frame_id` on published messages |
| `max_rows_per_tick` | `240` | Upper bound on rows emitted from one read |

### What the CSV contains

`palm_center.csv` has the columns `timestamp_ms, center_x, center_y, center_z`.
A different header is refused rather than parsed.

* `timestamp_ms` is `time.monotonic_ns()` on the capture machine, not ROS time
  and not wall time. It travels in `source_timestamp_ms` and is the only clock
  that measures the hand's motion correctly.
* `center_x` and `center_y` are **centimetres** right of and below the image
  centre. The demo derives them from the measured palm width, assuming a real
  palm width of 8.5 cm (`PALM_WIDTH_CM`), so the scale depends on how well the
  palm was fitted in that frame. All three columns are now centimetres.
* `center_z` is the demo's depth estimate, `DEPTH_CONSTANT / palm width in
  pixels`, which is the hand's distance from the camera in **centimetres**
  (the constant 4613 implies a ~543 px focal length for the 8.5 cm palm, which
  is typical of a webcam). It is much noisier than `x` and `y`. Check it with a
  ruler: a hand 30 cm from the lens should read about 30. Recordings made
  before the all-centimetre change have `center_x`/`center_y` in metres and an
  older depth constant, so they replay about 100x too slowly in x and y.

### Rows exist only while a hand is tracked

The demo writes a row only while its tripwire gate is active: a palm must first
enter the centre circle of the frame, and tracking ends after five seconds with
no hand. So **silence on `/hand/palm_center` means "no hand"**, and a consumer
must treat a gap as a stop rather than as a held position. `palm_teleop_node`
does this with its own `stale_after` timeout.

### Rows this node drops

`PalmCsvReader` refuses any row it cannot fully trust, because a malformed row
would become a bogus displacement downstream: wrong field count, unparsable or
non-finite numbers, a repeated or backwards timestamp, an implausible offset,
or a depth that is zero or negative. A frame that detected two hands writes two
rows under one timestamp; the second is dropped, since the CSV has no hand
index to tell them apart.

The file need not exist when the node starts. When the demo restarts it reopens
its CSV with mode `w`, truncating it; the reader notices and reattaches. On the
first attach, existing rows are read only to validate the header and are then
discarded, so starting this node against a long-running demo cannot drive the
arm through minutes of recorded motion.

Parsing and file-following rules are checked in
[`test/test_palm_csv_reader.py`](test/test_palm_csv_reader.py), including
against the recorded CSV in this repository when it is present. No camera and
no robot are involved in those tests.
