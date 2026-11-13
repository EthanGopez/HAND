# Lite6 hardware investigation

This document retains the original audit and subsequent evidence. The original
findings below describe the pre-repair code; see the offline repair follow-up
for the changes now implemented. No firmware was changed. The offline repair
used no controller connection or physical motion.

## Findings

The latest log establishes a joint-feedback timeout followed by a controller
recovery problem and a Servo singularity report. It does not establish the
cause of the initial timeout or the earlier circular path. An individual joint
at zero degrees is not sufficient to diagnose a singularity.

| Offline model pose, degrees J1 through J6 | Jacobian rank | Condition number |
| --- | ---: | ---: |
| `[0, 0, 0, 0, 0, 0]` | 5 | about `3.87e16` (singular) |
| `[0, 9.93, 31.8, 0, 21.87, 0]` | 6 | `13.5069` |

The configured Servo slowdown and halt thresholds are 17 and 30. The requested
pose is below both in this independent nominal-model calculation, despite
J1/J4/J6 being zero. This is not a collision, joint-limit, path, or physical
safety validation and does not establish the arm's actual current pose.
The analytic Jacobian was checked against finite differences, with maximum
absolute discrepancies below `1.8e-8` at both poses. See
[kinematics_results.json](kinematics_results.json).

## Latest launch timeline

Times are seconds after `Hardware feedback ready; enabling mouse at current pose`
in the latest run in the second attached log, beginning at epoch 1793669468.

| Offset | Evidence |
| --- | --- |
| 0.00 s | Hardware readiness check completes |
| 0.72 s | Mouse control reports ready |
| 4.27 s | First `Read() returns: 3` |
| 6.27 s | Controller switching times out after two seconds |
| 7.10 s | First Servo singularity warning in this run |
| 7.28–13.28 s | Read timeouts recur at about two-second intervals |
| 13.28 s | Controller reactivation warnings |

Thus the mouse pad/startup gating issue is resolved in this run. The remaining
problem occurs after startup. Earlier runs in the attachment still contain the
old startup trajectory, but the latest run does not.

## Confirmed code problems and limitations

1. **Blocking failed reads in a 150 Hz control loop.**
   `XArmDriver::update_joint_states` synchronously calls `get_joint_states` for
   this firmware. SDK `UXBUS_CONF::GET_TIMEOUT` is 2000 ms; `UXBUS_STATE::ERR_TOUT`
   is 3. The nominal control cycle is 6.67 ms. A timeout is roughly 300 cycles.
   The log's two-second recurrence agrees with that timeout.

   Sources: `xarm_api/src/xarm_driver.cpp:410`,
   `xarm_sdk/cxx/include/xarm/core/instruction/uxbus_cmd_config.h:181`.

2. **Controller switching waits inside the control update path.**
   `UFRobotSystemHardware::write` calls `_deactivate_controller()` after a failed
   read. That function calls the controller manager's own list/switch services
   using `_call_request`, which waits synchronously. Switching needs progress
   in the controller update loop. Blocking the loop this way is a credible
   mechanism for the observed switch timeout. This source-level mechanism has
   not been exercised against a running controller manager during this review;
   it must not be represented as a hardware-verified deadlock diagnosis.

   Sources: `xarm_controller/src/hardware/uf_robot_system_hardware.cpp:339`,
   `:400`, and `:22`.

3. **Failed reads can produce zero-valued SDK buffers.**
   A compiled reproducer linked against the installed `libxarm_cxx_sdk.so`
   overrides only the transport's send/receive operations, injects return code
   3, and checks real SDK decoding. All seven positions, seven velocities, and
   seven efforts are overwritten with zero on this failed read.

   `XArmDriver::update_joint_states` then unconditionally gives its message a
   new timestamp, copies those values, and publishes `/xarm/joint_states`.
   **Important distinction:** the hardware plugin copies driver positions into
   its exported states only when `read_code_ == 0`. Servo listens to the separate
   broadcaster topic `/joint_states`. Therefore this reproduction does not prove
   that fabricated zeros reached Servo in the recorded run. The hardware may
   retain the last accepted state instead. A capture of both sources and the
   independent `/xarm/robot_states` report is needed to distinguish them.

   Sources: `xarm_sdk/cxx/src/xarm/core/instruction/uxbus_cmd.cc:253` and `:699`,
   `xarm_api/src/xarm_driver.cpp:437`,
   `xarm_controller/src/hardware/uf_robot_system_hardware.cpp:310`.

4. **Read failures are reported as OK to ros2_control.**
   After logging a timeout, the hardware read function returns
   `hardware_interface::return_type::OK`, preserving the last accepted exported
   state. A broadcaster timestamp alone is not proof of a new physical read.
   The startup test checks message age and finite values, not measurement-source
   health. Rejecting all zero-valued angles is not a correct fix.

   Source: `xarm_controller/src/hardware/uf_robot_system_hardware.cpp:336`.

5. **Write error handling does not receive the SDK result.**
   `write()` stores the SDK return value in local `cmd_ret` and logs errors,
   but never assigns that result to member `write_code_`. `_need_reset()` tests
   `write_code_`, initialized to zero. Failed writes can therefore evade that
   recovery condition. No failed-write message in the supplied latest run proves
   this specific defect caused the observed incident.

   Source: `xarm_controller/src/hardware/uf_robot_system_hardware.cpp:364` and `:505`.

6. **Mouse control has no continuous hardware-health interlock.**
   The initializer exits on startup success. The GUI's click/stall protections
   and Servo timeout concern input-command freshness; they do not establish
   successful physical measurements or controller health. The latest log shows
   feedback degrading several seconds after this one-time readiness check.

   Sources: `hand_bringup/hand_bringup/prepare_hardware.py`,
   `hand_teleop/hand_teleop/mouse_teleop_node.py`.

## Configuration checks that passed

- Hardware uses `robot_type=lite`, `dof=6`, `lite6_controllers.yaml`,
  `lite6_traj_controller`, and Servo group `lite6`.
- Servo output is `/lite6_traj_controller/joint_trajectory`.
- SDK angles are configured in radians; the former requested startup angles
  were converted from degrees with `math.radians`.
- Mouse and hardware use wall-clock ROS time rather than Gazebo time.
- Only one mouse input source is launched; no vendor joystick component.
- The installed initializer resolves to the corrected source and contains no
  `FollowJointTrajectory` startup action.
- Servo singularity/collision protection remains enabled.

The missing Octomap sensor plugin explains absent 3D sensor obstacle updates;
it does not explain SDK read timeout code 3. Shutdown/class-loader/Tkinter
messages appear after Ctrl+C and cannot explain the earlier read failures.

## Evidence still missing

The saved log has no actual/commanded joint traces, controller tracking errors,
packet capture, or read-request latency before failure. SDK 1.18.1 and firmware
1.11.4 are identified, but this evidence does not establish incompatibility.
Do not assume that changing report type, increasing singularity thresholds,
adding retries, or upgrading firmware repairs this problem.

Before renewed motion testing, the vendor read/write path needs explicit
failure handling, successful read samples must retain their original validity,
and recovery must execute without blocking the control loop. Those changes need
isolated fault-injection tests and controlled hardware validation. No such
production-driver repair is claimed here.

## Offline reproductions

From `/workspace`:

```bash
# Uses a fake transport; sends no packets.
g++ -std=c++14 diagnostics/lite6/reproduce_sdk_read_failure.cpp \
  -Iros_ws/src/xarm_ros2/xarm_sdk/cxx/include \
  -Lros_ws/install/xarm_sdk/lib \
  -Wl,-rpath,/workspace/ros_ws/install/xarm_sdk/lib \
  -lxarm_cxx_sdk -pthread -o /tmp/reproduce_lite6_sdk_read_failure
/tmp/reproduce_lite6_sdk_read_failure

# Uses the offline model snapshot, without running any launch action.
/usr/bin/python3 diagnostics/lite6/check_kinematics.py \
  diagnostics/lite6/model_snapshot.urdf
```

62 Python cases passed across the hardware launch/readiness, mouse input,
discovery, and new read-only probe tests. The C++ fake-transport reproduction
and finite-difference Jacobian assertions also passed. These checks do not
validate physical movement or repair the timeout.

## Prepared read-only controller probe (not run against the arm)

With the ROS launcher and other SDK clients stopped, this diagnostic can compare
legacy angle reads with combined position/velocity/effort reads and record their
latency. It does not use SDK initialization and cannot send motor-enable,
mode-change, fault-clear, or motion commands. Tests reject functions outside
its fixed read allowlist. Each request uses an independent connection, so success
here would not validate the production stack's 150 Hz persistent/concurrent
traffic pattern.

```bash
python3 diagnostics/lite6/read_only_probe.py --ip 192.168.1.166 --protocol 2
# A separate comparison, if needed:
python3 diagnostics/lite6/read_only_probe.py --ip 192.168.1.166 --protocol 3
```

Both probes timing out suggests a different problem from combined reads alone
failing. Protocol-specific failures would identify a protocol path to investigate.
None of these patterns, by itself, authorizes firmware changes or a motion test.


## Follow-up: user-provided protocol 2 measurements

The user ran the prepared read-only probe with the hardware launcher stopped.
All six query types returned status byte 0, with response times from 1.33 to
4.43 ms. The error/warning response was `0000`. This validates those isolated
protocol 2 requests; it does not validate persistent SDK protocol 3 traffic or
the 150 Hz control loop.

The measured positions-only reply is approximately
`[180, 0, 0, 0, -0.00458, -180.01099]` degrees. Evaluating that exact sample with
the offline Lite6 model gives condition number **31046.36**, far above the
configured Servo halt threshold 30. Thus a near-singular measured pose now has
independent evidence; it need not be explained solely by failed-read artifacts.
Individual zeros remain valid, but this full configuration is near-singular.

From this measured pose, the former direct startup target would require about
-180 degrees on J1 and +180 degrees on J6, along with the other target changes.
Such moves could produce a large sweep. The pose at the start of the earlier
incident was not recorded, so this is a plausible explanation, not proof of
its exact trajectory or repeated circular motion. Automatic startup motion
remains removed.

The SDK automatically changes its protocol identifier from 2 to 3 for firmware
versions at least 1.8.6 (`xarm_report.cc:538`), which includes this controller.
The next isolated diagnostic is the same read-only probe with `--protocol 3`,
without the ROS hardware launcher or other SDK clients running. If both protocols
work in isolation, persistent traffic, request rate, and controller-recovery
behavior still require investigation. No firmware or protocol-default change
has been justified or applied.

See [protocol2_result.json](protocol2_result.json) for the saved sample and timing.


## Follow-up: changed user-reported pose and connection check

The next user-reported angles were `[204.4, -11.1, 21.8, 21.1, 33, -155.6]`
degrees. Offline evaluation of this pose gives full rank and condition number
18.18999: above slowdown threshold 17 and below halt threshold 30. This is a
model calculation from user-reported angles, not a live measurement or motion
clearance.

No hardware launch process was found. A read-only protocol-3 probe timed out.
A follow-up that explicitly separated TCP connection from protocol queries
found that TCP connections to port 502 timed out for both protocol selections.
No request could be sent in those comparison attempts. This is a current
reachability failure, not evidence that protocol 3 is incompatible. The robot
was not enabled or moved. The diagnostic now reports `failed_phase` to avoid
confusing connection failures with response timeouts.


## User-provided protocol 3 result

The user's isolated protocol 3 probe succeeded for all six query types, with
status byte 0 and times from 1.48 to 4.56 ms. Error/warning bytes were `0000`.
Measured angles were approximately `[204.43, -11.14, 21.81, 21.06, 32.95, -155.57]`
degrees. Thus both protocol identifiers work for isolated low-rate queries;
no protocol-default or firmware change is justified by these results. The
persistent read-only test is prepared with bounded duration, unique transaction
IDs, paced reads, and immediate abort on a failed read; it remains unrun against
the arm. Successful isolated probes do not reproduce the combined real control
traffic or establish the initial stall's cause.

## Offline repair follow-up

The user cannot connect the arm for this work. The following changes have been
implemented, compiled, and tested offline without attempting a robot connection:

- `XArmDriver::update_joint_states` returns immediately on failed SDK reads.
  Failed or non-finite samples cannot replace the last valid message or its
  timestamp. Legacy finite-difference velocity history comes from the driver's
  own last successful message instead of function-static shared arrays.
- The real hardware plugin returns ERROR on read/write failures, latches command
  transmission off, and requires reload before another activation. It does not
  call the controller manager's list/switch services from `read` or `write`.
- SDK write return codes now reach the hardware error path. Non-finite commands
  and double-to-float overflow are rejected before the transport hook.
- Activation requires a complete valid physical read before enabling motors.
  Position holds are seeded from measured angles and commanded velocity is zero.
  Activation no longer calls `clean_error`/`clean_warn`. Existing SDK driver
  initialization behavior outside this activation change remains vendor code.
- Both `/joint_states` and the successful driver `/xarm/joint_states` must be
  fresh before startup enables mouse input.
- `hardware_watchdog` checks successful driver feedback continuously after
  startup. Missing samples, invalid stamps/data, replayed samples that stop
  advancing, and clock inconsistency latch a failure and terminate the launch.
  The timeout is 0.5 seconds; restored feedback does not resume control.
- The startup trajectory remains removed. Singularity protection remains enabled.
- Uninitialized driver destruction is guarded against null SDK pointers.

The SDK still performs a synchronous read, and the initial read can still block
until its existing timeout. These changes remove repeated controller self-switch
recovery and prevent failed data from masquerading as a new driver sample; they
do not establish why the initial TCP/SDK stall occurred or guarantee a physical
stopping distance. Software command inhibition and ROS shutdown are not a
physical emergency-stop function. No connected-arm motion is claimed validated.

### Reproducible vendor patch

`patches/xarm_communication_faults.patch` contains all changed and new vendor
files, including the C++ tests. `scripts/apply_xarm_fault_patch.sh` requires
vendor commit `62936f7ea1846a85f7350de2c4c18f39e6d19715`, checks whether the patch
is already present, and applies it only if all hunks pass a dry check.
`build_ros_workspace.sh` invokes this helper before compiling. This prevents the
fixes from being lost when the Git submodule is initialized on a fresh checkout.
A clean temporary base tree was patched and every result byte-compared with the
working source; the reverse/idempotence check also passed.

### Verification

- Rebuilt and installed `xarm_api`, `xarm_controller`, and `hand_bringup` under
  `/workspace/ros_ws/install`, which the hardware launcher sources.
- **19 C++ cases passed:** six message-commit tests and thirteen actual hardware
  control-loop tests with fake transport hooks. These cover failed reads with
  zeroed SDK buffers, write errors, no automatic retry/reactivation, malformed
  samples, invalid/overflowing commands, valid zero positions, failed initial
  reads before motor enable, and measured-position startup holds.
- **79 Python cases passed:** launch wiring, dual-source readiness, watchdog
  state transitions, mouse mapping, discovery, and read-only diagnostic behavior.
- Vendor patch clean application, exact reproduction, and idempotence passed.
- Patch whitespace checks passed. No physical arm or hardware launch was used.

To rebuild offline:

```bash
cd /workspace
bash scripts/build_ros_workspace.sh
```

Hardware validation remains pending until the arm is available. It must cover
read reliability during combined traffic, actual tracking, communication loss,
shutdown/stop behavior, and user-observed motion. The previous circular path
still cannot be reconstructed from the saved logs alone.


## Real Servo / fake-hardware motion test

An isolated localhost-only ROS domain (218) ran the real installed MoveIt Servo,
robot state publisher, controller manager, joint-state broadcaster, and Lite6
trajectory controller with `uf_robot_hardware/UFRobotFakeSystemHardware` (the
vendor alias for `mock_components/GenericSystem`). No physical driver, robot IP,
GUI, or joystick publisher was launched. The process group was shut down after
the test.

**Additional configuration mismatch found:** the physical launcher currently uses
`limited=True`. Its restricted model bounds J1/J4/J6 to about +/-178.2 degrees
and J3 to about -3.5 through +178.2 degrees. The user's measured J1=204.43 degrees
is outside that model. The vendor Lite6 Xacro defaults permit J1/J4/J6 at +/-360
degrees and J3 at about -3.5 through +300 degrees. This is a joint-range/model
issue separate from the Jacobian singularity calculation.

The first fake simulation used the vendor default wider ranges (`limited=False`)
to represent the reported pose. Automatic approval review rejected the proposed
physical-launch range change as broadening a safety boundary beyond permission
to test fake hardware. **The physical launcher remains `limited=True`; no
physical range change was applied.** Its change requires explicit user approval
and later hardware validation. Singularity and collision checks stayed enabled
in the fake simulation.

At the exact latest reported measured pose, each direction +/-X, +/-Y, and +/-Z
was requested at 0.01 m/s for two seconds. The fake arm was reset to that pose
between trials, with Servo stopped during each fake-only reset. All six trials
moved in the requested direction, about 8.7–9.7 mm, with status codes 0, 1,
and/or 6. None reported singularity halt (2), collision halt (4), or joint-bound
halt (5). These short tests do not establish how far any motion can continue
before a limit or singularity.

An all-zero comparison produced no motion and codes 0/3 (collision proximity).
It does not isolate the runtime singularity halt behavior at zero because
collision behavior was also active. Its rank-deficient Jacobian remains
independently confirmed by the offline model calculation.

Results: [fake_servo_results.json](fake_servo_results.json).
Launch output: [fake_servo_test.log](fake_servo_test.log).
Reproducible runner: [run_fake_servo_test.py](run_fake_servo_test.py).


### Narrowed simulation proposal and final test

The simulation was rerun with `limited=True` and **only J1** changed to the
vendor's +/-360-degree range; all other restricted planning limits were retained.
This narrower model also completed all six short directional trials without
singularity/collision/joint-bound halt status codes. Final results and log files
now correspond to this narrower test, with travel about 8.8–9.6 mm per trial.

The proposed physical change is therefore limited to J1's model range, from
about +/-178.2 to +/-360 degrees. It has **not** been applied. It would widen
permitted base rotation and needs explicit user approval following automatic
review's rejection of physical safety-boundary changes. The physical launcher
continues to use its original restricted range. No physical-arm motion is
validated by these simulations.

The fake launch now refuses to run outside localhost-only domain 218, and the
runner refuses to start if that test domain already contains other nodes.

### Pose-aware mouse input filtering

The later physical log reached mouse readiness, then reported a Servo singularity
stop followed by a non-finite joint-command rejection. The hardware guard latched
transmission off and the feedback watchdog shut the launch down. The log does
not identify the precise numerical fault in the Servo/controller pipeline.
These protections remain enabled.

The mouse UI now previews each drag against the configured URDF and current
joint feedback, rejects motion approaching a singularity or configured joint
limit, and displays the reason. It does not modify physical joint bounds or
raise Servo's singularity thresholds. At condition 17 or above, motion that
worsens the condition is blocked; predicted/current condition 25 blocks motion.
The preview uses a 0.3-second horizon. Invalid/stale feedback fails closed.

The same predictor was tested with the real installed Servo and fake hardware
in localhost-only domain 218. The simulation-only J1 override described above
was retained to represent the reported pose. At 0.01 m/s, +X, +Y and -Z were
blocked for every sample and produced exactly zero joint change. -X, -Y and +Z
were allowed and moved approximately 9.3, 9.6 and 9.7 mm respectively. The
all-zero singular pose was blocked with zero motion. None of these seven cases
reported a Servo singularity/collision/joint-bound halt. This is numerical and
fake-hardware validation, not physical validation.

The actual reported J1 remains outside the physical launch's configured range;
the UI therefore blocks that pose with a joint-limit reason, regardless of
otherwise acceptable Cartesian directions. The user declined widening the
physical range, and it remains unchanged.

Results: [fake_servo_filtered_results.json](fake_servo_filtered_results.json).
Launch output: [fake_servo_filtered_test.log](fake_servo_filtered_test.log).
Reproduce with `run_fake_servo_test.py --guarded` after sourcing the workspace.
