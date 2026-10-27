# Host reference-runner interface

`ReferenceRunner` is an in-process JavaScript emulator. It has no transport protocol, MCU firmware, serial/USB output, actuator driver, encoder/homing routine or physical safety function. The state named `ARMED` is a software state only.

| Call | Valid use | Result |
| --- | --- | --- |
| `new ReferenceRunner()` / `reset()` | Explicit software reset | DISARMED, no output, no loaded path; no physical movement |
| `load(path, boxes)` | DISARMED only | Validates dense six-angle vectors and all swept edges; creates a 50 ms reference; READY or latched FAULT |
| `arm(feedback, nowMs)` | READY, valid known initial pose | ARMED if each feedback joint is within 0.01 rad of reference start; otherwise FAULT |
| `start(nowMs)` | ARMED | RUNNING with a new software start timestamp |
| `heartbeat(nowMs)` | Active states | Refreshes lease only after checking the old lease; an expired lease cannot be revived |
| `tick(nowMs, feedback)` | Any state | Active states validate time/feedback, then emit a copied sampled reference or latch FAULT |
| `stop()` | Any state | Clears reference/output and returns DISARMED |
| `snapshot()` | Any state | Copied public state/output and controller-validation count |

Time is uint32 milliseconds. Forward clock wrap is supported. A modular delta of at least 2³¹ is rejected as backward/ambiguous. A gap greater than 200 ms since the last accepted heartbeat faults ARMED/RUNNING/COMPLETED before accepting a late refresh. Each active tick checks six finite joint-limited feedback values and at most 0.05 rad deviation from the prior output. No feedback source is implemented; the ideal demonstration deliberately supplies the previous setpoint.

FAULT removes output. Load, arm, start or a later tick cannot revive it without explicit reset and the full sequence. Completion holds the final position reference only while the same heartbeat and feedback checks remain healthy. Reset/stop change memory only and cannot establish that real drives are stopped. A real controller must independently enforce limits and remove drive energy on failure.

The runner has a separate 30,000-pose load-validation cap. Planner counts, controller counts and their sum are reported separately; the lab ceiling is 60,000 pose evaluations. No accepted path exceeds 64 waypoints. Generated references are at most 3,001 frames. A path is copied into computed frames, and snapshots copy their output vectors. The public API assumes trusted application code; this is not an authenticated network control service.

`idealTrackingDemo()` only shows that the emulator advances through the reference when fed its own prior setpoint every 50 ms. Its zero final joint-reference error is not a measured accuracy, tracking, latency or torque result. Exported trajectory dt can be 20..100 ms; the independent runner demonstration retains its fixed 50 ms reference. Neither is a physical motion guarantee.
