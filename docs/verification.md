# Verification — 27 October 2026

Local Node 24 checks: 18 domain/interface/CLI/server/URDF tests, static build, exact 15-run JSON/CSV/SVG reproduction and exact generated URDF/model JSON reproduction. The actual loopback server test walks every static browser/worker import and checks fixed files, CSP, methods and Host rejection. A separate Rodrigues-chain evaluation reads the generated URDF and compares its full pose with the simulator; no ROS runtime is inferred.

Independent parent review approved the written frame/axis, residual, swept-bound and singular-case limitations. The parent requested a distinct no_supported_ik outcome, which now has a regression separate from definite wrist-centre out-of-reach. Final source/browser review and exact-head CI are separate merge gates. Browser QA, narrow viewport and screenshots are not yet claimed in this record.

The controller demo uses ideal previous-setpoint feedback. No encoder data, motor communication, firmware compilation, flashing, ROS/Gazebo import, physical movement, calibration, payload, contact or stop test occurred. These remain pending in docs/hardware.md.

Approved cover 14 SHA256: 69215e1e9424f7be2ac2b95e92d0dd21715cb31de607caf16265ee8db134b580. Generated concept provenance is tracked; actual benchmark SVG comes from recorded model runs. Shared utility source is pinned to merged LiDAR commit/hash in docs/shared-source.json and Apache attribution is retained. MyPortfolio and protected repositories are unchanged.
