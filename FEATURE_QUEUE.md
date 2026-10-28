# Feature queue

Delivered 28 October 2026: trajectory options reject unknown names and invalid option containers. A typo such as `maxVelociy` can no longer silently use the faster default. Explicit low velocity/acceleration and sample interval settings are checked in regression tests. Existing valid references and recorded demos are unchanged.

Next useful work:
Delivered final round: solve reports include bounded diagnostics over every sampled reference pose: wrist/elbow sine factors, wrist-centre base radius, near-singular sample counts, minima and sign-change brackets. Between-sample tangent contacts can be missed; no inverse-velocity or physical safety certificate is inferred.

- Expand joint-velocity and acceleration boundary cases across multiple segments.
- Extend controller fault/recovery sequences before any hardware adapter.

This lab emits model references. It does not establish physical tracking or safe operation.
