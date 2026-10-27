# Security and reporting

The application has no account, persistence, analytics, remote API, hardware transport or firmware upload. Its development server binds only 127.0.0.1, serves a fixed list with GET/HEAD, validates Host and sends CSP/no-sniff/no-store headers. Keep it local. Static deployment configuration is outside these local checks.

Treat target/configuration input as untrusted. Only finite bounded coordinates, proper rotations, six bounded joints, selected integer seeds/branches and finite work/output limits are accepted. CLI unknown/duplicate/missing options fail; exports use exclusive creation and never overwrite a path. Text values use textContent in the browser. Report import and executable plugins are not supported.

The control runner is a trusted in-process emulator with a software watchdog. It is not authenticated remote control or independent physical safety. Do not attach it directly to drives or infer fault containment from a browser model. Real control/hardware needs its own reviewed design and physical tests.

Report vulnerabilities through private repository reporting if available; otherwise request a private maintainer contact without posting credentials or sensitive data in an issue. Include a bounded reproduction and affected commit. No response-time guarantee is offered.
