# Preliminary six-axis build and acceptance plan

No physical arm has been assembled or tested. The generated cover is not a parts drawing. The ideal kinematic URDF/model is not manufacturing CAD, a dynamics model, a wiring pinout or an approved actuator specification.

## Mechanical work before fabrication

The model has a 0.15 m shoulder height, 0.30 m upper link, 0.25 m fore link and 0.10 m axial tool offset. The wrist axes X/-Y/X meet at one ideal point; a real wrist needs a mechanical design that preserves or explicitly changes those transforms. Bearings, shaft support, motor placement, reduction, cable routing and tool mounting are not defined by a drawn sphere/cylinder. Change the FK/IK/URDF and calibration together if physical axis offsets differ.

Determine link, actuator and payload masses and centres of mass. Analyze worst-case static/gravity loads, acceleration, friction, efficiency, backlash, structural stress, fatigue, bearing loads and base overturning. Each downstream mass contributes its weight times horizontal lever arm to static joint torque. No mass or load rating is measured here, so the URDF omits inertia and uses effort=0 placeholders. Do not select a motor or claim a payload from the simulation's 1 rad/s speed limit.

## Preliminary functional bill of materials

| Function | Quantity | Selection evidence needed |
| --- | --- | --- |
| Base and bench anchoring | 1 set | Structural/stability analysis and verified fasteners |
| Six joint actuation channels | 6 | Torque/speed study, feedback, power-off behavior and mechanical limits |
| Upper/fore/tool structures | 3 main links | Manufacturing dimensions, stiffness, alignment and cable clearance |
| Spherical-wrist mechanism | 1 assembly | Verified coincident/offset axes, bearings, interference and tool roll travel |
| Position feedback and reference/limit sensors | Per joint | Resolution, repeatability, wiring and fault response |
| Drives and power supply | Matched set | Current/voltage, stall/thermal cases, fusing and protection |
| Embedded controller | 1 | Independent reviewed watchdog, limits, output ordering and feedback control |
| Independent stop / energy isolation | 1 system | Reviewed stop behavior, drive isolation and test procedure |
| Harness, connectors and strain relief | As designed | Current ratings, grounding, moving bend radius and shielding |
| Tool/end effector | Optional | Separate geometry, retention and contact/load validation |

Exact parts, prices, availability, electrical ratings and connector pins are pending. No purchases, controller flashing or device access occurred. There is no assumed Arduino/servo pin map: choose and review the actual electronics and official datasheets first. The host reference runner is not a motor controller.

## Staged physical acceptance

1. Review mechanical/electrical designs, stop means, torque/current and base stability. Establish a protected bench area and a safe unpowered inspection/handling procedure.
2. Build the fixture and joints unpowered. Check axis directions, ranges, stops, assembly clearances and cable routing. Verify the real wrist offsets and geometry before using the analytic solver.
3. Inspect polarity, continuity, protective devices, feedback wiring, driver ratings and isolation with power removed. Review the actual schematic and pin map.
4. Bench-test one unloaded channel with current-limited power and an independent stop. Prove neutral boot, feedback loss, watchdog, output-ordering, mechanical limits and fault recovery on the actual controller. No JavaScript test establishes these properties physically.
5. Establish joint zero/sign and homing with a reviewed procedure. Measure link/axis transforms and compare several physical poses with FK using an independent measurement method. Record calibration residuals and repeatability.
6. Expand obstacle/link envelopes using measured uncertainty, cables, tools and stop distance. Start at reduced speed/travel without payload. Measure commanded-versus-observed paths, stopping, thermal behavior, drift and reconnect/restart response.
7. Review the evidence before payload or contact experiments. Record actual hardware, supply, firmware/toolchain revision, configuration, test method and results separately from simulation.

Mechanical fit, actual electronics/firmware, homing, motor movement, load/thermal performance, stop behavior and all physical acceptance remain pending. There is no ROS/Gazebo runtime test or commercial robot support claim.
