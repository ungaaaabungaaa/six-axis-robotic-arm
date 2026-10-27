// SPDX-License-Identifier: Apache-2.0
import { writeFile, readFile } from 'node:fs/promises';
import { MODEL } from '../src/kinematics.mjs';
const check = process.argv.slice(2).join(' ') === '--check';
if (process.argv.length > 2 && !check) throw Error('Only --check is supported.');
const cylinder = (length, radius) =>
  `<visual><origin xyz="${length / 2} 0 0" rpy="0 ${Math.PI / 2} 0"/><geometry><cylinder length="${length}" radius="${radius}"/></geometry></visual>`;
let urdf =
  '<?xml version="1.0"?>\n<!-- Kinematic model only: no inertias, effort=0 placeholders, no ros_control/transmissions. -->\n<robot name="six_axis_teaching_arm">\n<link name="base"><visual><origin xyz="0 0 .05"/><geometry><box size=".12 .12 .10"/></geometry></visual></link>\n';
const links = ['yaw', 'upper', 'fore', 'roll', 'pitch', 'tool'];
links.forEach((name, i) => {
  urdf += `<link name="${name}">${i === 1 ? cylinder(MODEL.lengths[0], MODEL.radii[0]) : i === 2 ? cylinder(MODEL.lengths[1], MODEL.radii[1]) : i === 5 ? cylinder(MODEL.lengths[2], MODEL.radii[2]) : ''}</link>\n`;
});
urdf += '<link name="tcp"/>\n';
const origins = [
    [0, 0, MODEL.height],
    [0, 0, 0],
    [MODEL.lengths[0], 0, 0],
    [MODEL.lengths[1], 0, 0],
    [0, 0, 0],
    [0, 0, 0],
  ],
  axes = [
    [0, 0, 1],
    [0, -1, 0],
    [0, -1, 0],
    [1, 0, 0],
    [0, -1, 0],
    [1, 0, 0],
  ];
links.forEach((name, i) => {
  urdf += `<joint name="q${i}" type="revolute"><parent link="${i === 0 ? 'base' : links[i - 1]}"/><child link="${name}"/><origin xyz="${origins[i].join(' ')}" rpy="0 0 0"/><axis xyz="${axes[i].join(' ')}"/><limit lower="${MODEL.limits[i][0]}" upper="${MODEL.limits[i][1]}" effort="0" velocity="1"/></joint>\n`;
});
urdf += `<joint name="tcp_offset" type="fixed"><parent link="tool"/><child link="tcp"/><origin xyz="${MODEL.lengths[2]} 0 0" rpy="0 0 0"/></joint>\n</robot>\n`;
const model =
  JSON.stringify(
    {
      schemaVersion: 1,
      kind: 'ideal kinematic geometry, not manufacturing or dynamic model',
      units: 'm',
      angleUnits: 'rad',
      frame: 'right-handed base z up',
      ...MODEL,
      jointOrigins: origins,
      jointAxes: axes,
      urdf: 'model/arm.urdf',
      urdfLimitNote:
        'effort=0 placeholder; no actuator, inertia, controller or physical load specification',
      physicalTests: 'pending',
    },
    null,
    2,
  ) + '\n';
for (const [name, value] of [
  ['model/arm.urdf', urdf],
  ['docs/model.json', model],
]) {
  const url = new URL('../' + name, import.meta.url);
  if (check) {
    if ((await readFile(url, 'utf8')) !== value) throw Error('Generated model changed: ' + name);
  } else await writeFile(url, value);
}
console.log(
  `${check ? 'Reproduced' : 'Generated'} URDF and model JSON; no ROS runtime or hardware was used.`,
);
