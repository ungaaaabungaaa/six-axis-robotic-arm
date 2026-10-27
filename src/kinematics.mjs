// SPDX-License-Identifier: Apache-2.0
import {
  dense,
  add,
  sub,
  scale,
  cross,
  distance,
  apply,
  transpose,
  mul,
  Rx,
  Ry,
  Rz,
  validRotation,
  rotationError,
  clamp,
} from './math.mjs';
const RAD = Math.PI / 180,
  EPS = 1e-9;
export const MODEL = Object.freeze({
  lengths: Object.freeze([0.3, 0.25, 0.1]),
  height: 0.15,
  radii: Object.freeze([0.025, 0.022, 0.02, 0.035, 0.03]),
  limits: Object.freeze(
    [
      [-150, 150],
      [-60, 120],
      [-135, 135],
      [-170, 170],
      [-120, 120],
      [-170, 170],
    ].map((pair) => Object.freeze(pair.map((v) => v * RAD))),
  ),
  home: Object.freeze([0, 80, -90, 0, 30, 0].map((v) => v * RAD)),
});
export function validQ(q) {
  return (
    dense(q, 6) && q.every((v, i) => v >= MODEL.limits[i][0] - EPS && v <= MODEL.limits[i][1] + EPS)
  );
}
export function fk(q) {
  if (!validQ(q)) throw Error('Six finite joint angles within model limits are required.');
  const shoulder = [0, 0, MODEL.height],
    origins = [[0, 0, 0], shoulder],
    axes = [[0, 0, 1]],
    points = [shoulder];
  let R = Rz(q[0]);
  axes.push(apply(R, [0, -1, 0]));
  R = mul(R, Ry(-q[1]));
  const elbow = add(shoulder, apply(R, [MODEL.lengths[0], 0, 0]));
  points.push(elbow);
  origins.push(elbow);
  axes.push(apply(R, [0, -1, 0]));
  R = mul(R, Ry(-q[2]));
  const wrist = add(elbow, apply(R, [MODEL.lengths[1], 0, 0]));
  points.push(wrist);
  origins.push(wrist);
  axes.push(apply(R, [1, 0, 0]));
  R = mul(R, Rx(q[3]));
  origins.push(wrist);
  axes.push(apply(R, [0, -1, 0]));
  R = mul(R, Ry(-q[4]));
  origins.push(wrist);
  axes.push(apply(R, [1, 0, 0]));
  R = mul(R, Rx(q[5]));
  const position = add(wrist, apply(R, [MODEL.lengths[2], 0, 0]));
  points.push(position);
  return { position, rotation: R, points, origins, axes };
}
export function jacobian(q) {
  const pose = fk(q),
    columns = pose.axes.map((axis, i) => [
      ...cross(axis, sub(pose.position, pose.origins[i])),
      ...axis,
    ]);
  return [0, 1, 2, 3, 4, 5].map((row) => columns.map((col) => col[row]));
}
export function validatePose(pose) {
  if (
    !pose ||
    !dense(pose.position, 3) ||
    pose.position.some((v) => Math.abs(v) > 2) ||
    !validRotation(pose.rotation)
  )
    throw Error('Pose needs xyz metres within±2m and a proper row-major rotation.');
  return pose;
}
const variants = (angle) => [-2 * Math.PI, 0, 2 * Math.PI].map((offset) => angle + offset);
export function ik(target, seed = MODEL.home) {
  validatePose(target);
  if (!validQ(seed)) throw Error('IK seed violates joint limits.');
  const wrist = sub(target.position, apply(target.rotation, [MODEL.lengths[2], 0, 0])),
    radial = Math.hypot(wrist[0], wrist[1]),
    primary = radial < 1e-8 ? seed[0] : Math.atan2(wrist[1], wrist[0]),
    z = wrist[2] - MODEL.height,
    [a, b] = MODEL.lengths,
    solutions = [];
  const yaws =
    radial < 1e-8
      ? [
          [primary, 0],
          [0, 0],
        ]
      : [
          [primary, radial],
          [Math.atan2(Math.sin(primary + Math.PI), Math.cos(primary + Math.PI)), -radial],
        ];
  for (const [yaw, r] of yaws) {
    const cosine = (r * r + z * z - a * a - b * b) / (2 * a * b);
    if (cosine > 1 + EPS || cosine < -1 - EPS) continue;
    for (const sign of [1, -1]) {
      const elbow = sign * Math.acos(clamp(cosine, -1, 1)),
        rawShoulder = Math.atan2(z, r) - Math.atan2(b * Math.sin(elbow), a + b * Math.cos(elbow));
      for (const shoulder of variants(rawShoulder)) {
        const armR = mul(Rz(yaw), Ry(-shoulder - elbow)),
          W = mul(transpose(armR), target.rotation),
          middle = Math.acos(clamp(W[0], -1, 1)),
          wristOptions = [];
        if (Math.abs(Math.sin(middle)) < 1e-7) {
          if (W[0] < 0) continue;
          const total = Math.atan2(W[7], W[4]);
          for (const first of [total / 2, 0, seed[3]])
            for (const last of variants(total - first))
              wristOptions.push({ q: [first, 0, last], singular: true });
        } else
          for (const direction of [1, -1]) {
            const beta = middle * direction,
              first = Math.atan2(W[3] / Math.sin(beta), -W[6] / Math.sin(beta)),
              last = Math.atan2(W[1] / Math.sin(beta), W[2] / Math.sin(beta));
            for (const u of variants(first))
              for (const v of variants(last))
                wristOptions.push({ q: [u, -beta, v], singular: false });
          }
        for (const option of wristOptions) {
          const q = [yaw, shoulder, elbow, ...option.q];
          if (!validQ(q) || solutions.some((other) => distance(q, other.q) < 1e-7)) continue;
          const pose = fk(q),
            positionResidual = distance(pose.position, target.position),
            orientationResidual = rotationError(pose.rotation, target.rotation);
          if (positionResidual < 1e-7 && orientationResidual < 1e-7)
            solutions.push({
              q,
              positionResidual,
              orientationResidual,
              wristSingular: option.singular,
              elbowSingular: Math.abs(Math.sin(elbow)) < 1e-7,
            });
        }
      }
    }
  }
  return solutions.sort((a, b) => distance(a.q, seed) - distance(b.q, seed));
}
