// SPDX-License-Identifier: Apache-2.0
export const identity = Object.freeze([1, 0, 0, 0, 1, 0, 0, 0, 1]);
export const dense = (value, length) =>
  Array.isArray(value) && value.length === length && Array.from(value).every(Number.isFinite);
export const add = (a, b) => a.map((v, i) => v + b[i]),
  sub = (a, b) => a.map((v, i) => v - b[i]),
  scale = (a, s) => a.map((v) => v * s),
  dot = (a, b) => a.reduce((s, v, i) => s + v * b[i], 0),
  cross = (a, b) => [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ],
  distance = (a, b) => Math.hypot(...sub(a, b)),
  clamp = (v, a, b) => Math.max(a, Math.min(b, v));
export const apply = (R, v) =>
  [0, 1, 2].map((i) => R[3 * i] * v[0] + R[3 * i + 1] * v[1] + R[3 * i + 2] * v[2]);
export const transpose = (R) => [R[0], R[3], R[6], R[1], R[4], R[7], R[2], R[5], R[8]];
export const mul = (A, B) =>
  [0, 1, 2].flatMap((i) =>
    [0, 1, 2].map((j) => A[i * 3] * B[j] + A[i * 3 + 1] * B[3 + j] + A[i * 3 + 2] * B[6 + j]),
  );
export const Rx = (a) => {
  const c = Math.cos(a),
    s = Math.sin(a);
  return [1, 0, 0, 0, c, -s, 0, s, c];
};
export const Ry = (a) => {
  const c = Math.cos(a),
    s = Math.sin(a);
  return [c, 0, s, 0, 1, 0, -s, 0, c];
};
export const Rz = (a) => {
  const c = Math.cos(a),
    s = Math.sin(a);
  return [c, -s, 0, s, c, 0, 0, 0, 1];
};
export const rpy = (roll, pitch, yaw) => {
  if (
    ![roll, pitch, yaw].every(Number.isFinite) ||
    [roll, pitch, yaw].some((v) => Math.abs(v) > 2 * Math.PI)
  )
    throw Error('RPY angles must be finite within±2pi.');
  return mul(mul(Rz(yaw), Ry(pitch)), Rx(roll));
};
export function validRotation(R) {
  if (!dense(R, 9)) return false;
  const rows = [R.slice(0, 3), R.slice(3, 6), R.slice(6, 9)];
  for (let i = 0; i < 3; i++)
    for (let j = 0; j < 3; j++)
      if (Math.abs(dot(rows[i], rows[j]) - (i === j ? 1 : 0)) > 1e-6) return false;
  return Math.abs(dot(rows[0], cross(rows[1], rows[2])) - 1) <= 1e-6;
}
export function rotationError(A, B) {
  const R = mul(A, transpose(B));
  return Math.acos(clamp((R[0] + R[4] + R[8] - 1) / 2, -1, 1));
}
