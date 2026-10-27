import test from 'node:test';
import assert from 'node:assert/strict';
import { MODEL, fk, ik, jacobian, validQ } from '../src/kinematics.mjs';
import { mul, transpose, rotationError, rpy } from '../src/math.mjs';
const close = (a, b, tolerance = 1e-7) =>
  assert.ok(Math.hypot(...a.map((v, i) => v - b[i])) < tolerance, `${a} != ${b}`);
test('zero configuration and all six axes have consistent metre/radian conventions', () => {
  const zero = fk([0, 0, 0, 0, 0, 0]);
  close(zero.position, [0.65, 0, 0.15]);
  close(zero.rotation, [1, 0, 0, 0, 1, 0, 0, 0, 1]);
  const q = [0.3, 0.7, -0.8, 0.4, 0.6, -0.2],
    a = fk(q),
    b = fk([...q.slice(0, 5), 0.7]);
  close(a.position, b.position);
  assert.ok(rotationError(a.rotation, b.rotation) > 0.8);
});
test('full-pose IK reconstructs elbow, signed radial and both wrist branches', () => {
  for (const q of [
    [0.4, 0.7, -1, 0.5, 0.6, -0.3],
    [-0.7, 0.8, 0.5, -0.4, -0.7, 0.9],
    [0, 2, 0.15, 0.3, 0.4, -0.3],
    [0.2, 1.4, -1, 2.7, 1.1, 2.8],
  ]) {
    const desired = fk(q),
      solutions = ik(desired, q);
    assert.ok(solutions.length);
    for (const result of solutions) {
      assert.equal(validQ(result.q), true);
      close(fk(result.q).position, desired.position);
      assert.ok(rotationError(fk(result.q).rotation, desired.rotation) < 1e-7);
    }
  }
});
test('singular wrist returns an explicit valid representative without false uniqueness', () => {
  const pose = fk([0.3, 0.9, -0.7, 1.2, 0, -0.8]),
    solutions = ik(pose);
  assert.ok(solutions.some((x) => x.wristSingular));
  for (const solution of solutions) {
    close(fk(solution.q).position, pose.position);
    assert.ok(rotationError(fk(solution.q).rotation, pose.rotation) < 1e-7);
  }
});
test('geometric Jacobian agrees with position and orientation finite differences for all six joints', () => {
  const q = [0.3, 0.8, -1.1, 0.7, 0.6, -0.4],
    base = fk(q),
    J = jacobian(q),
    h = 1e-6;
  for (let joint = 0; joint < 6; joint++) {
    const shifted = [...q];
    shifted[joint] += h;
    const pose = fk(shifted),
      dR = mul(pose.rotation, transpose(base.rotation));
    const numeric = [
      ...pose.position.map((v, i) => (v - base.position[i]) / h),
      (dR[7] - dR[5]) / (2 * h),
      (dR[2] - dR[6]) / (2 * h),
      (dR[3] - dR[1]) / (2 * h),
    ];
    close(
      J.map((row) => row[joint]),
      numeric,
      2e-6,
    );
  }
});
test('proper rotations, sparse vectors, bounds and unreachable targets are checked', () => {
  assert.throws(() => fk(Array(6)));
  assert.throws(() => ik({ position: [NaN, 0, 0], rotation: rpy(0, 0, 0) }));
  assert.throws(() => ik({ position: [0.4, 0, 0.2], rotation: [-1, 0, 0, 0, 1, 0, 0, 0, 1] }));
  assert.deepEqual(ik({ position: [1.5, 0, 0.4], rotation: rpy(0, 0, 0) }), []);
  assert.equal(validQ(MODEL.home), true);
  assert.equal(validQ([0, 0, 0, 0, 3, 0]), false);
});
