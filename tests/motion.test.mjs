import test from 'node:test';
import assert from 'node:assert/strict';
import { MODEL, fk } from '../src/kinematics.mjs';
import { poseClear, edgeClear, planMotion, trajectory } from '../src/motion.mjs';
test('six-axis sweeps detect an obstacle between clear endpoint poses', () => {
  const a = [-0.7, 0.8, -1.2, 0.4, 0.6, 0],
    b = [0.7, 0.8, -1.2, 0.4, 0.6, 0],
    point = fk([0, 0.8, -1.2, 0.4, 0.6, 0]).points[2],
    box = { min: point.map((v) => v - 0.012), max: point.map((v) => v + 0.012) };
  assert.equal(poseClear(a, [box]), true);
  assert.equal(poseClear(b, [box]), true);
  assert.equal(edgeClear(a, b, [box]), false);
  assert.equal(edgeClear(a, b, []), true);
});
test('motion respects work, joint and physical-envelope bounds and references obey rates', () => {
  const goal = [0.7, 0.8, -1, 0.4, 0.6, -0.3],
    plan = planMotion(MODEL.home, goal, [], { seed: 1 });
  assert.equal(plan.status, 'planned');
  assert.deepEqual(plan.path.at(-1), goal);
  assert.deepEqual(plan, planMotion(MODEL.home, goal, [], { seed: 1 }));
  const reference = trajectory(plan.path);
  assert.ok(reference.maxVelocityObserved <= 1 + 1e-9);
  assert.ok(reference.maxAccelerationObserved <= 2 + 1e-9);
  assert.ok(reference.frames.at(-1).velocity.every((v) => v === 0));
  assert.equal(planMotion(MODEL.home, goal, [], { poseBudget: 1 }).path, null);
  assert.equal(poseClear([0, -0.9, 0, 0, 0, 0], []), false);
  assert.throws(() => planMotion(MODEL.home, goal, [], { poseBudget: 30001 }));
  assert.throws(() => trajectory([MODEL.home, goal], { dt: 0.001 }));
});

test('wrist-roll swept tool envelope catches a thin obstacle between clear orientations', () => {
  const a = [0, 0.8, -1, -1, 0.8, 0],
    b = [0, 0.8, -1, 1, 0.8, 0],
    p = fk([0, 0.8, -1, 0, 0.8, 0]).position,
    boxes = [{ min: p.map((v) => v - 0.006), max: p.map((v) => v + 0.006) }];
  assert.equal(poseClear(a, boxes), true);
  assert.equal(poseClear(b, boxes), true);
  assert.equal(edgeClear(a, b, boxes), false);
});
