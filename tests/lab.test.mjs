import test from 'node:test';
import assert from 'node:assert/strict';
import { runLab } from '../src/lab.mjs';
import { edgeClear, poseClear } from '../src/motion.mjs';
test('actual six-axis scenarios distinguish routes, singular families and definite geometric failures', () => {
  for (const [scene, status, method] of [
    ['inspection', 'planned', 'direct'],
    ['fixture', 'planned', 'rrt'],
    ['blocked', 'goal_in_collision', null],
    ['far', 'geometric_out_of_reach', null],
    ['singular', 'planned', 'direct'],
  ]) {
    const r = runLab({ scene });
    assert.equal(r.metrics.status, status);
    assert.equal(r.metrics.method, method);
    assert.ok(r.metrics.collisionEvaluations <= 30000);
    if (status === 'planned') {
      assert.equal(r.control.status, 'COMPLETED');
      assert.ok(r.metrics.maxVelocityObserved <= 1 + 1e-9);
      assert.ok(r.metrics.maxAccelerationObserved <= 2 + 1e-9);
      for (let i = 1; i < r.plan.path.length; i++)
        assert.equal(edgeClear(r.plan.path[i - 1], r.plan.path[i], r.obstacles), true);
      assert.equal(poseClear(r.reference.frames.at(-1).q, r.obstacles), true);
    } else {
      assert.equal(r.reference, null);
      assert.equal(r.control, null);
    }
    if (scene === 'singular') assert.equal(r.metrics.wristSingular, true);
  }
});
test('selected branches, finite budgets and seeds produce repeatable honest results', () => {
  assert.deepEqual(runLab({ scene: 'fixture', seed: 2 }), runLab({ scene: 'fixture', seed: 2 }));
  const low = runLab({ poseBudget: 1 });
  assert.equal(low.metrics.status, 'planning_budget');
  assert.equal(low.reference, null);
  assert.throws(() => runLab({ branch: 31 }));
  assert.throws(() => runLab({ seed: NaN }));
  assert.throws(() => runLab({ scene: '__proto__' }));
});
test('unsupported constrained IK is not mislabeled as geometric unreachability', () => {
  const r = runLab({
    target: { position: [0.1, 0, -0.39], rotation: [1, 0, 0, 0, 1, 0, 0, 0, 1] },
  });
  assert.equal(r.metrics.status, 'no_supported_ik');
  assert.equal(r.reference, null);
});
