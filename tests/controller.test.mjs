import test from 'node:test';
import assert from 'node:assert/strict';
import { MODEL } from '../src/kinematics.mjs';
import { ReferenceRunner } from '../src/controller.mjs';
const goal = [0.4, 0.9, -1, 0.2, 0.5, -0.2],
  path = [MODEL.home, goal];
test('reference runner needs load, known-pose arm and explicit start; stop removes output', () => {
  const runner = new ReferenceRunner();
  assert.equal(runner.snapshot().output, null);
  assert.equal(runner.tick(0, MODEL.home).output, null);
  assert.equal(runner.load(path, []).state, 'READY');
  assert.equal(runner.arm(MODEL.home, 1).state, 'ARMED');
  assert.deepEqual(runner.snapshot().output.q, MODEL.home);
  assert.equal(runner.tick(2, MODEL.home).state, 'ARMED');
  assert.equal(runner.start(3).state, 'RUNNING');
  runner.heartbeat(53);
  assert.equal(runner.tick(53, MODEL.home).state, 'RUNNING');
  const snapshot = runner.snapshot();
  snapshot.output.q[0] = 99;
  assert.notEqual(runner.snapshot().output.q[0], 99);
  assert.equal(runner.stop().output, null);
  assert.equal(runner.tick(54, MODEL.home).output, null);
});
test('mismatch, expiry, backward time and tracking faults latch neutral until reset', () => {
  const runner = new ReferenceRunner();
  runner.load(path, []);
  assert.equal(runner.arm(goal, 0).state, 'FAULT');
  assert.equal(runner.start(1).output, null);
  runner.reset();
  runner.load(path, []);
  runner.arm(MODEL.home, 0);
  runner.start(1);
  assert.equal(runner.heartbeat(202).state, 'FAULT');
  assert.equal(runner.tick(203, MODEL.home).output, null);
  runner.reset();
  runner.load(path, []);
  runner.arm(MODEL.home, 100);
  runner.start(101);
  assert.equal(runner.tick(99, MODEL.home).state, 'FAULT');
  runner.reset();
  runner.load(path, []);
  runner.arm(MODEL.home, 0xfffffff0);
  runner.start(0xfffffff1);
  assert.equal(runner.heartbeat(20).state, 'RUNNING');
  assert.equal(runner.tick(20, goal).state, 'FAULT');
  runner.reset();
  runner.load(path, []);
  assert.equal(runner.arm(Array(6), 0).state, 'FAULT');
});
test('ideal previous-setpoint feedback completes sampled reference but does not prove real tracking', () => {
  const runner = new ReferenceRunner();
  runner.load(path, []);
  runner.arm(MODEL.home, 0);
  runner.start(0);
  let feedback = [...MODEL.home],
    last;
  for (let now = 0; now < 10000; now += 50) {
    runner.heartbeat(now);
    last = runner.tick(now, feedback);
    assert.notEqual(last.state, 'FAULT');
    feedback = last.output.q;
    if (last.state === 'COMPLETED') break;
  }
  assert.equal(last.state, 'COMPLETED');
  assert.ok(Math.hypot(...last.output.q.map((v, i) => v - goal[i])) < 1e-8);
  assert.equal(runner.tick(20000, feedback).output, null);
});
test('reference load validates all poses and swept edges and cannot bypass a fault', () => {
  const runner = new ReferenceRunner();
  assert.equal(runner.load([MODEL.home, Array(6)], []).state, 'FAULT');
  assert.equal(runner.load(path, []).state, 'FAULT');
  assert.equal(runner.snapshot().output, null);
  runner.reset();
  assert.equal(
    runner.load(path, [{ min: [-0.1, -0.1, 0.1], max: [0.1, 0.1, 0.8] }]).state,
    'FAULT',
  );
  assert.equal(runner.arm(MODEL.home, 1).output, null);
  runner.reset();
  runner.load(path, []);
  runner.arm(MODEL.home, 0);
  runner.start(0);
  assert.equal(runner.load(path, []).state, 'FAULT');
  assert.equal(runner.tick(1, MODEL.home).output, null);
});
