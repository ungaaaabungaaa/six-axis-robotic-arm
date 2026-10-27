// SPDX-License-Identifier: Apache-2.0
import { validQ } from './kinematics.mjs';
import { distance } from './math.mjs';
import { validateBoxes, poseClear, edgeClear, makeBudget, trajectory } from './motion.mjs';
/** Host reference emulator only. No actuator, transport or physical feedback driver. */
export class ReferenceRunner {
  constructor() {
    this.reset();
  }
  reset() {
    this.validationEvaluations = 0;
    this.state = 'DISARMED';
    this.reason = 'neutral';
    this.output = null;
    this.reference = null;
    this.lastClock = null;
    this.lastHeartbeat = 0;
    this.startedAt = 0;
    this.index = 0;
    return this.snapshot();
  }
  fault(reason) {
    this.state = 'FAULT';
    this.reason = reason;
    this.output = null;
    return this.snapshot();
  }
  stop() {
    this.reset();
    this.reason = 'stop';
    return this.snapshot();
  }
  snapshot() {
    return {
      state: this.state,
      reason: this.reason,
      index: this.index,
      validationEvaluations: this.validationEvaluations,
      output: this.output
        ? {
            q: [...this.output.q],
            velocity: [...this.output.velocity],
            acceleration: [...this.output.acceleration],
          }
        : null,
    };
  }
  checkClock(now) {
    if (!Number.isInteger(now) || now < 0 || now > 0xffffffff) {
      this.fault('invalid_clock');
      return false;
    }
    if (this.lastClock !== null && (now - this.lastClock) >>> 0 >= 0x80000000) {
      this.fault('backward_clock');
      return false;
    }
    this.lastClock = now;
    if (
      ['ARMED', 'RUNNING', 'COMPLETED'].includes(this.state) &&
      (now - this.lastHeartbeat) >>> 0 > 200
    ) {
      this.fault('watchdog');
      return false;
    }
    return true;
  }
  load(path, boxes) {
    if (this.state !== 'DISARMED') return this.fault('load_requires_disarmed');
    const budget = makeBudget();
    try {
      validateBoxes(boxes);
      if (
        !Array.isArray(path) ||
        path.length < 2 ||
        path.length > 64 ||
        !Array.from(path).every(validQ)
      )
        throw Error('Invalid path.');
      if (!poseClear(path[0], boxes, 0, budget)) throw Error('Start collision.');
      for (let i = 1; i < path.length; i++)
        if (!edgeClear(path[i - 1], path[i], boxes, budget)) throw Error('Swept collision.');
      this.reference = trajectory(path);
      this.validationEvaluations = budget.used;
      this.state = 'READY';
      this.reason = 'checked_reference';
      return this.snapshot();
    } catch {
      this.validationEvaluations = budget.used;
      return this.fault('invalid_or_unchecked_path');
    }
  }
  arm(feedback, now) {
    if (!this.checkClock(now)) return this.snapshot();
    if (this.state !== 'READY') return this.fault('arm_requires_ready');
    if (
      !validQ(feedback) ||
      feedback.some((v, i) => Math.abs(v - this.reference.frames[0].q[i]) > 0.01)
    )
      return this.fault('initial_pose_mismatch');
    this.state = 'ARMED';
    this.reason = 'explicit_arm';
    this.lastHeartbeat = now;
    this.output = this.reference.frames[0];
    return this.snapshot();
  }
  start(now) {
    if (!this.checkClock(now)) return this.snapshot();
    if (this.state !== 'ARMED') return this.fault('start_requires_armed');
    this.state = 'RUNNING';
    this.reason = 'explicit_start';
    this.startedAt = now;
    return this.snapshot();
  }
  heartbeat(now) {
    if (!this.checkClock(now)) return this.snapshot();
    if (['ARMED', 'RUNNING', 'COMPLETED'].includes(this.state)) this.lastHeartbeat = now;
    return this.snapshot();
  }
  tick(now, feedback) {
    if (!this.checkClock(now)) return this.snapshot();
    if (!['ARMED', 'RUNNING', 'COMPLETED'].includes(this.state)) return this.snapshot();
    if (!validQ(feedback) || feedback.some((v, i) => Math.abs(v - this.output.q[i]) > 0.05))
      return this.fault('tracking_feedback');
    if (this.state === 'RUNNING') {
      const elapsed = (now - this.startedAt) >>> 0;
      this.index = Math.min(this.reference.frames.length - 1, Math.floor(elapsed / 50));
      this.output = this.reference.frames[this.index];
      if (this.index === this.reference.frames.length - 1) {
        this.state = 'COMPLETED';
        this.reason = 'final_reference_held';
      }
    }
    return this.snapshot();
  }
}
export function idealTrackingDemo(path, boxes) {
  const runner = new ReferenceRunner();
  let state = runner.load(path, boxes);
  if (state.state !== 'READY') return { status: state.state, reason: state.reason, ticks: 0 };
  state = runner.arm(path[0], 0);
  if (state.state !== 'ARMED') return { status: state.state, reason: state.reason, ticks: 0 };
  runner.start(0);
  let feedback = [...path[0]],
    ticks = 0;
  for (let now = 0; now <= 150000; now += 50) {
    runner.heartbeat(now);
    state = runner.tick(now, feedback);
    ticks++;
    if (state.output) feedback = state.output.q;
    if (['FAULT', 'COMPLETED'].includes(state.state)) break;
  }
  return {
    status: state.state,
    reason: state.reason,
    ticks,
    validationEvaluations: runner.validationEvaluations,
    finalPositionError: distance(feedback, path.at(-1)),
    assumption: 'Ideal previous-setpoint feedback, not a physical tracking measurement.',
  };
}
