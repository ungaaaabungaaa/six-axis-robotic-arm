// SPDX-License-Identifier: Apache-2.0
// Geometry and timing adapted from the owned LiDAR arm bench; see NOTICE.
import { MODEL, validQ, fk } from './kinematics.mjs';
import { dot, sub, add, scale, distance, clamp } from './math.mjs';
const EPS = 1e-9;
export function validateBoxes(boxes) {
  if (!Array.isArray(boxes) || boxes.length > 12)
    throw Error('At most12 conservative obstacle boxes are supported.');
  for (const box of boxes)
    if (
      !box ||
      !Array.isArray(box.min) ||
      !Array.isArray(box.max) ||
      box.min.length !== 3 ||
      box.max.length !== 3 ||
      Array.from(box.min).some(
        (v, i) =>
          !Number.isFinite(v) ||
          !Number.isFinite(box.max[i]) ||
          Math.abs(v) > 2 ||
          Math.abs(box.max[i]) > 2 ||
          v >= box.max[i],
      )
    )
      throw Error('Obstacle boxes require finite ordered xyz bounds within±2m.');
  return boxes;
}
class BudgetLimit extends Error {}
export function makeBudget(limit = 30000) {
  if (!Number.isInteger(limit) || limit < 1 || limit > 30000)
    throw Error('Pose budget must be1..30000.');
  return { used: 0, limit };
}
function spend(budget) {
  if (budget) {
    if (budget.used >= budget.limit) throw new BudgetLimit('Pose evaluation budget reached.');
    budget.used++;
  }
}
export function segmentBox(a, b, box, radius) {
  let lo = 0,
    hi = 1;
  for (let axis = 0; axis < 3; axis++) {
    const d = b[axis] - a[axis],
      min = box.min[axis] - radius,
      max = box.max[axis] + radius;
    if (Math.abs(d) < EPS) {
      if (a[axis] < min - EPS || a[axis] > max + EPS) return false;
    } else {
      let first = (min - a[axis]) / d,
        last = (max - a[axis]) / d;
      if (first > last) [first, last] = [last, first];
      lo = Math.max(lo, first);
      hi = Math.min(hi, last);
      if (lo > hi + EPS) return false;
    }
  }
  return true;
}
export function segmentDistance(p1, q1, p2, q2) {
  const d1 = sub(q1, p1),
    d2 = sub(q2, p2),
    r = sub(p1, p2),
    a = dot(d1, d1),
    e = dot(d2, d2),
    f = dot(d2, r);
  let s = 0,
    t = 0;
  if (a <= EPS && e <= EPS) return distance(p1, p2);
  if (a <= EPS) t = clamp(f / e, 0, 1);
  else {
    const c = dot(d1, r);
    if (e <= EPS) s = clamp(-c / a, 0, 1);
    else {
      const b = dot(d1, d2),
        denominator = a * e - b * b;
      s = Math.abs(denominator) > EPS ? clamp((b * f - c * e) / denominator, 0, 1) : 0;
      t = (b * s + f) / e;
      if (t < 0) {
        t = 0;
        s = clamp(-c / a, 0, 1);
      } else if (t > 1) {
        t = 1;
        s = clamp((b - c) / a, 0, 1);
      }
    }
  }
  return distance(add(p1, scale(d1, s)), add(p2, scale(d2, t)));
}
export function poseClear(q, boxes, margin = 0, budget) {
  spend(budget);
  if (!validQ(q) || !Number.isFinite(margin) || margin < 0) return false;
  validateBoxes(boxes);
  const points = fk(q).points,
    segments = [
      [points[0], points[1]],
      [points[1], points[2]],
      [points[2], points[3]],
      [points[2], points[2]],
      [points[3], points[3]],
    ],
    fixture = { min: [-0.06, -0.06, 0], max: [0.06, 0.06, 0.1] };
  for (let i = 0; i < segments.length; i++) {
    const radius = MODEL.radii[i] + margin;
    if (Math.min(segments[i][0][2], segments[i][1][2]) - radius <= 0) return false;
    for (const box of [...boxes, fixture])
      if (segmentBox(...segments[i], box, radius)) return false;
  }
  for (const [a, b] of [
    [0, 2],
    [0, 3],
    [0, 4],
    [1, 4],
  ])
    if (
      segmentDistance(...segments[a], ...segments[b]) <=
      MODEL.radii[a] + MODEL.radii[b] + 2 * margin + EPS
    )
      return false;
  return true;
}
export function edgeClear(start, end, boxes, budget = makeBudget()) {
  if (!validQ(start) || !validQ(end)) return false;
  const delta = end.map((v, i) => v - start[i]),
    bound =
      0.65 * (Math.abs(delta[0]) + Math.abs(delta[1])) +
      0.35 * Math.abs(delta[2]) +
      0.1 * (Math.abs(delta[3]) + Math.abs(delta[4])),
    pieces = Math.max(1, Math.ceil(bound / 0.008));
  if (pieces > 512) throw new BudgetLimit('Edge subdivision bound reached.');
  const margin = bound / (2 * pieces);
  for (let i = 0; i < pieces; i++) {
    const q = start.map((v, j) => v + (delta[j] * (i + 0.5)) / pieces);
    if (!poseClear(q, boxes, margin, budget)) return false;
  }
  return true;
}
function random(seed) {
  let state = seed >>> 0;
  for (let i = 0; i < 8; i++) state = (Math.imul(state, 1664525) + 1013904223) >>> 0;
  return () => {
    state = (Math.imul(state, 1664525) + 1013904223) >>> 0;
    return state / 4294967296;
  };
}
function route(tree, index) {
  const path = [];
  while (index !== -1) {
    path.push(tree[index].q);
    index = tree[index].parent;
  }
  return path.reverse();
}
export function planMotion(start, goal, boxes, options = {}) {
  if (!validQ(start) || !validQ(goal)) throw Error('Planning endpoints violate joint limits.');
  validateBoxes(boxes);
  if (Object.keys(options).some((key) => !['seed', 'poseBudget', 'iterations'].includes(key)))
    throw Error('Unknown planner option.');
  const seed = options.seed ?? 1,
    iterations = options.iterations ?? 500;
  if (
    !Number.isInteger(seed) ||
    seed < 0 ||
    seed > 4294967295 ||
    !Number.isInteger(iterations) ||
    iterations < 1 ||
    iterations > 1000
  )
    throw Error('Invalid planner seed/iteration bound.');
  const budget = makeBudget(options.poseBudget ?? 30000);
  let path = null,
    method = null;
  try {
    if (!poseClear(start, boxes, 0, budget))
      return { status: 'start_in_collision', path: null, method: null, evaluations: budget.used };
    if (!poseClear(goal, boxes, 0, budget))
      return { status: 'goal_in_collision', path: null, method: null, evaluations: budget.used };
    if (edgeClear(start, goal, boxes, budget)) {
      path = [start, goal];
      method = 'direct';
    }
    if (!path) {
      const lifted = [
        start,
        [start[0], Math.PI / 2, -Math.PI / 2, ...start.slice(3)],
        [goal[0], Math.PI / 2, -Math.PI / 2, ...start.slice(3)],
        [goal[0], Math.PI / 2, -Math.PI / 2, ...goal.slice(3)],
        goal,
      ];
      if (lifted.slice(1).every((q, i) => edgeClear(lifted[i], q, boxes, budget))) {
        path = lifted;
        method = 'lift';
      }
    }
    if (!path) {
      const rng = random(seed);
      let first = [{ q: [...start], parent: -1 }],
        second = [{ q: [...goal], parent: -1 }],
        swapped = false;
      const extend = (tree, target) => {
        let nearest = 0;
        for (let i = 1; i < tree.length; i++)
          if (distance(tree[i].q, target) < distance(tree[nearest].q, target)) nearest = i;
        const d = distance(tree[nearest].q, target),
          q = tree[nearest].q.map((v, j) => v + (target[j] - v) * Math.min(1, 0.25 / (d || 1)));
        if (first.length + second.length >= 1200 || !edgeClear(tree[nearest].q, q, boxes, budget))
          return null;
        tree.push({ q, parent: nearest });
        return { index: tree.length - 1, reached: d <= 0.25 };
      };
      for (let iteration = 0; iteration < iterations && !path; iteration++) {
        const sample =
            rng() < 0.2 ? second[0].q : MODEL.limits.map(([lo, hi]) => lo + (hi - lo) * rng()),
          grown = extend(first, sample);
        if (grown) {
          for (let step = 0; step < 12; step++) {
            const other = extend(second, first[grown.index].q);
            if (!other) break;
            if (other.reached) {
              const a = route(first, grown.index),
                b = route(second, other.index).reverse();
              path = swapped ? [...b.reverse(), ...a.reverse().slice(1)] : [...a, ...b.slice(1)];
              method = 'rrt';
              break;
            }
          }
        }
        [first, second] = [second, first];
        swapped = !swapped;
      }
    }
    if (!path)
      return { status: 'planning_budget', path: null, method: null, evaluations: budget.used };
    if (path.length > 64) throw new BudgetLimit('Waypoint bound reached.');
    for (let i = 1; i < path.length; i++)
      if (!edgeClear(path[i - 1], path[i], boxes, budget))
        throw Error('Final path failed swept validation.');
    return { status: 'planned', path: path.map((q) => [...q]), method, evaluations: budget.used };
  } catch (error) {
    if (error instanceof BudgetLimit)
      return { status: 'planning_budget', path: null, method: null, evaluations: budget.used };
    throw error;
  }
}
export function trajectory(path, { dt = 0.05, maxVelocity = 1, maxAcceleration = 2 } = {}) {
  if (
    !Array.isArray(path) ||
    path.length < 2 ||
    path.length > 64 ||
    !Array.from(path).every(validQ) ||
    !Number.isFinite(dt) ||
    dt < 0.02 ||
    dt > 0.1 ||
    !Number.isFinite(maxVelocity) ||
    maxVelocity < 0.2 ||
    maxVelocity > 1 ||
    !Number.isFinite(maxAcceleration) ||
    maxAcceleration < 0.5 ||
    maxAcceleration > 2
  )
    throw Error('Invalid bounded trajectory settings.');
  const segments = [];
  let count = 1;
  for (let i = 1; i < path.length; i++) {
    const delta = path[i].map((v, j) => v - path[i - 1][j]),
      distance = Math.max(...delta.map(Math.abs)),
      duration = Math.max(
        0.1,
        (1.875 * distance) / maxVelocity,
        Math.sqrt((5.773503 * distance) / maxAcceleration),
      ),
      ticks = Math.ceil(duration / dt);
    count += ticks;
    if (count > 3001) throw Error('Trajectory exceeds3001frames.');
    segments.push({ start: path[i - 1], delta, ticks, duration: ticks * dt });
  }
  const frames = [
    {
      time: 0,
      q: [...path[0]],
      velocity: [0, 0, 0, 0, 0, 0],
      acceleration: [0, 0, 0, 0, 0, 0],
      tcp: fk(path[0]).position,
    },
  ];
  let time = 0;
  for (const segment of segments) {
    for (let i = 1; i <= segment.ticks; i++) {
      const u = i / segment.ticks,
        s = 10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5,
        ds = (30 * u ** 2 - 60 * u ** 3 + 30 * u ** 4) / segment.duration,
        dds = (60 * u - 180 * u ** 2 + 120 * u ** 3) / segment.duration ** 2,
        q = segment.start.map((v, j) => v + segment.delta[j] * s);
      frames.push({
        time: time + i * dt,
        q,
        velocity: segment.delta.map((v) => v * ds),
        acceleration: segment.delta.map((v) => v * dds),
        tcp: fk(q).position,
      });
    }
    time += segment.duration;
  }
  return {
    frames,
    duration: time,
    maxVelocityObserved: Math.max(...frames.flatMap((frame) => frame.velocity.map(Math.abs))),
    maxAccelerationObserved: Math.max(
      ...frames.flatMap((frame) => frame.acceleration.map(Math.abs)),
    ),
  };
}
