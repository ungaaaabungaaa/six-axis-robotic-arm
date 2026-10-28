// SPDX-License-Identifier: Apache-2.0
import { MODEL, validQ } from './kinematics.mjs';
export function referenceSingularities(frames) {
  if (!Array.isArray(frames) || frames.length < 1 || frames.length > 3001)
    throw Error('Diagnostics require 1–3001 reference frames.');
  const thresholds = { wrist: 1e-7, elbow: 1e-7, baseAxis: 1e-6 },
    nearSamples = { wrist: 0, elbow: 0, baseAxis: 0 },
    minima = { wrist: Infinity, elbow: Infinity, baseAxis: Infinity },
    crossings = [];
  let previous = null, lastTime = -1;
  for (let index = 0; index < frames.length; index++) {
    const frame = frames[index];
    if (!frame || !validQ(frame.q) || !Number.isFinite(frame.time) || frame.time < 0 || frame.time <= lastTime)
      throw Error('Diagnostics require dense joint-limited frames with increasing finite times.');
    const q = frame.q,
      factors = {
        wrist: Math.sin(q[4]),
        elbow: Math.sin(q[2]),
        baseAxis: MODEL.lengths[0] * Math.cos(q[1]) + MODEL.lengths[1] * Math.cos(q[1] + q[2]),
      };
    for (const kind of Object.keys(factors)) {
      const value = factors[kind];
      minima[kind] = Math.min(minima[kind], Math.abs(value));
      if (Math.abs(value) <= thresholds[kind]) nearSamples[kind]++;
      if (previous && previous[kind] * value < 0)
        crossings.push({ kind, fromIndex: index - 1, toIndex: index, fromTime: lastTime, toTime: frame.time });
    }
    previous = factors;
    lastTime = frame.time;
  }
  return {
    samples: frames.length,
    thresholds: { wristSin: thresholds.wrist, elbowSin: thresholds.elbow, baseRadiusM: thresholds.baseAxis },
    minimumAbs: { wristSin: minima.wrist, elbowSin: minima.elbow, baseRadiusM: minima.baseAxis },
    nearSamples,
    crossings,
    limitation: 'Ideal spherical-wrist model factors at sampled poses plus sign-change brackets. Tangent contacts between samples can be missed. This is not a Jacobian condition number, inverse-velocity solution, continuous nonsingularity proof or physical safety certificate.',
  };
}
