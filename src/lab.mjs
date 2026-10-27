// SPDX-License-Identifier: Apache-2.0
import { MODEL, ik, validatePose } from './kinematics.mjs';
import { planMotion, trajectory } from './motion.mjs';
import { idealTrackingDemo } from './controller.mjs';
import { getScene } from './scenes.mjs';
import { apply, sub, distance } from './math.mjs';
export function runLab({
  scene = 'inspection',
  target = null,
  branch = 0,
  seed = 1,
  poseBudget = 30000,
  dt = 0.05,
} = {}) {
  const setup = getScene(scene);
  if (
    !Number.isInteger(seed) ||
    seed < 0 ||
    seed > 0xffffffff ||
    !Number.isInteger(branch) ||
    branch < 0 ||
    branch > 31 ||
    !Number.isInteger(poseBudget) ||
    poseBudget < 1 ||
    poseBudget > 30000 ||
    !Number.isFinite(dt) ||
    dt < 0.02 ||
    dt > 0.1
  )
    throw Error('Invalid bounded laboratory configuration.');
  const desired = target ?? setup.target;
  validatePose(desired);
  const solutions = ik(desired),
    selected = solutions[branch] ?? null;
  if (solutions.length && !selected) throw Error('Selected IK branch does not exist.');
  const plan = selected
      ? planMotion(MODEL.home, selected.q, setup.boxes, { seed, poseBudget })
      : null,
    reference = plan?.status === 'planned' ? trajectory(plan.path, { dt }) : null,
    control = reference ? idealTrackingDemo(plan.path, setup.boxes) : null;
  const wrist = sub(desired.position, apply(desired.rotation, [MODEL.lengths[2], 0, 0])),
    reach = distance(wrist, [0, 0, MODEL.height]),
    geometric =
      reach > MODEL.lengths[0] + MODEL.lengths[1] + 1e-9 ||
      reach < Math.abs(MODEL.lengths[0] - MODEL.lengths[1]) - 1e-9;
  const metrics = {
    status: solutions.length
      ? plan.status
      : geometric
        ? 'geometric_out_of_reach'
        : 'no_supported_ik',
    ikSolutions: solutions.length,
    branch: selected ? branch : null,
    wristSingular: selected?.wristSingular ?? null,
    positionResidual: selected?.positionResidual ?? null,
    orientationResidual: selected?.orientationResidual ?? null,
    method: plan?.method ?? null,
    collisionEvaluations: plan?.evaluations ?? 0,
    controllerValidationEvaluations: control?.validationEvaluations ?? 0,
    totalPoseEvaluations: (plan?.evaluations ?? 0) + (control?.validationEvaluations ?? 0),
    waypoints: plan?.path?.length ?? 0,
    frames: reference?.frames.length ?? 0,
    durationSeconds: reference?.duration ?? null,
    maxVelocityObserved: reference?.maxVelocityObserved ?? null,
    maxAccelerationObserved: reference?.maxAccelerationObserved ?? null,
    controlStatus: control?.status ?? null,
  };
  const report = {
    schemaVersion: 1,
    scene,
    seed,
    poseBudget,
    dt,
    target: desired,
    obstacles: setup.boxes,
    solutions,
    selected,
    plan,
    reference,
    control,
    metrics,
    assumptions:
      'Ideal six-axis spherical wrist and conservative static geometry. Control uses ideal previous-setpoint feedback, not physical measurements.',
  };
  if (new TextEncoder().encode(JSON.stringify(report)).byteLength > 3 * 1024 * 1024)
    throw Error('Report exceeds3MiB.');
  return report;
}
