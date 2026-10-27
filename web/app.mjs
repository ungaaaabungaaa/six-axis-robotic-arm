// SPDX-License-Identifier: Apache-2.0
import { MODEL, fk } from './src/kinematics.mjs';
import { rpy, apply, add } from './src/math.mjs';
import { SCENES } from './src/scenes.mjs';
const $ = (id) => document.getElementById(id),
  canvas = $('view'),
  ctx = canvas.getContext('2d'),
  RAD = Math.PI / 180;
let report = null,
  index = 0,
  custom = false,
  worker = null,
  playing = false,
  last = 0;
const inputs = ['x', 'y', 'z', 'roll', 'pitch', 'yaw'];
function toRPY(R) {
  const pitch = Math.asin(Math.max(-1, Math.min(1, -R[6])));
  return Math.abs(Math.cos(pitch)) > 1e-7
    ? [Math.atan2(R[7], R[8]), pitch, Math.atan2(R[3], R[0])]
    : [0, pitch, Math.atan2(-R[1], R[4])];
}
function resetBranches() {
  const option = document.createElement('option');
  option.value = '0';
  option.textContent = '0 / closest to home';
  $('branch').replaceChildren(option);
}
function preset() {
  const scene = SCENES[$('scene').value],
    values = [...scene.target.position, ...toRPY(scene.target.rotation).map((v) => v / RAD)];
  inputs.forEach((id, i) => ($(id).value = String(Number(values[i].toFixed(7)))));
  $('scene-help').textContent = scene.description;
  custom = false;
  resetBranches();
}
function current() {
  return (
    report?.reference?.frames[index] ?? {
      time: 0,
      q: MODEL.home,
      velocity: Array(6).fill(0),
      acceleration: Array(6).fill(0),
    }
  );
}
function project([x, y, z]) {
  const a = Number($('azimuth').value) * RAD,
    e = Number($('elevation').value) * RAD;
  return [
    canvas.width * 0.43 + 510 * (-Math.sin(a) * x + Math.cos(a) * y),
    canvas.height * 0.72 -
      510 * (z * Math.cos(e) - (Math.cos(a) * x + Math.sin(a) * y) * Math.sin(e)),
  ];
}
function line(a, b, color, width = 1, dash = []) {
  ctx.beginPath();
  ctx.setLineDash(dash);
  ctx.moveTo(...project(a));
  ctx.lineTo(...project(b));
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.stroke();
  ctx.setLineDash([]);
}
function dot(point, color, radius = 4) {
  ctx.beginPath();
  ctx.arc(...project(point), radius, 0, Math.PI * 2);
  ctx.fillStyle = color;
  ctx.fill();
}
function box(box) {
  const v = [];
  for (let x = 0; x < 2; x++)
    for (let y = 0; y < 2; y++)
      for (let z = 0; z < 2; z++)
        v.push([
          x ? box.max[0] : box.min[0],
          y ? box.max[1] : box.min[1],
          z ? box.max[2] : box.min[2],
        ]);
  for (let i = 0; i < 8; i++)
    for (const bit of [1, 2, 4]) if (!(i & bit)) line(v[i], v[i | bit], '#9ba7af', 1.5);
}
function axes(pose, size, dashed = false) {
  for (const [vector, color] of [
    [[size, 0, 0], '#a54f2c'],
    [[0, size, 0], '#35724f'],
    [[0, 0, size], '#3d63a1'],
  ])
    line(
      pose.position,
      add(pose.position, apply(pose.rotation, vector)),
      color,
      2.5,
      dashed ? [5, 3] : [],
    );
}
function draw() {
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  for (let i = -5; i <= 10; i++) {
    const n = i / 10;
    line([-0.5, n, 0], [1, n, 0], '#edf0f0');
    line([n, -0.5, 0], [n, 1, 0], '#edf0f0');
  }
  box({ min: [-0.06, -0.06, 0], max: [0.06, 0.06, 0.1] });
  if (report) {
    report.obstacles.forEach(box);
    axes(report.target, 0.1, true);
    dot(report.target.position, '#a54f2c', 5);
    if (report.reference)
      for (let i = 1; i <= index; i++)
        line(report.reference.frames[i - 1].tcp, report.reference.frames[i].tcp, '#b0aec4', 2);
  }
  const pose = fk(current().q);
  for (let i = 1; i < pose.points.length; i++)
    line(pose.points[i - 1], pose.points[i], '#3e445f', i === 1 ? 12 : i === 2 ? 10 : 7);
  pose.points.forEach((point) => dot(point, '#3e445f', 7));
  dot(pose.points[2], '#6e628e', 11);
  axes(pose, 0.07);
  ctx.fillStyle = '#65707e';
  ctx.font = '14px Arial';
  ctx.fillText(
    'BASE FRAME / metres / current solid axes · target dashed axes',
    20,
    canvas.height - 20,
  );
}
function row(values) {
  const tr = document.createElement('tr');
  for (const value of values) {
    const td = document.createElement('td');
    td.textContent = String(value);
    tr.append(td);
  }
  return tr;
}
function update() {
  const f = current(),
    pose = fk(f.q);
  $('frame').value = String(index);
  $('time').textContent = f.time.toFixed(2) + ' s';
  $('joints').replaceChildren();
  for (let i = 0; i < 6; i++)
    $('joints').append(
      row([
        ['Base yaw', 'Shoulder', 'Elbow', 'Wrist roll 1', 'Wrist pitch', 'Wrist roll 2'][i],
        f.q[i].toFixed(3),
        f.velocity[i].toFixed(3),
        f.acceleration[i].toFixed(3),
      ]),
    );
  $('tcp').textContent =
    `TCP xyz: ${pose.position.map((v) => v.toFixed(4)).join(', ')} m${report?.reference ? '' : ' · parked model only'}`;
  $('pose').replaceChildren(
    row(['Position (m)', ...pose.position.map((v) => v.toFixed(4))]),
    row(['RPY (deg)', ...toRPY(pose.rotation).map((v) => (v / RAD).toFixed(2))]),
  );
  $('rotation').textContent = [0, 1, 2]
    .map((i) =>
      pose.rotation
        .slice(i * 3, i * 3 + 3)
        .map((v) => v.toFixed(5).padStart(9))
        .join(' '),
    )
    .join('\n');
  draw();
}
function stop() {
  playing = false;
  $('play').textContent = 'Play';
}
function invalidate(message = 'Inputs changed. Run again.') {
  worker?.terminate();
  worker = null;
  stop();
  report = null;
  index = 0;
  $('status').textContent = message;
  $('outcome').textContent = 'No current result. Solve again with these inputs.';
  $('run').disabled = false;
  for (const id of ['play', 'step', 'frame', 'export']) $(id).disabled = true;
  for (const id of ['solutions', 'route', 'duration', 'control']) $(id).textContent = '—';
  $('singularity').textContent = 'No current solution.';
  $('residual').textContent = 'FK residuals will appear after a solution.';
  $('rates').textContent = '';
  $('frame').max = '0';
  update();
}
function load(value) {
  report = value;
  index = 0;
  stop();
  const m = value.metrics;
  $('status').textContent = m.status.replaceAll('_', ' ');
  $('solutions').textContent = `${m.ikSolutions} / ${m.branch ?? 'none'}`;
  $('route').textContent = `${m.method ?? 'none'} / ${m.collisionEvaluations}`;
  $('duration').textContent =
    m.durationSeconds === null ? 'No reference' : m.durationSeconds.toFixed(2) + ' s';
  $('control').textContent = m.controlStatus ?? 'No reference';
  $('singularity').textContent = m.wristSingular
    ? 'Selected pose has an aligned wrist. This solution represents one member of a coupled roll family.'
    : value.selected
      ? 'Selected pose has no aligned-wrist flag. This is not a full conditioning or physical stability assessment.'
      : 'No supported IK solution was selected.';
  $('residual').textContent = value.selected
    ? `Selected IK→FK residual: ${(m.positionResidual * 1000).toExponential(2)} mm / ${(m.orientationResidual / RAD).toExponential(2)} degrees.`
    : 'No residual is available.';
  $('rates').textContent =
    m.maxVelocityObserved === null
      ? ''
      : `Peak joint rate ${m.maxVelocityObserved.toFixed(3)} rad/s; acceleration ${m.maxAccelerationObserved.toFixed(3)} rad/s². Ideal control completion is not hardware tracking evidence.`;
  const messages = {
    planned:
      'A full checked joint route is available. The ideal host runner completed its reference with previous-setpoint feedback.',
    goal_in_collision:
      'The selected inverse solution is in collision. No reference was emitted. Other branch choices require their own check.',
    start_in_collision:
      'The model home pose intersects supplied geometry. No reference was emitted.',
    planning_budget:
      'The finite search/work budget was reached. No reference was emitted; this does not prove that no route exists.',
    geometric_out_of_reach:
      'The wrist centre is outside the two-link geometric reach annulus. No reference was emitted.',
    no_supported_ik:
      'No supported inverse solution was found within the model limits and singular representatives. This is not a proof of general unreachability.',
  };
  $('outcome').textContent = messages[m.status] ?? m.status;
  const selected = String(m.branch ?? 0);
  $('branch').replaceChildren();
  if (value.solutions.length)
    value.solutions.forEach((solution, i) => {
      const option = document.createElement('option');
      option.value = String(i);
      option.textContent = `${i} / ${solution.wristSingular ? 'aligned wrist' : 'pose branch'}${i === 0 ? ' · closest home' : ''}`;
      $('branch').append(option);
    });
  else resetBranches();
  $('branch').value = selected;
  for (const id of ['play', 'step', 'frame']) $(id).disabled = !value.reference;
  $('export').disabled = false;
  $('frame').max = String((value.reference?.frames.length ?? 1) - 1);
  update();
}
$('config').addEventListener('submit', (event) => {
  event.preventDefault();
  invalidate('Solving pose…');
  $('run').disabled = true;
  worker = new Worker('/worker.mjs', { type: 'module' });
  worker.onmessage = ({ data }) => {
    $('run').disabled = false;
    worker?.terminate();
    worker = null;
    if (data.error) {
      $('status').textContent = 'Run rejected';
      $('outcome').textContent = data.error;
      return;
    }
    load(data.report);
  };
  worker.onerror = () => {
    worker?.terminate();
    worker = null;
    $('run').disabled = false;
    $('status').textContent = 'Worker failed';
    $('outcome').textContent = 'The local worker could not run. Check the served module files.';
  };
  worker.postMessage({
    scene: $('scene').value,
    target: custom
      ? {
          position: ['x', 'y', 'z'].map((id) => Number($(id).value)),
          rotation: rpy(...['roll', 'pitch', 'yaw'].map((id) => Number($(id).value) * RAD)),
        }
      : null,
    branch: Number($('branch').value),
    seed: Number($('seed').value),
    poseBudget: Number($('budget').value),
  });
});
$('scene').addEventListener('change', () => {
  preset();
  invalidate();
});
inputs.forEach((id) =>
  $(id).addEventListener('input', () => {
    custom = true;
    resetBranches();
    invalidate();
  }),
);
for (const id of ['seed', 'budget']) $(id).addEventListener('input', () => invalidate());
$('branch').addEventListener('change', () => invalidate());
for (const id of ['azimuth', 'elevation']) $(id).addEventListener('input', draw);
$('frame').addEventListener('input', () => {
  stop();
  index = Number($('frame').value);
  update();
});
$('step').addEventListener('click', () => {
  stop();
  index = Math.min(index + 1, report.reference.frames.length - 1);
  update();
});
$('play').addEventListener('click', () => {
  if (playing) {
    stop();
    return;
  }
  if (index === report.reference.frames.length - 1) index = 0;
  playing = true;
  last = 0;
  $('play').textContent = 'Pause';
});
function animate(now) {
  if (playing) {
    if (!last) last = now;
    if (now - last >= 50) {
      const count = Math.floor((now - last) / 50);
      index = Math.min(index + count, report.reference.frames.length - 1);
      last += count * 50;
      update();
      if (index === report.reference.frames.length - 1) stop();
    }
  }
  requestAnimationFrame(animate);
}
requestAnimationFrame(animate);
$('export').addEventListener('click', () => {
  const url = URL.createObjectURL(
      new Blob([JSON.stringify(report, null, 2) + '\n'], { type: 'application/json' }),
    ),
    a = document.createElement('a');
  a.href = url;
  a.download = `six-axis-${report.scene}-seed-${report.seed}.json`;
  a.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
});
preset();
update();
