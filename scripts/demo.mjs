// SPDX-License-Identifier: Apache-2.0
import { writeFile, readFile } from 'node:fs/promises';
import { runLab } from '../src/lab.mjs';
const check = process.argv.slice(2).join(' ') === '--check';
if (process.argv.length > 2 && !check) throw Error('Only --check is supported.');
const records = [];
for (const scene of ['inspection', 'fixture', 'blocked', 'far', 'singular'])
  for (const seed of [1, 2, 3]) {
    const r = runLab({ scene, seed });
    records.push({
      scene,
      seed,
      ...Object.fromEntries(
        Object.entries(r.metrics).map(([key, value]) => [
          key,
          typeof value === 'number' ? Number(value.toFixed(6)) : value,
        ]),
      ),
    });
  }
const json =
    JSON.stringify(
      {
        schemaVersion: 1,
        note: 'Actual ideal-model runs. Reference time is not compute time; control completion uses ideal previous-setpoint feedback. Null motion metrics mean no reference.',
        configuration: {
          seeds: [1, 2, 3],
          branch: 0,
          plannerPoseBudget: 30000,
          controllerValidationBudget: 30000,
          dt: 0.05,
        },
        records,
      },
      null,
      2,
    ) + '\n',
  keys = Object.keys(records[0]),
  csv =
    keys.join(',') +
    '\n' +
    records.map((r) => keys.map((k) => r[k] ?? '').join(',')).join('\n') +
    '\n';
let svg =
  '<svg xmlns="http://www.w3.org/2000/svg" width="1080" height="810" viewBox="0 0 1080 810" role="img" aria-labelledby="title desc"><title id="title">Six-axis arm: fifteen measured model runs</title><desc id="desc">Three inspection direct routes, three fixture RRT routes, three blocked tool poses, three geometric out-of-reach poses and three singular wrist routes. Bars show reference duration only.</desc><rect width="1080" height="810" fill="#f6f5ef"/><g font-family="monospace" fill="#303849"><text x="30" y="38" font-size="24">AXIS / Full-pose reference runs</text><text x="30" y="67" font-size="13">5 scenes × 3 seeds · branch 0 · 30000 planning + 30000 validation cap · no hardware</text><text x="30" y="105" font-size="13">SCENE / SEED</text><text x="260" y="105" font-size="13">REFERENCE SECONDS</text><text x="650" y="105" font-size="13">OUTCOME / PLANNER CHECKS</text>';
records.forEach((r, i) => {
  const y = 137 + i * 40;
  svg += `<text x="30" y="${y}" font-size="13">${r.scene} / ${r.seed}</text>`;
  if (r.durationSeconds !== null)
    svg += `<rect x="260" y="${y - 13}" width="${r.durationSeconds * 13}" height="18" fill="${r.method === 'rrt' ? '#796895' : '#424b67'}"/><text x="${270 + r.durationSeconds * 13}" y="${y}" font-size="12">${r.durationSeconds.toFixed(2)}</text>`;
  else svg += `<text x="260" y="${y}" font-size="12">No reference</text>`;
  svg += `<text x="650" y="${y}" font-size="12">${r.status} / ${r.collisionEvaluations}</text>`;
});
svg +=
  '<text x="30" y="786" font-size="12">Kinematic output and ideal feedback only. Singular representatives and finite search are limited.</text></g></svg>\n';
for (const [name, value] of [
  ['reports/scenarios.json', json],
  ['reports/scenarios.csv', csv],
  ['docs/images/measured-runs.svg', svg],
]) {
  const url = new URL('../' + name, import.meta.url);
  if (check) {
    if ((await readFile(url, 'utf8')) !== value) throw Error('Recorded result changed: ' + name);
  } else await writeFile(url, value);
}
console.log(`${check ? 'Reproduced' : 'Recorded'} all15 measured model runs, CSV and SVG.`);
