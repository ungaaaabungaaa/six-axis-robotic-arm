// SPDX-License-Identifier: Apache-2.0
import { writeFile } from 'node:fs/promises';
import { pathToFileURL } from 'node:url';
import { runLab } from '../src/lab.mjs';
import { fk } from '../src/kinematics.mjs';
import { rpy } from '../src/math.mjs';
export async function main(args) {
  if (args.length === 0 || args[0] === '--help')
    return 'Usage: node scripts/cli.mjs solve [--scene inspection] [--seed 1] [--branch 0] [--budget 30000] [--output new.json]\nCustom pose: add all --x --y --z metres and --roll --pitch --yaw degrees.\nForward pose: fk --q q0,q1,q2,q3,q4,q5 (radians). No device operations.\n';
  const command = args.shift();
  if (!['solve', 'fk'].includes(command)) throw Error('Unknown command. Use --help.');
  const allowed =
      command === 'fk'
        ? ['q', 'output']
        : [
            'scene',
            'seed',
            'branch',
            'budget',
            'dt',
            'output',
            'x',
            'y',
            'z',
            'roll',
            'pitch',
            'yaw',
          ],
    options = {};
  while (args.length) {
    const flag = args.shift(),
      key = flag?.slice(2),
      value = args.shift();
    if (
      !flag?.startsWith('--') ||
      !allowed.includes(key) ||
      Object.hasOwn(options, key) ||
      value === undefined ||
      !value.length
    )
      throw Error('Unknown, duplicate or missing option.');
    options[key] = value;
  }
  let result;
  if (command === 'fk') {
    if (options.q === undefined) throw Error('--q with six radians is required.');
    result = fk(options.q.split(',').map(Number));
  } else {
    const fields = ['x', 'y', 'z', 'roll', 'pitch', 'yaw'],
      present = fields.filter((key) => Object.hasOwn(options, key));
    if (present.length && present.length !== 6) throw Error('Supply all six custom pose values.');
    const target = present.length
      ? {
          position: fields.slice(0, 3).map((key) => Number(options[key])),
          rotation: rpy(...fields.slice(3).map((key) => (Number(options[key]) * Math.PI) / 180)),
        }
      : null;
    result = runLab({
      scene: options.scene ?? 'inspection',
      target,
      seed: Number(options.seed ?? 1),
      branch: Number(options.branch ?? 0),
      poseBudget: Number(options.budget ?? 30000),
      dt: Number(options.dt ?? 0.05),
    });
  }
  const value = JSON.stringify(result, null, 2) + '\n';
  if (options.output) {
    await writeFile(options.output, value, { flag: 'wx' });
    return 'Created ' + options.output + '\n';
  }
  return value;
}
if (process.argv[1] && import.meta.url === pathToFileURL(process.argv[1]).href) {
  try {
    process.stdout.write(await main(process.argv.slice(2)));
  } catch (error) {
    process.stderr.write(error.message + '\n');
    process.exitCode = 1;
  }
}
