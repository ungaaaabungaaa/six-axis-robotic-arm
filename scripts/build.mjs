// SPDX-License-Identifier: Apache-2.0
import { mkdir, rm, copyFile } from 'node:fs/promises';
const root = new URL('../', import.meta.url),
  out = new URL('../dist/', import.meta.url);
await rm(out, { recursive: true, force: true });
await mkdir(new URL('src/', out), { recursive: true });
for (const name of ['index.html', 'style.css', 'app.mjs', 'worker.mjs'])
  await copyFile(new URL('web/' + name, root), new URL(name, out));
for (const name of [
  'math.mjs',
  'kinematics.mjs',
  'motion.mjs',
  'controller.mjs',
  'scenes.mjs',
  'lab.mjs',
  'diagnostics.mjs',
])
  await copyFile(new URL('src/' + name, root), new URL('src/' + name, out));
console.log('Built self-contained static lab in dist/.');
