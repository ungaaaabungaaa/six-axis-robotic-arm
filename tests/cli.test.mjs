import test from 'node:test';
import assert from 'node:assert/strict';
import { mkdtemp, writeFile, readFile, rm } from 'node:fs/promises';
import os from 'node:os';
import path from 'node:path';
import { main } from '../scripts/cli.mjs';
test('CLI full-pose and FK modes validate units/fields and preserve existing exports', async () => {
  assert.match(await main(['--help']), /No device operations/);
  const result = JSON.parse(await main(['solve', '--scene', 'inspection']));
  assert.equal(result.metrics.status, 'planned');
  const zero = JSON.parse(await main(['fk', '--q', '0,0,0,0,0,0']));
  assert.ok(Math.abs(zero.position[0] - 0.65) < 1e-9);
  for (const args of [
    ['solve', '--x', '.4'],
    ['solve', '--seed', 'NaN'],
    ['solve', '--seed', '1', '--seed', '2'],
    ['solve', '--unknown', 'x'],
    ['fk', '--q', '1,2,3'],
    ['fk', '--q', '0,0,0,0,0,NaN'],
  ])
    await assert.rejects(main(args));
  const dir = await mkdtemp(path.join(os.tmpdir(), 'six-axis-cli-'));
  try {
    const out = path.join(dir, 'saved.json');
    await writeFile(out, 'keep');
    await assert.rejects(main(['solve', '--output', out]));
    assert.equal(await readFile(out, 'utf8'), 'keep');
  } finally {
    await rm(dir, { recursive: true, force: true });
  }
});
test('solve reports reference singularities and keeps failed paths explicitly null',async()=>{
 const singular=JSON.parse(await main(['solve','--scene','singular']));
 assert.equal(singular.singularityDiagnostics.samples,singular.reference.frames.length);
 assert.ok(singular.singularityDiagnostics.nearSamples.wrist>0);
 assert.match(singular.singularityDiagnostics.limitation,/not a Jacobian condition number/);
 const failed=JSON.parse(await main(['solve','--scene','far']));
 assert.equal(failed.singularityDiagnostics,null);
});
