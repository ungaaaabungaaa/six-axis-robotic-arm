import test from 'node:test';
import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import { fk, MODEL } from '../src/kinematics.mjs';
import { identity, add, apply, mul, rotationError } from '../src/math.mjs';
test('generated URDF joint-chain transforms independently match six-axis FK', async () => {
  const xml = await readFile(new URL('../model/arm.urdf', import.meta.url), 'utf8'),
    joints = [...xml.matchAll(/<joint name="([^"]+)" type="([^"]+)">([\s\S]*?)<\/joint>/g)];
  assert.equal(joints.length, 7);
  const attr = (body, tag, key) =>
      body.match(new RegExp('<' + tag + '[^>]*' + key + '="([^"]+)"'))?.[1],
    vec = (text) => text.split(' ').map(Number);
  for (const q of [
    MODEL.home,
    [0.4, 0.8, -1.1, 0.7, 0.6, -0.2],
    [-0.5, 1.1, -0.8, -0.4, -0.6, 0.9],
  ]) {
    let R = [...identity],
      p = [0, 0, 0],
      parent = 'base',
      index = 0;
    for (const [, name, type, body] of joints) {
      assert.equal(attr(body, 'parent', 'link'), parent);
      parent = attr(body, 'child', 'link');
      p = add(p, apply(R, vec(attr(body, 'origin', 'xyz'))));
      if (type === 'revolute') {
        const [x, y, z] = vec(attr(body, 'axis', 'xyz')),
          angle = q[index++],
          c = Math.cos(angle),
          s = Math.sin(angle),
          t = 1 - c;
        const rotation = [
          t * x * x + c,
          t * x * y - s * z,
          t * x * z + s * y,
          t * x * y + s * z,
          t * y * y + c,
          t * y * z - s * x,
          t * x * z - s * y,
          t * y * z + s * x,
          t * z * z + c,
        ];
        R = mul(R, rotation);
        assert.equal(Number(attr(body, 'limit', 'effort')), 0);
      } else assert.equal(type, 'fixed');
    }
    assert.equal(index, 6);
    assert.equal(parent, 'tcp');
    const expected = fk(q);
    assert.ok(Math.hypot(...p.map((v, i) => v - expected.position[i])) < 1e-9);
    assert.ok(rotationError(R, expected.rotation) < 1e-7);
  }
  assert.equal(
    /<inertial|transmission|ros_control/.test(xml.replace(/<!--[\s\S]*?-->/g, '')),
    false,
  );
});
