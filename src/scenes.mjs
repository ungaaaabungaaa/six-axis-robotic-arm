// SPDX-License-Identifier: Apache-2.0
import { fk } from './kinematics.mjs';
const normal = fk([0.7, 0.8, -1, 0.4, 0.6, -0.3]),
  singular = fk([0.3, 0.9, -0.7, 1.2, 0, -0.8]),
  fixture = fk([1.58, 1.14, -1.4, 0.33, 0.6, -0.2]);
export const SCENES = Object.freeze({
  inspection: {
    name: 'Inspection pose',
    description: 'Place and orient a tool in a clear workspace.',
    target: { position: normal.position, rotation: normal.rotation },
    boxes: [],
  },
  fixture: {
    name: 'Reach around a fixture',
    description: 'A raised fixture can block direct and staged joint routes.',
    target: { position: fixture.position, rotation: fixture.rotation },
    boxes: [{ min: [0.14, 0.184, 0.344], max: [0.233, 0.277, 0.436] }],
  },
  blocked: {
    name: 'Blocked tool pose',
    description: 'The requested TCP lies inside an obstacle. No selected branch may enter it.',
    target: { position: normal.position, rotation: normal.rotation },
    boxes: [
      { min: normal.position.map((v) => v - 0.045), max: normal.position.map((v) => v + 0.045) },
    ],
  },
  far: {
    name: 'Beyond reach',
    description: 'A valid rigid pose lies beyond the maximum arm reach.',
    target: { position: [1, 0, 0.4], rotation: [1, 0, 0, 0, 1, 0, 0, 0, 1] },
    boxes: [],
  },
  singular: {
    name: 'Aligned wrist',
    description:
      'Two wrist roll axes align; IK reports representative members of a coupled family.',
    target: { position: singular.position, rotation: singular.rotation },
    boxes: [],
  },
});
export function getScene(name) {
  if (!Object.hasOwn(SCENES, name)) throw Error('Unknown scene.');
  return SCENES[name];
}
