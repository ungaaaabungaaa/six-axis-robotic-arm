import test from 'node:test';
import assert from 'node:assert/strict';
import { referenceSingularities } from '../src/diagnostics.mjs';
import { MODEL } from '../src/kinematics.mjs';
import { trajectory } from '../src/motion.mjs';
const frame=(q,time=0)=>({q,time});
test('reference diagnostics distinguish regular and known model singular samples',()=>{
 const regular=referenceSingularities([frame(MODEL.home)]);
 assert.deepEqual(regular.nearSamples,{wrist:0,elbow:0,baseAxis:0});
 const q=[0,Math.PI/2,0,.2,0,.1];
 const singular=referenceSingularities([frame(q)]);
 assert.deepEqual(singular.nearSamples,{wrist:1,elbow:1,baseAxis:1});
 assert.ok(singular.minimumAbs.wristSin<1e-7);
 assert.ok(singular.minimumAbs.elbowSin<1e-7);
 assert.ok(singular.minimumAbs.baseRadiusM<1e-6);
});
test('adjacent sign changes bracket singularities even without an exact zero sample',()=>{
 const a=[0,.8,-1,.2,-.2,0],b=[0,.8,-1,.2,.2,0];
 const report=referenceSingularities([frame(a),frame(b,1)]);
 assert.equal(report.nearSamples.wrist,0);
 assert.deepEqual(report.crossings,[{kind:'wrist',fromIndex:0,toIndex:1,fromTime:0,toTime:1}]);
 const reference=trajectory([a,b]);
 const sampled=referenceSingularities(reference.frames);
 assert.equal(sampled.samples,reference.frames.length);
 assert.ok(sampled.nearSamples.wrist>0||sampled.crossings.some(item=>item.kind==='wrist'));
});
test('diagnostics enforce finite dense joint/time inputs and the reference work limit',()=>{
 for(const frames of [[],Array(2),[frame(Array(6))],[frame(MODEL.home,-1)],[frame(MODEL.home),frame(MODEL.home,0)],Array(3002).fill(frame(MODEL.home))])
  assert.throws(()=>referenceSingularities(frames));
});
