const assert=require('node:assert/strict');
const {Translation,compose}=require('./hand-translation.js');
const I=[[1,0,0],[0,1,0],[0,0,1]],Z=[0,0,0];
function calibrated(a=[0,0,1],r=I){const t=new Translation();for(let i=0;i<=70;i++)t.update(a,Z,r,i*1000/60,String(i));assert(t.ready);return t;}
let t=calibrated();for(let i=71;i<671;i++)t.update([0,0,1],Z,I,i*1000/60,String(i));assert.deepEqual(t.position,Z);assert.deepEqual(t.velocity,Z);
// Known 1 m/s² for 0.2 seconds: un-damped ideal displacement is 20 mm.
t=calibrated();for(let i=71;i<=82;i++)t.update([1/9.80665,0,1],Z,I,i*1000/60,String(i));assert(t.position[0]>.017&&t.position[0]<.021);
const displaced=t.position[0];for(let i=83;i<150;i++)t.update([0,0,1],Z,I,i*1000/60,String(i));assert.deepEqual(t.velocity,Z);assert(t.position[0]>=displaced);
// A tilted stationary board and gyro bias must not produce translation.
const R=[[1,0,0],[0,0,-1],[0,1,0]];t=calibrated([0,1,0],R);for(let i=71;i<300;i++)t.update([0,1,0],Z,R,i*1000/60,String(i));assert(Math.hypot(...t.position)<1e-12);
// Duplicate readings and long timing gaps must not integrate acceleration.
const p=t.position.slice();t.update([5,0,1],Z,I,5000,'299');assert.deepEqual(t.position,p);t.update([5,0,1],Z,I,8000,'300');assert.deepEqual(t.position,p);assert.deepEqual(t.velocity,Z);
// Common rigid transform applies equally to every available finger.
for(let i=1;i<=5;i++)assert.deepEqual(compose([1,2,3],R,[i,1,2]),[1+i,0,4]);
t.reset();assert.deepEqual(t.position,Z);assert(!t.ready);
// Moving during initialization must not silently establish a stationary bias.
t=new Translation();for(let i=0;i<150;i++)t.update([0,0,1],[20,0,0],I,i*1000/60,String(i));assert(!t.ready);
console.log('PASS: rest, pulse trajectory, tilt gravity compensation, duplicate/gap rejection, all-five transforms, recenter and calibration motion rejection');
// Slow acceleration after a long rest must release the rest latch.
t=calibrated();for(let i=71;i<150;i++)t.update([0,0,1],Z,I,i*1000/60,String(i));for(let i=150;i<180;i++)t.update([.1/9.80665,0,1],Z,I,i*1000/60,String(i));assert(t.position[0]>.008);
// Gravity-only rotation plus constant body-frame bias must not become translation.
t=calibrated([.001,0,1],I);for(let i=71;i<200;i++){const theta=(i-71)*.005,r=[[Math.cos(theta),0,Math.sin(theta)],[0,1,0],[-Math.sin(theta),0,Math.cos(theta)]];t.update([-.0-Math.sin(theta)+.001,0,Math.cos(theta)],[0,17.2,0],r,i*1000/60,String(i));}assert(Math.hypot(...t.position)<1e-10);
// Verify the real orientation-to-translation pipeline doesn't swallow a pulse as tilt.
const {Orientation}=require('./hand-orientation.js');const o=new Orientation();t=new Translation();
for(let i=0;i<210;i++){const a=[i>=150&&i<180?.1/9.80665:0,0,1],now=i*1000/60,r=o.update(a,Z,now,!t.ready||t.rest>=.6);t.update(a,Z,r.matrix,now,String(i));}
assert(t.position[0]>.008);assert(t.ready);console.log('PASS: gentle motion leaves rest, rotating body bias is compensated, coupled orientation/translation pulse produces displacement');

// A stationary sensor with a persistent 1-degree gravity-estimate error must
// stop integrating after raw samples have remained quiet, rather than run away.
t=calibrated();const e=Math.PI/180,badR=[[Math.cos(e),0,Math.sin(e)],[0,1,0],[-Math.sin(e),0,Math.cos(e)]];
for(let i=71;i<400;i++)t.update([0,0,1],Z,badR,i*1000/60,String(i));
assert.deepEqual(t.velocity,Z);assert(Math.hypot(...t.position)<.06);
// Prolonged unconstrained movement is explicitly held, not silently clamped.
t=calibrated();for(let i=71;i<250;i++)t.update([.1,0,1],[0,0,10],I,i*1000/60,String(i));assert(t.held);const held=t.position.slice();t.update([.5,0,1],[0,0,10],I,250*1000/60,'250');assert.deepEqual(t.position,held);assert(t.status.includes('held'));t.reset();assert(!t.held);
console.log('PASS: tilt-error drift stops at sustained rest; unreliable unconstrained motion is explicitly held');
const {MovementMode}=require('./hand-translation.js');let saved=null;const store={getItem:()=>saved,setItem:(k,v)=>{saved=v;}};let mode=new MovementMode(store);assert.equal(mode.mode,'centered');assert.deepEqual(mode.position([20,30,40]),Z);mode.select('moving');assert.deepEqual(mode.position([1,2,3]),[1,2,3]);mode=new MovementMode(store);assert.equal(mode.mode,'moving');mode.select('centered');assert.deepEqual(compose(mode.position([99,99,99]),I,[.1,.2,.3]),[.1,.2,.3]);assert.throws(()=>mode.select('world'));console.log('PASS: centered bypass, movement mode, persisted selection and fingertip composition');
const {Workspace}=require('./hand-translation.js');let bounds=null;const wsStore={getItem:()=>bounds,setItem:(k,v)=>bounds=v};let ws=new Workspace(wsStore);ws.set([10,10,10],'ft');assert(Math.abs(ws.metres[0]-3.048)<1e-12);ws=new Workspace(wsStore);assert.equal(ws.unit,'ft');assert(ws.contains([1,1,1]));ws.set([10,10,10],'cm');assert(!ws.contains([.1,0,0]));assert(ws.contains([.01,.01,.01]));const point=[.1,.2,.3];ws.scale(1000,600);assert.deepEqual(point,[.1,.2,.3]);assert.throws(()=>ws.set([-1,10,10],'cm'));assert.throws(()=>ws.set([NaN,10,10],'ft'));assert.throws(()=>ws.set([Infinity,10,10],'m'));ws.reset();assert.deepEqual(ws.metres,[1,1,1]);console.log('PASS: feet/cm conversion, persisted bounds, out-of-bounds detection, invalid dimensions and reset');
const {ViewZoom}=require('./hand-translation.js');let zoomSaved=null;const zoomStore={getItem:()=>zoomSaved,setItem:(k,v)=>zoomSaved=v},zoom=new ViewZoom(zoomStore);assert.equal(zoom.factor,1);const dimensions=ws.metres.slice();zoom.change(1.25);assert.equal(zoom.factor,1.25);assert.deepEqual(ws.metres,dimensions);assert.equal(new ViewZoom(zoomStore).factor,1.25);zoom.set(100);assert.equal(zoom.factor,8);zoom.set(.001);assert.equal(zoom.factor,.25);zoom.fit();assert.equal(zoom.factor,1);console.log('PASS: independent persistent zoom, limits and explicit fit');
