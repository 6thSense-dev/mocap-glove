const assert=require('node:assert/strict'),{FingerAlignment,SourceFrame,rotationProduct:mm,properRotation:proper,rotationAngle:angle}=require('./hand-orientation.js');
const I=[[1,0,0],[0,1,0],[0,0,1]],X=[[1,0,0],[0,0,-1],[0,1,0]],Y=[[0,0,-1],[0,1,0],[1,0,0]],Z=[[0,-1,0],[1,0,0],[0,0,1]],data=new Map(),store={getItem:k=>data.get(k),setItem:(k,v)=>data.set(k,v)};
const samples=r=>Array.from({length:12},(_,i)=>({rotation:r,time:i*40}));let a=new FingerAlignment(store);
assert.throws(()=>a.align(1,'p',samples(Z),false));a.align(1,'p',samples(Z),true);assert(angle(a.apply(1,'p',Z),I)<1e-6);
assert(angle(a.apply(1,'p',mm(X,Z)),X)<1e-6);assert(a.check(1,'p',mm(X,Z),'tail').passed);assert(a.check(1,'p',mm(Y,Z),'right').passed);assert(!a.check(1,'p',mm(Z,X),'tail').passed);
a.align(2,'p',samples(X),true);assert(angle(a.apply(2,'p',X),I)<1e-6);assert(angle(a.apply(1,'p',Z),I)<1e-6);assert.equal(a.get(1,'different'),null);a=new FingerAlignment(store);assert(angle(a.apply(1,'p',Z),I)<1e-6);
assert.throws(()=>a.align(3,'p',samples([[-1,0,0],[0,1,0],[0,0,1]]),true));assert.throws(()=>a.align(3,'p',[...samples(I).slice(0,11),{rotation:X,time:440}],true));
const source=new SourceFrame(store);source.turn('Z');assert.deepEqual(source.position([1,2,3]),[-2,1,3]);assert.deepEqual(source.rotation(I),Z);assert(proper(source.rotation(X)));const sourceKey=source.key();a.align(4,sourceKey,samples(source.rotation(X)),true);assert(angle(a.apply(4,sourceKey,source.rotation(mm(Y,X))),mm(mm(Z,Y),Z.map((_,i)=>Z.map(row=>row[i]))))<1e-6);
for(let i=0;i<3;i++)source.turn('Z');assert.deepEqual(source.rotationMatrix,I);assert.equal(a.get(4,source.key()),null);
for(let i=0;i<20;i++){source.turn(['X','Y','Z'][i%3]);assert(proper(source.rotationMatrix));assert(proper(source.rotation(X)));}
a.reset(1,'p');assert.equal(a.get(1,'p'),null);console.log('PASS: reference consent/stability, flat and two-axis lifts, independent persisted mounting offsets, proper shared frame mapping and reflection rejection');
const {ApproachCheck}=require('./hand-orientation.js');let approach=new ApproachCheck();approach.start('1','2',[.05,0,0],[.1,0,0]);let result=approach.finish('1','2',[.08,0,0],[.1,0,0]);assert(result.normal[1]<result.normal[0]);assert(result.result.startsWith('current'));
approach.start('1','2',[-.05,0,0],[.1,0,0]);result=approach.finish('1','2',[-.08,0,0],[.1,0,0]);assert(result.result.startsWith('opposite'));assert.throws(()=>approach.finish('1','2',[-.08,0,0],[.12,0,0]));
// Common proper rotations and common reflection preserve pairwise distance;
// a per-port mirror can change approach direction and must not be auto-applied.
const reflect=p=>[p[0],p[1],-p[2]],dist=(p,q)=>Math.hypot(...p.map((v,i)=>v-q[i])),pa=[.03,.07,.02],pb=[.02,.1,.05];assert(Math.abs(dist(pa,pb)-dist(reflect(pa),reflect(pb)))<1e-12);source.reset();source.turn('Z');assert(Math.abs(dist(pa,pb)-dist(source.position(pa),source.position(pb)))<1e-12);
console.log('PASS: approach distance decreases, opposite branch diagnosed, moving reference rejected, common frame distance invariance');
