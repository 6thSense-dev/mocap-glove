const assert=require('node:assert/strict'),{DisplaySettings}=require('./hand-orientation.js');
const records=new Map(),store={getItem:k=>records.get(k)||null,setItem:(k,v)=>records.set(k,v)};
let d=new DisplaySettings(store);assert.deepEqual(d.position(1,'saved',[1,2,3]),[1,2,3]);
d.reverse(1,'saved',0);assert.deepEqual(d.position(1,'saved',[1,2,3]),[-1,2,3]);assert.deepEqual(d.position(2,'saved',[1,2,3]),[1,2,3]);assert.deepEqual(d.position(1,'different',[1,2,3]),[1,2,3]);
d.axis(1,'saved',0,2);assert.deepEqual(d.position(1,'saved',[1,2,3]),[-3,2,1]);assert.deepEqual(d.get(1,'saved').axes,[2,1,0]);
d=new DisplaySettings(store);assert.deepEqual(d.position(1,'saved',[1,2,3]),[-3,2,1]);d.reset(1,'saved');assert.deepEqual(d.position(1,'saved',[1,2,3]),[1,2,3]);
for(let port=1;port<=5;port++){d.reverse(port,'saved',port%3);assert(Math.abs(Math.hypot(...d.position(port,'saved',[1,2,3]))-Math.sqrt(14))<1e-12);}
records.set(d.key(3,'corrupt'),'{bad');assert.deepEqual(d.get(3,'corrupt').axes,[0,1,2]);assert.throws(()=>d.save(3,'saved',{axes:[0,0,1],signs:[1,1,1]}));
const blocked=new DisplaySettings({getItem:()=>{throw Error('blocked');},setItem:()=>{throw Error('blocked');}});blocked.reverse(1,'saved',2);assert.deepEqual(blocked.position(1,'saved',[1,2,3]),[1,2,-3]);assert(blocked.error);
// Position mapping returns a new vector, preserving raw source and orientation.
const pose={position:[1,2,3],rotation:[[1,0,0],[0,1,0],[0,0,1]]},before=JSON.stringify(pose);d.position(1,'saved',pose.position);assert.equal(JSON.stringify(pose),before);
console.log('PASS: reversal, axis swap, independent ports/profiles, reload persistence, reset, invalid data, blocked storage and untouched orientation');
