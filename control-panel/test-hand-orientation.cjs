const assert=require('node:assert/strict');const {Orientation}=require('./hand-orientation.js');
let o=new Orientation();o.update([0,0,1],[0,0,0],0);let r;for(let t=100;t<=1000;t+=100)r=o.update([0,0,1],[0,0,90],t);assert(Math.abs(r.matrix[1][0]-1)<1e-6);o.zeroHeading();assert(Math.abs(o.update([0,0,1],[0,0,0],1100).matrix[0][0]-1)<1e-6);
o=new Orientation();r=o.update([0,1,0],[0,0,0],0);assert(Math.abs(r.matrix[2][1]-1)<1e-6);const before=JSON.stringify(r.matrix);r=o.update([0,1,0],[500,0,0],1000);assert(r.gap);assert.equal(JSON.stringify(r.matrix),before);
console.log('PASS: gyro yaw, heading reset, gravity tilt and stale-gap handling');
