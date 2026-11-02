const assert=require('node:assert/strict'),{CameraView,Workspace,ViewZoom}=require('./hand-translation.js');
const saved=new Map(),store={getItem:k=>saved.get(k),setItem:(k,v)=>saved.set(k,v)},c=new CameraView(store),ws=new Workspace(store),z=new ViewZoom(store),point=[.1,.2,.3],copy=point.slice(),dimensions=ws.metres.slice();
const close=(a,b)=>a.forEach((v,i)=>assert(Math.abs(v-b[i])<1e-10));
c.preset('front');close(c.vector([1,0,0]),[1,0]);close(c.vector([0,0,1]),[0,1]);close(c.vector([0,1,0]),[0,0]);
c.preset('top');close(c.vector([0,1,0]),[0,1]);close(c.vector([0,0,1]),[0,0]);
c.preset('side');close(c.vector([0,1,0]),[-1,0]);close(c.vector([0,0,1]),[0,1]);
c.preset('front');c.rotateRoll(Math.PI/2);close(c.vector([1,0,0]),[0,1]);
const before=c.project(point,1000,500,100);c.move(100,50,1000,500);close(c.project(point,1000,500,100),[before[0]+100,before[1]+50]);
c.orbit(20,-10);c.save();const loaded=new CameraView(store);close(loaded.project(point,1000,500,100),c.project(point,1000,500,100));
assert.deepEqual(point,copy);assert.deepEqual(ws.metres,dimensions);assert.equal(z.factor,1);
c.orbit(0,100000);assert.equal(c.elevation,Math.PI/2);c.reset();assert.equal(c.roll,0);assert.deepEqual(c.pan,[0,0]);assert.equal(c.azimuth,.5);assert.equal(c.elevation,-.5);
console.log('PASS: front/top/side axes, actual camera roll, screen pan, orbit persistence, reset and sensor/workspace/zoom independence');

// From +Z, positive receiver Z rotation is counterclockwise on screen.
c.preset('top');const origin=c.project([0,0,0],1000,500,100),right=c.project([1,0,0],1000,500,100),positive=c.project([0,1,0],1000,500,100),clockwise=c.project([0,-1,0],1000,500,100);assert(right[0]>origin[0]);assert(positive[1]<origin[1]);assert(clockwise[1]>origin[1]);console.log('PASS: Top looks down from +Z; positive and negative 90-degree rotations have the correct physical sense');
// Presets discard a saved roll/orbit/pan and correctly identify manual orbit.
c.orbit(13,27);c.rotateRoll(.8);c.move(35,-20,1000,500);assert.equal(c.presetName(),'custom');c.preset('top');assert.equal(c.presetName(),'top');assert.equal(c.roll,0);assert.deepEqual(c.pan,[0,0]);
// Actual camera path used by unified.html: changing only depth leaves overhead pixel fixed.
close(c.project([.1,.2,-.4],330,240,250),c.project([.1,.2,.7],330,240,250));
const topXY=c.project([.1,.2,.3],330,240,250);c.preset('front');assert.notDeepEqual(c.project([.1,.2,.3],330,240,250),topXY);c.preset('side');assert.notDeepEqual(c.project([.1,.2,.3],330,240,250),topXY);c.orbit(1,1);assert.equal(c.presetName(),'custom');
const fs=require('node:fs'),html=fs.readFileSync(__dirname+'/unified.html','utf8');assert(html.includes('project=v=>camera.project(v,w,h,scale)'));assert(html.includes('camera.preset(name)'));assert(html.includes('/hand-translation.js'));console.log('PASS: unified render uses tested camera; Top eliminates Z depth, distinct presets, roll/pan reset and custom label');
