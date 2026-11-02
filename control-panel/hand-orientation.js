(function(root){
function multiply(a,b){return [a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3],a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2],a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1],a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0]];}
function matrix(q){const [w,x,y,z]=q;return [[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]];}
class Orientation{
 constructor(){this.q=[1,0,0,0];this.last=null;this.initialized=false;this.heading=0;}
 update(a,g,now,gravityCorrection=true){if(!a.concat(g).every(Number.isFinite)||!Number.isFinite(now))throw Error('Invalid IMU sample');const norm=Math.hypot(...a),valid=norm>.8&&norm<1.2;const dt=this.last===null?0:(now-this.last)/1000;this.last=now;
 if(!this.initialized){if(!valid)return {matrix:matrix(this.q),valid:false};const roll=Math.atan2(a[1],a[2]),pitch=Math.atan2(-a[0],Math.hypot(a[1],a[2]));this.q=multiply([Math.cos(pitch/2),0,Math.sin(pitch/2),0],[Math.cos(roll/2),Math.sin(roll/2),0,0]);this.initialized=true;}
 else if(dt>0&&dt<.35){const r=matrix(this.q),v=r[2],m=a.map(x=>x/norm),e=valid?[m[1]*v[2]-m[2]*v[1],m[2]*v[0]-m[0]*v[2],m[0]*v[1]-m[1]*v[0]]:[0,0,0],omega=g.map((x,i)=>x*Math.PI/180+(gravityCorrection?1.5:0)*e[i]),speed=Math.hypot(...omega),angle=speed*dt/2;const delta=speed>1e-10?[Math.cos(angle),...omega.map(x=>Math.sin(angle)*x/speed)]:[1,0,0,0];this.q=multiply(this.q,delta);const n=Math.hypot(...this.q);this.q=this.q.map(x=>x/n);}
 return {matrix:matrix(this.q),valid:this.initialized,gravityValid:valid,gap:dt>=.35};}
 zeroHeading(){const r=matrix(this.q),yaw=Math.atan2(r[1][0],r[0][0]);this.q=multiply([Math.cos(yaw/2),0,0,-Math.sin(yaw/2)],this.q);}
}
if(typeof module!=='undefined')module.exports={Orientation,matrix,multiply};else root.HandOrientation=Orientation;
})(typeof window!=='undefined'?window:this);

/* Display-only position remapping. Never changes the measured orientation. */
(function(root){
const defaults=()=>({axes:[0,1,2],signs:[1,1,1]});
function valid(s){return s&&Array.isArray(s.axes)&&Array.isArray(s.signs)&&s.axes.length===3&&s.signs.length===3&&new Set(s.axes).size===3&&s.axes.every(x=>[0,1,2].includes(x))&&s.signs.every(x=>x===1||x===-1);}
class DisplaySettings {
 constructor(storage){this.storage=storage;this.cache={};this.error='';}
 key(port,profile){return '6thsense.finger-display.v1:'+JSON.stringify([String(port),profile||'no-profile']);}
 get(port,profile){const k=this.key(port,profile);if(this.cache[k])return this.cache[k];let s;try{s=JSON.parse(this.storage.getItem(k));}catch{}return this.cache[k]=valid(s)?s:defaults();}
 save(port,profile,s){if(!valid(s))throw Error('Invalid axis mapping');const k=this.key(port,profile);this.cache[k]={axes:s.axes.slice(),signs:s.signs.slice()};try{this.storage.setItem(k,JSON.stringify(this.cache[k]));this.error='';}catch{this.error='Settings apply now but could not be saved in this browser.';}return this.cache[k];}
 axis(port,profile,out,input){const s=this.get(port,profile),axes=s.axes.slice(),other=axes.indexOf(input);[axes[out],axes[other]]=[axes[other],axes[out]];return this.save(port,profile,{axes,signs:s.signs});}
 reverse(port,profile,out){const s=this.get(port,profile),signs=s.signs.slice();signs[out]*=-1;return this.save(port,profile,{axes:s.axes,signs});}
 reset(port,profile){return this.save(port,profile,defaults());}
 position(port,profile,p){const s=this.get(port,profile);return s.axes.map((axis,i)=>s.signs[i]*p[axis]);}
}
if(typeof module!=='undefined')Object.assign(module.exports,{DisplaySettings});else root.HandDisplaySettings=DisplaySettings;
})(typeof window!=='undefined'?window:this);

(function(root){
const transpose=a=>a[0].map((_,i)=>a.map(row=>row[i]));
const mm=(a,b)=>a.map(row=>b[0].map((_,j)=>row.reduce((sum,v,k)=>sum+v*b[k][j],0)));
const eye=()=>[[1,0,0],[0,1,0],[0,0,1]];
function proper(r){if(!Array.isArray(r)||r.length!==3||!r.every(row=>Array.isArray(row)&&row.length===3&&row.every(Number.isFinite)))return false;const g=mm(r,transpose(r)),det=r[0][0]*(r[1][1]*r[2][2]-r[1][2]*r[2][1])-r[0][1]*(r[1][0]*r[2][2]-r[1][2]*r[2][0])+r[0][2]*(r[1][0]*r[2][1]-r[1][1]*r[2][0]);return Math.abs(det-1)<.005&&g.every((row,i)=>row.every((v,j)=>Math.abs(v-(i===j?1:0))<.005));}
function angle(a,b){const r=mm(a,transpose(b));return Math.acos(Math.max(-1,Math.min(1,(r[0][0]+r[1][1]+r[2][2]-1)/2)))*180/Math.PI;}
class FingerAlignment {
 constructor(storage){this.storage=storage;this.cache={};this.error='';}
 key(port,profile){return '6thsense.finger-orientation.v1:'+JSON.stringify([String(port),profile||'no-profile']);}
 get(port,profile){const key=this.key(port,profile);if(key in this.cache)return this.cache[key];let record;try{record=JSON.parse(this.storage.getItem(key));}catch{}return this.cache[key]=record&&proper(record.offset)?record:null;}
 save(port,profile,record){const key=this.key(port,profile);this.cache[key]=record;try{this.storage.setItem(key,JSON.stringify(record));this.error='';}catch{this.error='Alignment applies now but could not be saved.';}}
 align(port,profile,samples,confirmed){if(!confirmed)throw Error('Confirm the PCB reference pose first.');if(samples.length<8||samples.at(-1).time-samples[0].time<300)throw Error('Hold the finger still while fresh readings collect.');const ref=samples.at(-1).rotation;if(!samples.every(s=>proper(s.rotation)&&angle(s.rotation,ref)<5))throw Error('Orientation is moving or unstable; hold the finger still.');this.save(port,profile,{offset:transpose(ref),checks:{},reference:'component side up; flex tail +Y; PCB right edge +X',saved:new Date().toISOString()});}
 apply(port,profile,rotation){if(!proper(rotation))throw Error('Invalid measured rotation');const record=this.get(port,profile);return record?mm(rotation,record.offset):rotation;}
 check(port,profile,rotation,axis){const record=this.get(port,profile);if(!record)throw Error('Record a flat reference first.');const expected=axis==='tail'?[[1,0,0],[0,0,-1],[0,1,0]]:axis==='right'?[[0,0,-1],[0,1,0],[1,0,0]]:null;if(!expected)throw Error('Unknown tilt check');const error=angle(this.apply(port,profile,rotation),expected),result={error_degrees:error,passed:error<=15};record.checks={...record.checks,[axis]:result};this.save(port,profile,record);return result;}
 reset(port,profile){this.save(port,profile,null);}
}
class SourceFrame {
 constructor(storage){this.storage=storage;this.rotationMatrix=eye();try{const r=JSON.parse(storage.getItem('6thsense.source-frame.v1'));if(proper(r))this.rotationMatrix=r;}catch{}}
 key(){return JSON.stringify(this.rotationMatrix);}
 save(){try{this.storage.setItem('6thsense.source-frame.v1',this.key());}catch{}}
 turn(axis){const turns={X:[[1,0,0],[0,0,-1],[0,1,0]],Y:[[0,0,1],[0,1,0],[-1,0,0]],Z:[[0,-1,0],[1,0,0],[0,0,1]]};if(!turns[axis])throw Error('Unknown source axis');this.rotationMatrix=mm(turns[axis],this.rotationMatrix);this.save();}
 reset(){this.rotationMatrix=eye();this.save();}
 position(p){return this.rotationMatrix.map(row=>row.reduce((sum,v,i)=>sum+v*p[i],0));}
 rotation(r){if(!proper(r))throw Error('Invalid source orientation');return mm(this.rotationMatrix,r);}
}
if(typeof module!=='undefined')Object.assign(module.exports,{FingerAlignment,SourceFrame,rotationProduct:mm,properRotation:proper,rotationAngle:angle});else {root.HandFingerAlignment=FingerAlignment;root.HandSourceFrame=SourceFrame;}
})(typeof window!=='undefined'?window:this);

(function(root){
const distance=(a,b)=>Math.hypot(...a.map((v,i)=>v-b[i]));
class ApproachCheck {
 start(port,target,a,b){if(port===target)throw Error('Choose two different ports.');this.reference={port,target,a:a.slice(),b:b.slice()};}
 finish(port,target,a,b){const ref=this.reference;if(!ref||ref.port!==port||ref.target!==target)throw Error('Record a start position for these two ports first.');if(distance(ref.b,b)>.01)throw Error('Reference finger moved more than 1 cm. Keep it and the hub still, then repeat.');const normal=[distance(ref.a,ref.b),distance(a,b)],mirror=[distance(ref.a.map(v=>-v),ref.b),distance(a.map(v=>-v),b)];const decrease=normal[0]-normal[1],mirrorDecrease=mirror[0]-mirror[1];return {normal,mirror,decrease,mirrorDecrease,result:decrease>.01&&mirrorDecrease<=.01?'current branch supports approach':mirrorDecrease>.01&&decrease<=.01?'opposite mirror branch supports approach':'inconclusive: repeat a larger approach with unchanged orientation'};}
}
if(typeof module!=='undefined')Object.assign(module.exports,{ApproachCheck});else root.HandApproachCheck=ApproachCheck;
})(typeof window!=='undefined'?window:this);
