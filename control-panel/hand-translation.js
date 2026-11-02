/* Experimental short-duration inertial displacement, in metres. No absolute reference. */
(function(root){
const mul=(r,v)=>r.map(row=>row.reduce((sum,x,i)=>sum+x*v[i],0));
const compose=(position,rotation,local)=>mul(rotation,local).map((x,i)=>x+position[i]);
class Translation {
 constructor(){this.reset();}
 reset(){this.position=[0,0,0];this.velocity=[0,0,0];this.bias=[0,0,0];this.gyroBias=[0,0,0];this.samples=[];this.last=null;this.sampleId=null;this.rest=0;this.ready=false;this.progress=0;this.acceleration=[0,0,0];this.noise=0;this.quietWindow=[];this.motionTime=0;this.held=false;this.restPosition=[0,0,0];this.status='Hold still briefly to zero movement';}
 update(a,g,r,now,id){
  if(!a.concat(g,r.flat(),[now]).every(Number.isFinite))throw Error('Invalid movement sample');
  if(id===undefined||id===null||id===this.sampleId){this.status='Waiting for fresh IMU sample';return this.result();}
  this.sampleId=id;
  const dt=this.last===null?0:(now-this.last)/1000;this.last=now;
  const world=mul(r,a).map((x,i)=>9.80665*(x-(i===2?1:0)));
  this.acceleration=world.slice();
  if(dt<0||dt>.1){this.velocity=[0,0,0];this.rest=0;if(!this.ready)this.samples=[];this.status='Timing gap · movement estimate paused';return this.result();}
  if(!this.ready){
   if(Math.hypot(...g)>3||Math.abs(Math.hypot(...a)-1)>.1){this.samples=[];this.progress=0;this.status='Movement not initialized: hold still for 1 second';return this.result();}
   this.samples.push({world,g,now});
   const means=[0,1,2].map(i=>this.samples.reduce((s,v)=>s+v.world[i],0)/this.samples.length);
   const scatter=Math.sqrt(this.samples.reduce((s,v)=>s+v.world.reduce((t,x,i)=>t+(x-means[i])**2,0),0)/this.samples.length);
   if(scatter>.15){this.samples=[];this.progress=0;this.status='Movement not initialized: hold still for 1 second';return this.result();}
   this.progress=Math.min(1,(now-this.samples[0].now)/1000);this.status='Zeroing movement: '+Math.round(this.progress*100)+'% · hold still';
   if(this.samples.length>=40&&now-this.samples[0].now>=1000){this.bias=[0,1,2].map(i=>r.reduce((sum,row,j)=>sum+row[i]*means[j],0));this.noise=scatter;this.gyroBias=[0,1,2].map(i=>this.samples.reduce((s,v)=>s+v.g[i],0)/this.samples.length);this.ready=true;this.status='Experimental movement estimate · at rest';}
   return this.result();
  }
  if(dt<=0||dt<.002){this.status='Waiting for valid sample timing';return this.result();}
  const biasWorld=mul(r,this.bias),acceleration=world.map((x,i)=>x-biasWorld[i]);this.acceleration=acceleration;
  // Detect sustained quiet from raw IMU stability, not only the gravity-subtracted
  // acceleration: a small tilt error otherwise prevents rest detection forever.
  this.quietWindow.push({a:a.slice(),g:g.slice(),now});
  this.quietWindow=this.quietWindow.filter(s=>now-s.now<=800);
  const mean=[0,1,2].map(i=>this.quietWindow.reduce((sum,s)=>sum+s.a[i],0)/this.quietWindow.length);
  const scatter=Math.sqrt(this.quietWindow.reduce((sum,s)=>sum+s.a.reduce((v,x,i)=>v+(x-mean[i])**2,0),0)/this.quietWindow.length);
  const quiet=this.quietWindow.length>=20&&now-this.quietWindow[0].now>=600&&scatter<.002&&Math.abs(Math.hypot(...a)-1)<.035&&this.quietWindow.every(s=>Math.hypot(...s.g.map((x,i)=>x-this.gyroBias[i]))<1.5);
  this.rest=quiet?.6:0;
  if(this.held){this.velocity=[0,0,0];this.status='Movement estimate held: uncertain · Recenter to resume';return this.result();}
  if(this.rest>=.6){this.velocity=[0,0,0];this.motionTime=0;this.restPosition=this.position.slice();
   const bodyResidual=[0,1,2].map(i=>r.reduce((sum,row,j)=>sum+row[i]*world[j],0));
   this.bias=this.bias.map((x,i)=>x+.02*(bodyResidual[i]-x));
   this.status='Experimental movement estimate · rest assumed';}
  else {
   // Gentle velocity decay limits runaway drift, but also underestimates real motion.
   this.motionTime+=dt;
   if(this.motionTime>1.5||Math.hypot(...this.position.map((x,i)=>x-this.restPosition[i]))>.35){this.held=true;this.velocity=[0,0,0];this.status='Movement estimate held: uncertain · Recenter to resume';return this.result();}
   const before=this.velocity.slice(),decay=Math.exp(-.6*dt);
   this.velocity=this.velocity.map((x,i)=>(x+acceleration[i]*dt)*decay);
   this.position=this.position.map((x,i)=>x+(before[i]+this.velocity[i])*dt/2);
   this.status='Experimental movement estimate · moving';
  }
  return this.result();
 }
 result(){return {position:this.position.slice(),velocity:this.velocity.slice(),ready:this.ready,status:this.status,acceleration:this.acceleration.slice(),progress:this.progress,rest_seconds:this.rest,held:this.held};}
}
class MovementMode {
 constructor(storage){this.storage=storage;let saved;try{saved=storage.getItem('6thsense.hub-mode.v1');}catch{}this.mode=saved==='moving'?'moving':'centered';}
 select(mode){if(!['moving','centered'].includes(mode))throw Error('Invalid movement mode');this.mode=mode;try{this.storage.setItem('6thsense.hub-mode.v1',mode);}catch{}}
 position(estimated){return this.mode==='moving'?estimated.slice():[0,0,0];}
}
class Workspace {
 constructor(storage){this.storage=storage;this.unit='cm';this.metres=[1,1,1];try{const saved=JSON.parse(storage.getItem('6thsense.workspace.v1'));if(saved)this.set(saved.values,saved.unit);}catch{}}
 set(values,unit){const factor={cm:.01,m:1,ft:.3048}[unit];if(!factor||values.length!==3||!values.every(v=>Number.isFinite(v)&&v*factor>=.01&&v*factor<=100))throw Error('Each dimension must be between 1 cm and 100 m.');this.unit=unit;this.metres=values.map(v=>v*factor);try{this.storage.setItem('6thsense.workspace.v1',JSON.stringify({values,unit}));}catch{}}
 values(unit=this.unit){const factor={cm:.01,m:1,ft:.3048}[unit];return this.metres.map(v=>v/factor);}
 contains(point){return point.every((x,i)=>Math.abs(x)<=this.metres[i]/2);}
 scale(width,height){const [x,y,z]=this.metres;return Math.min(width*.85/(.88*x+.45*y),height*.75/(.22*x+.42*y+.9*z));}
 reset(){this.set([100,100,100],'cm');}
}
class ViewZoom {
 constructor(storage){this.storage=storage;let value;try{value=Number(storage.getItem('6thsense.view-zoom.v1'));}catch{}this.factor=Number.isFinite(value)&&value>=.25&&value<=8?value:1;}
 set(value){if(!Number.isFinite(value))return;this.factor=Math.max(.25,Math.min(8,value));try{this.storage.setItem('6thsense.view-zoom.v1',String(this.factor));}catch{}}
 change(multiplier){if(multiplier>0)this.set(this.factor*multiplier);}
 fit(){this.set(1);}
}
class CameraView {
 constructor(storage){this.storage=storage;this.reset(false);try{const s=JSON.parse(storage.getItem('6thsense.camera.v1'));if(s&&[s.azimuth,s.elevation,s.roll,...s.pan].every(Number.isFinite)&&s.pan.length===2){this.azimuth=s.azimuth;this.elevation=Math.max(-Math.PI/2,Math.min(Math.PI/2,s.elevation));this.roll=s.roll;this.pan=s.pan.map(x=>Math.max(-3,Math.min(3,x)));}}catch{}}
 save(){try{this.storage.setItem('6thsense.camera.v1',JSON.stringify({azimuth:this.azimuth,elevation:this.elevation,roll:this.roll,pan:this.pan}));}catch{}}
 reset(save=true){this.azimuth=.5;this.elevation=-.5;this.roll=0;this.pan=[0,0];if(save)this.save();}
 presetName(){const near=(a,b)=>Math.abs(a-b)<1e-7;if(!near(this.roll,0))return 'custom';if(near(this.azimuth,0)&&near(this.elevation,-Math.PI/2))return 'top';if(near(this.azimuth,0)&&near(this.elevation,0))return 'front';if(near(this.azimuth,Math.PI/2)&&near(this.elevation,0))return 'side';return 'custom';}
 preset(name){const presets={front:[0,0],top:[0,-Math.PI/2],side:[Math.PI/2,0]};if(!presets[name])throw Error('Invalid camera preset');[this.azimuth,this.elevation]=presets[name];this.roll=0;this.pan=[0,0];this.save();}
 orbit(dx,dy){if(![dx,dy].every(Number.isFinite))return;this.azimuth+=dx*.006;this.elevation=Math.max(-Math.PI/2,Math.min(Math.PI/2,this.elevation+dy*.006));}
 move(dx,dy,width,height){if(width<=0||height<=0||![dx,dy,width,height].every(Number.isFinite))return;this.pan=[this.pan[0]+dx/width,this.pan[1]+dy/height].map(v=>Math.max(-3,Math.min(3,v)));}
 rotateRoll(angle){if(Number.isFinite(angle)){this.roll=Math.atan2(Math.sin(this.roll+angle),Math.cos(this.roll+angle));}}
 vector(p){const a=this.azimuth,e=this.elevation,r=this.roll,right=Math.cos(a)*p[0]-Math.sin(a)*p[1],up=-Math.sin(e)*(Math.sin(a)*p[0]+Math.cos(a)*p[1])+Math.cos(e)*p[2];return [Math.cos(r)*right-Math.sin(r)*up,Math.sin(r)*right+Math.cos(r)*up];}
 project(p,width,height,scale){const [x,y]=this.vector(p);return [width*(.5+this.pan[0])+x*scale,height*(.55+this.pan[1])-y*scale];}
}
if(typeof module!=='undefined')module.exports={Translation,compose,MovementMode,Workspace,ViewZoom,CameraView};else {root.HandTranslation=Translation;root.composeHandPoint=compose;root.HandMovementMode=MovementMode;root.HandWorkspace=Workspace;root.HandViewZoom=ViewZoom;root.HandCameraView=CameraView;}
})(typeof window!=='undefined'?window:this);
(function(root){function handViewTransform(mode,position,rotation){if(mode==='hand')return {position:[0,0,0],rotation:[[1,0,0],[0,1,0],[0,0,1]]};if(mode==='room')return {position:position.slice(),rotation:rotation.map(r=>r.slice())};throw Error('Unknown view frame');}if(typeof module!=='undefined')module.exports.handViewTransform=handViewTransform;else root.handViewTransform=handViewTransform;})(typeof window!=='undefined'?window:this);
