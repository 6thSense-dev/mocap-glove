(function(root){
const I=()=>[[1,0,0],[0,1,0],[0,0,1]],add=(a,b)=>a.map((v,i)=>v+b[i]),sub=(a,b)=>a.map((v,i)=>v-b[i]),mul=(r,v)=>r.map(row=>row.reduce((s,x,i)=>s+x*v[i],0)),mm=(a,b)=>a.map(row=>b[0].map((_,j)=>row.reduce((s,v,k)=>s+v*b[k][j],0))),transpose=r=>r[0].map((_,j)=>r.map(row=>row[j])),norm=v=>Math.hypot(...v),scale=(v,s)=>v.map(x=>x*s),unit=v=>scale(v,1/Math.max(norm(v),1e-10));
function axis(a,t){a=unit(a);const [x,y,z]=a,c=Math.cos(t),s=Math.sin(t),d=1-c;return [[c+x*x*d,x*y*d-z*s,x*z*d+y*s],[y*x*d+z*s,c+y*y*d,y*z*d-x*s],[z*x*d-y*s,z*y*d+x*s,c+z*z*d]];}
function rpy(v){return mm(mm(axis([0,0,1],v[2]),axis([0,1,0],v[1])),axis([1,0,0],v[0]));}
const names={1:'thumb',2:'index',3:'middle',4:'ring',5:'pinky'},baseR=[[-1,0,0],[0,0,1],[0,1,0]],baseP=[0,-.055,0];
class Kinematics{
 constructor(model){this.model=model;this.angles={};this.errors={};this.joints=model.joints.map(j=>({...j,R:rpy(j.origin.rpy)}));this.groups=Object.fromEntries(Object.entries(names).map(([p,n])=>[p,this.joints.filter(j=>j.name.includes('_'+n+'_')&&j.type==='revolute')]));this.rest=this.forward();}
 forward(angles=this.angles){const links={right_base_link:{r:baseR,p:baseP}};for(const j of this.joints){const parent=links[j.parent];if(!parent)throw Error('Invalid URDF tree');const turn=j.type==='revolute'?axis(j.axis,angles[j.name]||0):I();links[j.child]={r:mm(mm(parent.r,j.R),turn),p:add(parent.p,mul(parent.r,j.origin.xyz))};}return links;}
 tip(port,links=this.forward()){const v=links['right_'+names[port]+'_tip_link'];return {position:v.p,direction:mul(v.r,[0,0,1])};}
 fit(port,target,direction){if(!target.concat(direction).every(Number.isFinite))throw Error('Invalid IK target');const joints=this.groups[port];if(!joints)throw Error('Unknown port');const loss=()=>{const tip=this.tip(port),e=sub(tip.position,target),d=sub(tip.direction,unit(direction));return e.reduce((s,v)=>s+v*v,0)+.00035*d.reduce((s,v)=>s+v*v,0);};
 for(const step of [.16,.08,.035])for(const j of joints){const old=this.angles[j.name]||0;let best=old,value=loss();for(const trial of [old-step,old+step]){this.angles[j.name]=Math.max(j.lower,Math.min(j.upper,trial));const next=loss();if(next<value){value=next;best=this.angles[j.name];}}this.angles[j.name]=best;}
 const tip=this.tip(port);return this.errors[port]={position_error_m:norm(sub(tip.position,target)),inferred:true};}
 resetPort(port){for(const j of this.groups[port])this.angles[j.name]=0;delete this.errors[port];}
 target(port,measured,reference,rotation){const rest=this.tip(port,this.rest),referenceR=port===1?axis([0,0,1],Math.PI/2):I(),relative=mm(rotation,transpose(referenceR));return {position:add(rest.position,scale(sub(measured,reference),.8)),direction:mul(relative,rest.direction)};}
}
const api={Kinematics,axis,rpy,mul,mm,transpose,baseR,baseP,names};if(typeof module!=='undefined')module.exports=api;else root.AeroKinematics=api;
})(typeof window!=='undefined'?window:this);
