(function(root){
const add=(a,b)=>a.map((v,i)=>v+b[i]),sub=(a,b)=>a.map((v,i)=>v-b[i]),scale=(v,k)=>v.map(x=>x*k),norm=v=>Math.hypot(...v),unit=v=>scale(v,1/Math.max(norm(v),1e-9)),dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0);
const bases={1:[-.032,-.005,0],2:[-.026,.045,0],3:[0,.052,0],4:[.023,.045,0],5:[.042,.030,0]};
// Illustration only: joints inferred from measured fingertip and its orientation.
// Fixed link lengths per session; never move a measured endpoint to fit the rig.
class RobotHand{
 constructor(){this.lengths={};}
 solve(port,tip,rotation){const base=bases[port],heading=unit(rotation.map(row=>row[1]));if(!base||!tip.every(Number.isFinite))throw Error('Invalid fingertip');if(!this.lengths[port]){const total=Math.max(.065,norm(sub(tip,base))*1.12);this.lengths[port]=[.48,.30,.22].map(v=>v*total);}const [a,b,c]=this.lengths[port],desiredWrist=sub(tip,scale(heading,c)),v=sub(desiredWrist,base),axis=unit(v),d=Math.max(Math.abs(a-b)+1e-6,Math.min(a+b-1e-6,norm(v))),wrist=add(base,scale(axis,d)),along=(a*a-b*b+d*d)/(2*d),height=Math.sqrt(Math.max(0,a*a-along*along));let pole=sub([0,0,-1],scale(axis,dot([0,0,-1],axis)));if(norm(pole)<.01)pole=sub([1,0,0],scale(axis,axis[0]));const middle=add(add(base,scale(axis,along)),scale(unit(pole),height)),inferredEnd=add(wrist,scale(heading,c));return {joints:[base,middle,wrist,inferredEnd],measuredTip:tip.slice(),residual:norm(sub(inferredEnd,tip)),inferred:true};}
 scene(enabled,poses,detected){if(!enabled)return [];return [1,2,3,4,5].map(port=>{const p=poses[port];if(!(detected&(1<<(port-1)))||!p||p.error||!p.hub_relative_position_m||!p.display_rotation)return {port,tracked:false,joints:[bases[port],add(bases[port],port===1?[-.07,0,0]:[0,.09,0])],inferred:true};return {port,tracked:true,...this.solve(port,p.hub_relative_position_m,p.display_rotation)};});}
}
if(typeof module!=='undefined')module.exports={RobotHand,bases};else root.RobotHand=RobotHand;
})(typeof window!=='undefined'?window:this);
