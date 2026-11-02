(function(root){
class ZReflection{
 constructor(storage){this.storage=storage;this.key='6thsense.finger-z-reflection.v1';this.settings={};try{const d=JSON.parse(storage.getItem(this.key));for(let i=1;i<=5;i++)this.settings[i]=d?.[i]===true;}catch{}}
 enabled(port){return this.settings[port]===true;}
 set(port,enabled){if(![1,2,3,4,5].includes(Number(port)))throw Error('Unknown receiver');this.settings[port]=!!enabled;this.storage.setItem(this.key,JSON.stringify(this.settings));}
 setConnected(mask,enabled){for(let i=1;i<=5;i++)if(mask&(1<<(i-1)))this.set(i,enabled);}
 position(port,p){return [p[0],p[1],this.enabled(port)?-p[2]:p[2]];}
 rotation(port,r){if(!this.enabled(port))return r.map(row=>row.slice());const s=[1,1,-1];return r.map((row,i)=>row.map((v,j)=>s[i]*v*s[j]));}
 pose(port,p){return {...p,hub_relative_position_m:this.position(port,p.hub_relative_position_m),display_rotation:this.rotation(port,p.display_rotation),z_reflected:this.enabled(port)};}
}
if(typeof module!=='undefined')module.exports={ZReflection};else root.HandZReflection=ZReflection;
})(typeof window!=='undefined'?window:this);
