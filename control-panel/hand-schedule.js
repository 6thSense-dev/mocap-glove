(function(root){
function captureGroups(mask){
 const a=[0,1,2].filter(i=>mask&(1<<i)),b=[3,4].filter(i=>mask&(1<<i));
 return Array.from({length:Math.max(a.length,b.length)},(_,i)=>(i<a.length?1<<a[i]:0)|(i<b.length?1<<b[i]:0));
}
if(typeof module!=='undefined')module.exports={captureGroups};else root.handCaptureGroups=captureGroups;
})(typeof window!=='undefined'?window:this);
