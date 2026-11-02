// Two-point empirical interpolation; no accuracy or field-model claim.
(function(root){
function position(signal,a,b,mode='linear'){if(![signal,a,b].every(Number.isFinite)||Math.abs(b-a)<3)return null;if(mode==='dipole'){if(signal<=0||a<=0||b<=0)return null;const x=1/Math.cbrt(signal),lo=1/Math.cbrt(a),hi=1/Math.cbrt(b);return 100*(x-lo)/(hi-lo);}return 100*(signal-a)/(b-a);}
function quality(signal,noise,clipped){return !clipped&&Number.isFinite(signal)&&signal>=Math.max(3,5*noise);}
function distance(signal,a,b,da,db,mode='linear'){const p=position(signal,a,b,mode);if(p===null||![da,db].every(Number.isFinite)||da===db||p<0||p>100)return null;return da+(db-da)*p/100;}
const api={position,quality,distance};if(typeof module!=="undefined")module.exports=api;else root.AxisModel=api;
})(typeof window!=="undefined"?window:this);
