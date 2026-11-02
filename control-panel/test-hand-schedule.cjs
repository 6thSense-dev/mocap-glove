const assert=require('node:assert/strict');const {captureGroups}=require('./hand-schedule.js');
for(let mask=0;mask<32;mask++){
 const groups=captureGroups(mask);assert.equal(groups.reduce((v,x)=>v|x,0),mask);
 let used=0;for(const g of groups){assert.equal(used&g,0);used|=g;assert([0,1,2,4].includes(g&7));assert([0,8,16].includes(g&24));}
}
assert.deepEqual(captureGroups(19),[17,2]);assert.deepEqual(captureGroups(31),[9,18,4]);
console.log('PASS: every detection mask covers each finger once with at most one receiver per bus per burst');
