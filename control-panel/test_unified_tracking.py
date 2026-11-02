import unittest,tempfile,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from test_motion_calibration import MotionTests,pack
from unified_tracking import fit_receivers,load_profiles,coherent_pose,read_capture
from motion_calibration import fit_motion,motion_pose

class UnifiedTests(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name);(self.root/'captures').mkdir();self.device='test-hub'
 def tearDown(self):self.tmp.cleanup()
 def records(self,port,values,poses):
  files=[]
  for i,v in enumerate(values):
   name=f'capture-20261031-120000-{port*1000+i:06d}.csv';files.append(name)
   sp={'clipped':False,'amplitudes':[{'xyz_source_counts':pack(row)} for row in v]}
   (self.root/'captures'/(name+'.json')).write_text(json.dumps({'mask':1<<(port-1),'axes':7,'device_port':self.device,'firmware':'0.2.7-diagnostics','spectrum':{str(port):sp}}))
  return {'files':files,'anchors':[{'file':files[i],'position_m':poses[i].tolist()} for i in range(2)]}
 def test_independent_receivers_partial_failure_persistence(self):
  values,p,_=MotionTests().fixture();rng=np.random.default_rng(61)
  # Distinct physical analogue gains and phases, realistic small measurement noise.
  v2=values*np.exp(rng.normal(0,.12,(3,3)))*np.exp(1j*.12)
  v2+=rng.normal(0,.1,v2.shape)+1j*rng.normal(0,.1,v2.shape)
  r1=self.records(1,values,p);r2=self.records(2,v2,p);r2['anchors']=[]
  result=fit_receivers(self.root,{'1':r1,'2':r2,'5':{'files':r1['files'],'anchors':[]}},self.device)
  self.assertTrue(result['1']['accepted']);self.assertTrue(result['2']['accepted']);self.assertFalse(result['5']['accepted'])
  profiles,_=load_profiles(self.root,self.device);self.assertEqual(set(profiles),{'1','2'})
  self.assertIsNotNone(profiles['1']['cal']['metres_per_unit']);self.assertIsNone(profiles['2']['cal']['metres_per_unit'])
  old=profiles['1']['file'];bad={'files':r1['files'][:2]};self.assertFalse(fit_receivers(self.root,{'1':bad},self.device)['1']['accepted']);self.assertEqual(load_profiles(self.root,self.device)[0]['1']['file'],old)
 def test_coherent_position_and_orientation_share_proper_transform(self):
  values,p,_=MotionTests().fixture();cal=fit_motion(pack(values),pack(values[:2]),np.linalg.norm(p[:2],axis=1))
  w=Rotation.from_euler('xyz',[35,-70,125],degrees=True).as_matrix()
  for v in values[::5]:
   base=coherent_pose(cal,pack(v),np.eye(3));rot=coherent_pose(cal,pack(v),w)
   np.testing.assert_allclose(rot['source_position'],w@base['source_position'],atol=1e-8)
   np.testing.assert_allclose(rot['source_rotation'],w@base['source_rotation'],atol=1e-8)
   self.assertAlmostEqual(np.linalg.det(rot['source_rotation']),1)
   self.assertAlmostEqual(np.linalg.norm(rot['source_position']),np.linalg.norm(base['source_position']))
  with self.assertRaises(ValueError):coherent_pose(cal,pack(values[0]),np.diag([1,1,-1]))
 def test_unknown_scale_stays_relative(self):
  values,_,_=MotionTests().fixture();cal=fit_motion(pack(values));p=coherent_pose(cal,pack(values[0]),np.eye(3));self.assertFalse(p['metric']);self.assertEqual(p['units'],'relative')
 def test_port_and_device_and_path_validation(self):
  values,p,_=MotionTests().fixture();record=self.records(1,values,p)
  for port,device in [(2,self.device),(1,'wrong')]:
   with self.assertRaises(ValueError):read_capture(self.root,record['files'][0],port,device)
  with self.assertRaises(ValueError):read_capture(self.root,'../bad',1,self.device)
if __name__=='__main__':unittest.main()
