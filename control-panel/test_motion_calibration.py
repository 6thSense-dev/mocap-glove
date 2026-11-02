import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from pose_calibration import fields
from motion_calibration import fit_motion,motion_pose

def pack(x):return np.stack((x.real,x.imag),axis=-1).tolist()
class MotionTests(unittest.TestCase):
 def fixture(self):
  rng=np.random.default_rng(42);g=np.exp(rng.normal(0,.25,(3,3)));phase=np.array([.4,-.9,1.1])[:,None]+rng.normal(0,.01,(3,3));poses=[];values=[]
  for i in range(48):
   p=rng.normal(size=3);p[2]=abs(p[2])+.4;p=p/np.linalg.norm(p)*rng.uniform(.08,.16);r=Rotation.random(random_state=rng).as_matrix();poses.append(p);values.append(fields(p)@r*g*np.exp(1j*phase))
  return np.array(values),np.array(poses),g
 def test_free_movement_recovers_gain_ratios_and_distance(self):
  c,p,g=self.fixture();cal=fit_motion(pack(c),pack(c[:2]),np.linalg.norm(p[:2],axis=1))
  np.testing.assert_allclose(cal['gains'],g/np.exp(np.mean(np.log(g))),rtol=1e-4)
  for i in range(10):
   out=motion_pose(cal,pack(c[i]));self.assertAlmostEqual(np.linalg.norm(out['position']),np.linalg.norm(p[i]),places=6)
 def test_noisy_motion_still_cross_validates(self):
  c,p,_=self.fixture();rng=np.random.default_rng(99);c=c+rng.normal(size=c.shape)*.2+1j*rng.normal(size=c.shape)*.2
  cal=fit_motion(pack(c),pack(c[:2]),np.linalg.norm(p[:2],axis=1))
  self.assertLess(cal['held_out_rms'],.01)
 def test_inconsistent_distance_anchors_keep_relative_units(self):
  c,p,_=self.fixture();dist=np.linalg.norm(p[:2],axis=1);dist[1]*=2
  cal=fit_motion(pack(c),pack(c[:2]),dist)
  self.assertIsNone(cal['metres_per_unit'])
 def test_non_dipole_data_rejected(self):
  rng=np.random.default_rng(12);c=rng.normal(size=(48,3,3))*100
  with self.assertRaises(ValueError):fit_motion(pack(c.astype(complex)))
 def test_stationary_data_rejected(self):
  c,_,_=self.fixture()
  with self.assertRaises(ValueError):fit_motion(pack(np.repeat(c[:1],32,axis=0)))
 def test_too_few_samples_rejected(self):
  c,_,_=self.fixture()
  with self.assertRaises(ValueError):fit_motion(pack(c[:8]))
 def test_unanchored_does_not_claim_metres(self):
  c,_,_=self.fixture();cal=fit_motion(pack(c));self.assertIsNone(cal['metres_per_unit']);self.assertEqual(cal['units'],'relative')
if __name__=='__main__':unittest.main()
