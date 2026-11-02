import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from hand_registration import make_registration,apply_registration
class HandRegistrationTests(unittest.TestCase):
 def fixture(self):
  q=Rotation.from_euler('xyz',[32,-70,120],degrees=True).as_matrix();records=[];profiles={str(p):'fit'+str(p) for p in [1,2,5]}
  positions={1:[-.18,0,0],2:[-.02,.16,0],5:[.02,.16,0]};mount=Rotation.from_euler('xyz',[21,15,71],degrees=True).as_matrix()
  for port,p in positions.items():
   target=Rotation.from_euler('z',90 if port==1 else 0,degrees=True).as_matrix()
   for _ in range(8):records.append({'port':port,'pose':{'position':(q.T@p).tolist(),'rotation_receiver_to_transmitter':(q.T@target@mount.T).tolist()}})
  return {'records':records,'profiles':profiles},q,mount
 def test_known_right_hand_pose_and_motion(self):
  record,q,mount=self.fixture();reg=make_registration(record);np.testing.assert_allclose(reg['source_rotation'],q,atol=1e-7)
  for port in [1,2,5]:
   for p in [[-.10,.12,.04],[.01,.15,-.03]]:
    r=Rotation.from_euler('xyz',[25,-15,60],degrees=True).as_matrix();pose={'position':q.T@p,'rotation_receiver_to_transmitter':q.T@r@mount.T,'metric':True}
    out=apply_registration(reg,str(port),'fit'+str(port),pose)
    np.testing.assert_allclose(out['hub_relative_position_m'],p,atol=1e-7);np.testing.assert_allclose(out['display_rotation'],r,atol=1e-7)
    self.assertAlmostEqual(np.linalg.det(out['display_rotation']),1)
 def test_scale_is_explicit_and_profile_change_rejected(self):
  record,q,_=self.fixture();reg=make_registration(record);pose={'position':[1,2,3],'rotation_receiver_to_transmitter':np.eye(3),'metric':False}
  with self.assertRaises(ValueError):apply_registration(reg,'1','changed',pose,.5)
  with self.assertRaises(ValueError):apply_registration(reg,'2','fit2',pose)
  out=apply_registration(reg,'2','fit2',pose,.5);self.assertIn('estimated',out['distance_status']);self.assertAlmostEqual(np.linalg.norm(out['hub_relative_position_m']),np.sqrt(14)*.5)
 def test_bad_or_incomplete_reference_rejected(self):
  record,_,_=self.fixture();record['records']=record['records'][:8]
  with self.assertRaises(ValueError):make_registration(record)
if __name__=='__main__':unittest.main()
