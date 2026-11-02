import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from axis_registration import solve_axis_moves,update_reference
class AxisRegistrationTests(unittest.TestCase):
 def data(self,q):
  start=np.array([.08,.14,.03]);groups={}
  for name,delta in [('start',[0,0,0]),('x',[.04,0,0]),('y',[0,.05,0]),('z',[0,0,.035])]:
   groups[name]=[{'position':(q.T@(start+delta)).tolist(),'rotation_receiver_to_transmitter':np.eye(3).tolist()} for _ in range(4)]
  return groups
 def test_all_axis_permutations_signs_and_hub_rotations(self):
  for reflected in (False,True):
   for angles in ([0,0,90],[30,70,-140],[-80,45,12]):
    q=Rotation.from_euler('xyz',angles,degrees=True).as_matrix()
    if reflected:q=q@np.diag([-1,1,1])
    d=self.data(q);fit=solve_axis_moves(d);w=np.asarray(fit['rotation']);self.assertAlmostEqual(np.linalg.det(w),1)
    for i,name in enumerate(('x','y','z')):
     delta=np.array(d[name][0]['position'])-d['start'][0]['position'];out=w@(delta*fit['mirror']);expected=np.eye(3)[i]*np.linalg.norm(delta);np.testing.assert_allclose(out,expected,atol=1e-10)
 def test_reject_rotating_sensor_and_non_independent_moves(self):
  d=self.data(np.eye(3));d['x'][0]['rotation_receiver_to_transmitter']=Rotation.from_euler('z',90,degrees=True).as_matrix().tolist()
  with self.assertRaises(ValueError):solve_axis_moves(d)
  d=self.data(np.eye(3));d['y']=d['x']
  with self.assertRaises(ValueError):solve_axis_moves(d)
 def test_reference_original_unchanged_and_proper_mount(self):
  old={'source_rotation':np.eye(3).tolist(),'mount_offsets':{'2':np.eye(3).tolist()},'branch_seeds':{'2':[.1,.2,.3]}};fit=solve_axis_moves(self.data(np.diag([-1,1,1])));new=update_reference(old,fit);self.assertEqual(old['branch_seeds']['2'],[.1,.2,.3]);self.assertAlmostEqual(np.linalg.det(new['mount_offsets']['2']),1);self.assertEqual(new['branch_seeds']['2'],[-.1,-.2,-.3])
if __name__=='__main__':unittest.main()
