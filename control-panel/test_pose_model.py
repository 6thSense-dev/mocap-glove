import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from pose_model import solve_pose

class PoseTests(unittest.TestCase):
 def test_random_signed_poses(self):
  rng=np.random.default_rng(132)
  for _ in range(200):
   p=rng.normal(size=3);p[2]=abs(p[2])+.3;p*=rng.uniform(.05,.2)/np.linalg.norm(p)
   u=p/np.linalg.norm(p);r=Rotation.random(random_state=rng).as_matrix()
   m=(3*np.outer(u,u)-np.eye(3))@r/np.linalg.norm(p)**3
   result=solve_pose(m,phase_referenced=True)
   np.testing.assert_allclose(result['position_m'],p,atol=1e-12)
   np.testing.assert_allclose(result['rotation_receiver_to_transmitter'],r,atol=1e-12)
 def test_no_unsigned_live_input(self):
  with self.assertRaises(ValueError):solve_pose(np.eye(3))
 def test_reflected_winding_rejected(self):
  with self.assertRaises(ValueError):solve_pose(np.diag([-1,-1,-2]),phase_referenced=True)
 def test_bad_shape_or_missing_axis(self):
  for m in [np.zeros((3,3)),np.eye(2),np.diag([1,0,2]),np.full((3,3),float('nan'))]:
   with self.assertRaises(ValueError):solve_pose(m,phase_referenced=True)
 def test_distortion_rejected(self):
  with self.assertRaises(ValueError):solve_pose(np.diag([-1,-1,5]),phase_referenced=True)
 def test_hemisphere_ambiguity_explicit(self):
  a=solve_pose(np.diag([-1,-1,2]),phase_referenced=True)
  b=solve_pose(np.diag([-1,-1,2]),phase_referenced=True,hemisphere=(0,0,-1))
  np.testing.assert_allclose(a['position_m'],-np.array(b['position_m']))
if __name__=='__main__':unittest.main()

class RotationConventionTests(unittest.TestCase):
 def test_known_positive_tilts_and_shared_frame_rotation(self):
  p=np.array([.04,.12,.08]);u=p/np.linalg.norm(p)
  for axis in 'xyz':
   r=Rotation.from_euler(axis,90,degrees=True).as_matrix()
   m=(3*np.outer(u,u)-np.eye(3))@r/np.linalg.norm(p)**3
   solved=solve_pose(m,phase_referenced=True)
   np.testing.assert_allclose(solved['rotation_receiver_to_transmitter'],r,atol=1e-12)
   q=Rotation.from_euler('z',180,degrees=True).as_matrix()
   transformed=solve_pose(q@m,phase_referenced=True,hemisphere=q@p)
   np.testing.assert_allclose(transformed['position_m'],q@p,atol=1e-12)
   np.testing.assert_allclose(transformed['rotation_receiver_to_transmitter'],q@r,atol=1e-12)
