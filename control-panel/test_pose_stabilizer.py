import unittest,numpy as np
from pose_stabilizer import PoseStabilizer
from pose_model import solve_pose
class StabilizerTests(unittest.TestCase):
 def pose(self,x):return {'position':list(x),'hub_relative_position_m':list(x),'display_rotation':np.eye(3).tolist()}
 def test_stationary_noise_reduction(self):
  rng=np.random.default_rng(8);s=PoseStabilizer();raw=[];filtered=[]
  for i in range(600):
   x=np.array([.04,.15,-.01])+rng.normal(0,.002,3);p=s.apply(self.pose(x),i/60)
   if i>120:raw.append(x);filtered.append(p['hub_relative_position_m'])
  self.assertLess(np.std(filtered,axis=0).mean(),np.std(raw,axis=0).mean()*.5)
 def test_step_response_and_gap_reset(self):
  s=PoseStabilizer()
  for i in range(60):s.apply(self.pose([0,.15,0]),i/60)
  for i in range(60,68):p=s.apply(self.pose([.1,.15,0]),i/60)
  self.assertGreater(p['hub_relative_position_m'][0],.09)
  np.testing.assert_allclose(s.previous(3),[.1,.15,0])
  p=s.apply(self.pose([.2,.15,0]),3);self.assertTrue(p['smoothing']['reset']);self.assertEqual(p['hub_relative_position_m'],[.2,.15,0])
 def test_rotation_remains_proper(self):
  from scipy.spatial.transform import Rotation
  s=PoseStabilizer()
  for i in range(120):
   p=self.pose([0,.15,0]);p['display_rotation']=Rotation.from_euler('z',170+i/3,degrees=True).as_matrix().tolist();p=s.apply(p,i/60)
   r=np.array(p['display_rotation']);np.testing.assert_allclose(r.T@r,np.eye(3),atol=1e-12);self.assertAlmostEqual(np.linalg.det(r),1)
 def test_crosses_reference_plane_without_jump(self):
  prior=None;positions=[]
  for theta in np.linspace(.01,np.pi-.01,180):
   u=np.array([np.sin(theta),0,np.cos(theta)]);m=(3*np.outer(u,u)-np.eye(3))/.15**3
   p=solve_pose(m,phase_referenced=True,previous_position=prior);prior=p['position_m'];positions.append(prior)
   np.testing.assert_allclose(prior,.15*u,atol=1e-10)
  self.assertLess(np.max(np.linalg.norm(np.diff(positions,axis=0),axis=1)),.003)
 def test_start_on_plane_and_flip(self):
  p=solve_pose(np.diag([2,-1,-1]),phase_referenced=True);self.assertIn('unresolved',p['branch_status'])
  s=PoseStabilizer();s.apply(self.pose([.1,.2,0]),1);s.flip();np.testing.assert_allclose(s.previous(1.01),[-.1,-.2,0])

 def test_single_rotation_spike_rejected(self):
  from scipy.spatial.transform import Rotation
  s=PoseStabilizer()
  for i in range(20):
   p=self.pose([0,.15,0])
   if i==10:p['display_rotation']=Rotation.from_euler('z',120,degrees=True).as_matrix().tolist()
   out=s.apply(p,i/60)
   self.assertLess(Rotation.from_matrix(out['display_rotation']).magnitude(),1e-9)
 def test_gap_preserves_selected_mirror_but_flags_uncertainty(self):
  s=PoseStabilizer();s.apply(self.pose([0,0,-.15]),1)
  prior=s.previous(3)
  pose=solve_pose(np.diag([-1,-1,2])/.15**3,phase_referenced=True,previous_position=prior)
  self.assertLess(pose['position_m'][2],0)
  out=s.apply(self.pose(pose['position_m']),3)
  self.assertTrue(out['branch_reacquired_after_gap'])
  self.assertTrue(out['smoothing']['reset'])

 def test_gap_warning_persists_on_next_frame(self):
  s=PoseStabilizer();s.apply(self.pose([0,0,-.15]),1);s.apply(self.pose([0,0,-.15]),3)
  self.assertTrue(s.apply(self.pose([0,0,-.15]),3.02)['branch_reacquired_after_gap'])
