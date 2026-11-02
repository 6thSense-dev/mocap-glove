import unittest
from unittest.mock import patch
import numpy as np
from glove_tracking import register_to_references,display_pose
class RegistrationTests(unittest.TestCase):
 def test_known_rotation_and_pose(self):
  w=np.array([[0,-1,0],[1,0,0],[0,0,1.]])
  x=[[.1,0,0],[0,.15,0]]
  cal={'metres_per_unit':1,'anchors':[{'file':str(i),'position_m':(w@v).tolist()} for i,v in enumerate(x)]}
  with patch('glove_tracking.motion_pose',side_effect=lambda c,i:{'position':x[int(i)]}):
   r=register_to_references(cal,lambda name:name)
  np.testing.assert_allclose(r['rotation'],w,atol=1e-10)
  with patch('glove_tracking.motion_pose',return_value={'position':x[0],'rotation_relative_to_start':np.eye(3).tolist()}):
   p=display_pose(cal,r,None)
  np.testing.assert_allclose(p['hub_relative_position_m'],[0,.1,0],atol=1e-10)
 def test_no_anchors_never_invents_hub_frame(self):
  self.assertIsNone(register_to_references({},None))
  with patch('glove_tracking.motion_pose',side_effect=lambda *args,**kwargs:{'position':[1,2,3]}):
   p=display_pose({},None,None)
  self.assertIsNone(p['hub_relative_position_m'])
 def test_parallel_anchors_rejected(self):
  cal={'metres_per_unit':1,'anchors':[{'file':'a','position_m':[0,1,0]},{'file':'b','position_m':[0,2,0]}]}
  with patch('glove_tracking.motion_pose',return_value={'position':[1,0,0]}):
   self.assertIsNone(register_to_references(cal,lambda n:n))

class SignalTests(unittest.TestCase):
 def test_signal_is_not_a_pose(self):
  from glove_tracking import signal_summary
  s=signal_summary({'clipped':False,'amplitudes':[{'xyz_counts':[3,4,0]},{'xyz_counts':[0,0,12]},{'xyz_counts':[0,0,0]}]})
  self.assertEqual(s['tone_strength_counts'],[5,12,0]);self.assertEqual(s['total_counts'],13);self.assertFalse(s['position_valid'])

class MultiPortTests(unittest.TestCase):
 def test_uncalibrated_port_does_not_disappear(self):
  from server import Hub
  h=Hub();h.port='test';sp={'clipped':False,'amplitudes':[{'xyz_counts':[10,20,30],'xyz_source_counts':[[10,0],[20,0],[30,0]]} for _ in range(3)]}
  h.last_capture={'axes':7,'device_port':'test','file':'test','spectrum':{'1':sp,'4':sp}}
  h.glove_calibrations={'4':({'test':True},None)}
  with patch('glove_tracking.automatic_pose',side_effect=lambda *args,**kwargs:{'position':[1,2,3]}):s=h.action('glove_solve',{})
  poses=s['diagnostics']['glove_poses'];self.assertTrue(poses['1']['provisional']);self.assertFalse(poses['4']['provisional']);self.assertEqual(poses['1']['model_port'],'4')
  h.last_capture={**h.last_capture,'file':'next-frame'}
  with patch('glove_tracking.automatic_pose',side_effect=ValueError('bad fit')):s=h.action('glove_solve',{})
  self.assertIn('signal',s['diagnostics']['glove_poses']['1']);self.assertIn('error',s['diagnostics']['glove_poses']['1'])
 def test_fast_tracking_gates(self):
  from server import Hub
  import time
  h=Hub();h.status={'found':31,'firmware':'0.2.4-diagnostics','coil_drive':True}
  with self.assertRaises(ValueError):h.action('capture_track',{'mask':31,'axes':7,'bench_ready':True})
  h.status['firmware']='0.2.5-diagnostics'
  for data in [{'mask':31,'axes':4,'bench_ready':True},{'mask':31,'axes':7}]:
   with self.assertRaises(ValueError):h.action('capture_track',data)
  h.last_coil=time.monotonic()
  with self.assertRaises(ValueError):h.action('capture_track',{'mask':31,'axes':7,'bench_ready':True})

class AutomaticTimingTests(unittest.TestCase):
 def test_random_poses_and_bus_delays_preserve_signed_matrix(self):
  from glove_tracking import align_acquisition_delay
  from scipy.spatial.transform import Rotation
  rng=np.random.default_rng(144)
  phase=np.array([[.8]*3,[.2]*3,[-.3]*3]);f=np.array([4500,5625,6750])[:,None]
  for _ in range(60):
   direction=rng.normal(size=3);direction/=np.linalg.norm(direction)
   m=(3*np.outer(direction,direction)-np.eye(3))@Rotation.random(random_state=rng).as_matrix()*100
   delay=rng.uniform(-17,17);z=m*np.exp(1j*(phase+2*np.pi*f*delay*1e-6))
   corrected,timing=align_acquisition_delay({'phase':phase},np.stack([z.real,z.imag],axis=-1))
   a=np.array(corrected);recovered=(a[:,:,0]+1j*a[:,:,1])*np.exp(-1j*phase)
   np.testing.assert_allclose(recovered.real,m,atol=1e-6)
   self.assertAlmostEqual(timing['delay_us'],delay,places=4)
 def test_unrelated_channel_phases_rejected(self):
  from glove_tracking import align_acquisition_delay
  z=100*np.exp(1j*np.array([[0,1,-1],[.3,-.8,1.3],[-1.1,.7,0]]))
  with self.assertRaises(ValueError):align_acquisition_delay({'phase':np.zeros((3,3))},np.stack([z.real,z.imag],axis=-1))
 def test_missing_or_weak_axes_rejected(self):
  from glove_tracking import align_acquisition_delay
  with self.assertRaises(ValueError):align_acquisition_delay({'phase':np.zeros((3,3))},np.zeros((3,3,2)))
