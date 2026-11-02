import unittest
from unittest.mock import patch
import server
class FakeSerial:
 def __init__(self,*a,**k):self.writes=[];self.power=0;self.found=0;self.closed=False
 def write(self,data):
  self.writes.append(data)
  if data==b'sensors on\n':self.power=1
  if data==b'sensors off\n':self.power=0;self.found=0
  if data==b'probe\n':self.found=1
 def flush(self):pass
 def reset_input_buffer(self):pass
 def close(self):self.closed=True
 def readline(self):
  c=self.writes[-1]
  if c==b'status\n':return f'OK firmware=0.1.0-bench core=16000000 sensors={self.power} found={self.found} coil_drive_build=0\n'.encode()
  if c==b'probe\n':return b'OK found=1\n'
  if c==b'read 1\n':return b'ADC 1 status=0000 ch=-4,5,6,0\n'
  return b'OK\n'
class PanelTests(unittest.TestCase):
 def setUp(self):
  self.hub=server.Hub();self.hub.ports=lambda:[{'port':'test-port'}]
 def connect(self):
  with patch.object(server.serial,'Serial',FakeSerial),patch.object(server.time,'sleep'):
   return self.hub.action('connect',{'port':'test-port'})
 def test_connect_does_not_power_sensors(self):
  self.assertTrue(self.connect()['connected']);self.assertFalse(self.hub.status['sensors']);self.assertEqual(self.hub.link.writes,[b'status\n'])
 def test_wrong_port_rejected(self):
  with self.assertRaises(ValueError):self.hub.action('connect',{'port':'arbitrary'})
 def test_probe_needs_power(self):
  self.connect()
  with self.assertRaises(ValueError):self.hub.action('probe',{})
 def test_detect_read_and_poweroff(self):
  self.connect();self.hub.action('power_on',{});self.hub.action('probe',{});s=self.hub.action('read',{'sensor':1});self.assertEqual(s['readings']['1']['channels'],[-4,5,6,0]);self.hub.action('power_off',{});self.assertFalse(self.hub.status['sensors']);self.assertEqual(self.hub.readings,{})
 def test_capture_requires_detected_sensor(self):
  self.connect()
  with self.assertRaises(ValueError):self.hub.action('capture',{'mask':31})
 def test_coils_not_exposed(self):
  self.connect()
  with self.assertRaises(ValueError):self.hub.action('capture_tx',{'mask':1})
 def test_disconnect_requests_poweroff(self):
  self.connect();link=self.hub.link;self.hub.action('disconnect',{});self.assertTrue(link.closed);self.assertIn(b'sensors off\n',link.writes)
if __name__=='__main__':unittest.main(verbosity=2)

class TrackingStateTests(unittest.TestCase):
 def test_close_discards_old_capture(self):
  h=server.Hub();h.last_capture={'file':'old'};h.close();self.assertIsNone(h.last_capture)
 def test_resolve_same_capture_does_not_advance_filter(self):
  h=server.Hub();h.port='test';h.last_capture={'axes':7,'device_port':'test','file':'same','spectrum':{}}
  h.diagnostics={'glove_solved_capture':'same','glove_poses':{'1':{'capture':'same'}}}
  with patch('glove_tracking.automatic_pose',side_effect=AssertionError('must not solve again')):
   result=h.action('glove_solve',{})
  self.assertEqual(result['diagnostics']['glove_poses']['1']['capture'],'same')

class SavedProfileTests(unittest.TestCase):
 def setUp(self):
  import tempfile,json
  from pathlib import Path
  self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
  root=Path(self.temp.name);(root/'calibrations').mkdir();(root/'captures').mkdir()
  name='capture-20000101-000000-000000.csv'
  (root/'calibrations'/'motion-test.json').write_text(json.dumps({'receiver_mask':8,'files':[name],'held_out_rms':0.01}))
  (root/'captures'/(name+'.json')).write_text(json.dumps({'device_port':'synthetic-device','firmware':'0.2.7-diagnostics','mask':8,'spectrum':{'4':{'clipped':False,'amplitudes':[{'xyz_source_counts':[1,2,3]}]*3}}}))
  scope=patch.object(server,'ROOT',root);scope.start();self.addCleanup(scope.stop)
  registration=patch('glove_tracking.register_to_references',return_value={'rotation':[[1,0,0],[0,1,0],[0,0,1]]});registration.start();self.addCleanup(registration.stop)
 def test_saved_j4_profile_loads_with_registration(self):
  h=server.Hub();h.port='synthetic-device'
  result=h.action('glove_load',{})['diagnostics']
  self.assertIn('4',result['glove_profiles'])
  self.assertIsNotNone(result['glove_profiles']['4']['registration'])
  self.assertEqual(result['glove_profile_errors'],[])
 def test_incompatible_device_reports_reason(self):
  h=server.Hub();h.port='different-device'
  result=h.action('glove_load',{})['diagnostics']
  self.assertEqual(result['glove_profiles'],{})
  self.assertIn('Device mismatch',result['glove_profile_errors'][0]['reason'])

class StreamSchedulingTests(unittest.TestCase):
 def test_early_burst_waits_instead_of_stopping_session(self):
  import time
  h=server.Hub();h.status={'found':1,'firmware':'0.2.7-diagnostics'};h.last_coil=time.monotonic()
  observed=[]
  def exchange(command):
   observed.append(time.monotonic()-start)
   return b'ERR fixture stops before hardware'
  h.exchange=exchange;start=time.monotonic()
  with self.assertRaisesRegex(ValueError,'fixture stops'):
   h.action('capture_stream',{'mask':1,'bench_ready':True})
  self.assertGreaterEqual(observed[0],.014)
