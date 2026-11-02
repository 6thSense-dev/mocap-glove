import unittest,math
from unittest.mock import patch
import server
from test_panel import FakeSerial
class DiagSerial(FakeSerial):
 def readline(self):
  c=self.writes[-1]
  if c==b'status\n':return f'OK firmware=0.2.0-diagnostics core=122880000 sensors={self.power} found={self.found} coil_drive_build=1\n'.encode()
  if c.startswith(b'coil_test'):return b'COIL result=ok axes=1 ms=10 samples=100 vdda_mv=3300 zero=2048,2048,2048 min=1648,2048,2048 max=2448,2048,2048\n'
  if c==b'currents\n':return b'CURRENT raw=2048,2048,2048 vdda_mv=3300 gain=20 shunt_mohm=200\n'
  if c==b'imu probe\n':return b'IMU on=0 id=0043 error=0000\n'
  return super().readline()
class Diagnostics(unittest.TestCase):
 def setUp(self):
  self.h=server.Hub();self.h.ports=lambda:[{'port':'test'}]
  with patch.object(server.serial,'Serial',DiagSerial),patch.object(server.time,'sleep'):self.h.action('connect',{'port':'test'})
 def ready(self):self.h.action('power_on',{})
 def test_connect_never_drives(self):self.assertEqual(self.h.link.writes,[b'status\n'])
 def test_needs_power(self):
  with self.assertRaises(ValueError):self.h.action('coil_test',{'axes':1,'ms':10,'bench_ready':True})
 def test_needs_readiness(self):
  self.ready()
  with self.assertRaises(ValueError):self.h.action('coil_test',{'axes':1,'ms':10})
  self.assertFalse(any(c.startswith(b'coil_test') for c in self.h.link.writes))
 def test_bounds(self):
  self.ready()
  for axes,ms in [(0,10),(8,10),(1,0),(1,101),(-1,10)]:
   with self.assertRaises(ValueError):self.h.action('coil_test',{'axes':axes,'ms':ms,'bench_ready':True})
 def test_current_conversion_and_cooldown(self):
  self.ready();s=self.h.action('coil_test',{'axes':1,'ms':10,'bench_ready':True})
  self.assertAlmostEqual(s['diagnostics']['coil_test']['peak_ma'][0],80.59,places=2)
  with self.assertRaises(ValueError):self.h.action('coil_test',{'axes':1,'ms':10,'bench_ready':True})
 def test_emergency_bypasses_transaction(self):
  self.h.emergency();self.assertEqual(self.h.link.writes[-1],b'!')
 def test_currents_and_imu(self):
  self.ready();self.assertIn('currents',self.h.action('currents',{})['diagnostics']);self.assertIn('imu_probe',self.h.action('imu_probe',{})['diagnostics'])
 def test_capture_drive_requires_readiness(self):
  self.ready();self.h.action('probe',{})
  with self.assertRaises(ValueError):self.h.action('capture_tx',{'mask':1,'axes':1})
 def test_live_rejects_old_firmware(self):
  self.ready();self.h.action('probe',{})
  with self.assertRaises(ValueError):self.h.action('capture_live',{'mask':1,'axes':4,'bench_ready':True})
 def test_live_rejects_multiple_receivers_and_wrong_axis(self):
  self.ready();self.h.status.update(firmware='0.2.3-diagnostics',found=31)
  for mask,axes in [(3,4),(1,1),(1,7)]:
   with self.assertRaises(ValueError):self.h.action('capture_live',{'mask':mask,'axes':axes,'bench_ready':True})
  self.assertFalse(any(c.startswith(b'capture_live') for c in self.h.link.writes))
 def test_unknown_firmware_rejected(self):
  self.h.link.readline=lambda:b'OK firmware=unknown core=1 sensors=1 found=1 coil_drive_build=1\n'
  with self.assertRaises(ValueError):self.h.action('status',{})
  self.assertIsNone(self.h.link)
 def test_spectrum_separates_three_sources(self):
  rows=[(1,i,0,1000*math.cos(2*math.pi*4500*i/24000),2000*math.sin(2*math.pi*5625*i/24000),3000*math.cos(2*math.pi*6750*i/24000),0) for i in range(256)]
  a=server.spectrum(rows)['1']['amplitudes'];self.assertEqual([a[i]['xyz_counts'][i] for i in range(3)],[1000,2000,3000]);self.assertEqual(a[0]['xyz_counts'][1],0)
 def test_complex_phase_keeps_relative_winding_sign(self):
  rows=[(1,i,0,1000*math.cos(2*math.pi*4500*i/24000+.4),-500*math.cos(2*math.pi*4500*i/24000+.4),0,0) for i in range(256)]
  tone=server.spectrum(rows)['1']['amplitudes'][0]
  a,b,_=[complex(*v) for v in tone['xyz_complex_counts']]
  self.assertAlmostEqual((b/a).real,-.5,places=6)
  self.assertAlmostEqual((b/a).imag,0,places=6)
  self.assertIn('not transmitter-referenced',tone['phase_reference'])
 def test_source_phase_correction_removes_capture_offset(self):
  # ADC/filter latency intentionally absent in synthetic data, not hardware validation.
  for sensor,bus in [(1,0),(4,1)]:
   for tick in [32,14567,655359]:
    frequency=5625;phase=2*math.pi*frequency*tick/122880000
    rows=[(sensor,i,0,1000*math.cos(2*math.pi*frequency*i/24000+phase+.3),0,0,0) for i in range(256)]
    ticks=[4294967295]*2;ticks[bus]=tick
    a=server.spectrum(rows,ticks)[str(sensor)]['amplitudes'][1]['xyz_source_counts'][0]
    self.assertAlmostEqual(a[0],1000*math.cos(.3),places=5)
    self.assertAlmostEqual(a[1],1000*math.sin(.3),places=5)
 def test_unknown_source_phase_is_not_fabricated(self):
  rows=[(1,i,0,100,0,0,0) for i in range(256)]
  self.assertIsNone(server.spectrum(rows,[4294967295]*2)['1']['amplitudes'][0]['xyz_source_counts'])
 def test_clip_flag(self):self.assertTrue(server.spectrum([(1,i,0,32767,0,0,0) for i in range(256)])['1']['clipped'])
 def test_startup_excluded_but_reported(self):
  rows=[(1,i,0,32767 if i<3 else 1000*math.cos(2*math.pi*6750*i/24000),0,0,0) for i in range(256)]
  s=server.spectrum(rows)['1'];self.assertTrue(s['raw_clipped']);self.assertFalse(s['clipped']);self.assertEqual(s['analyzed_samples'],192);self.assertEqual(s['amplitudes'][2]['xyz_counts'][0],1000)
 def test_late_clip_not_hidden(self):
  rows=[(1,i,0,32767 if i==100 else 0,0,0,0) for i in range(256)]
  self.assertTrue(server.spectrum(rows)['1']['clipped'])
if __name__=='__main__':unittest.main()
