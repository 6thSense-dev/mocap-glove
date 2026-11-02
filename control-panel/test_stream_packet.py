import io,unittest
from stream_packet import parse_stream
class StreamPacketTests(unittest.TestCase):
 def packet(self):return b'SENSOR 1 clipped=0\n'+b''.join(f'TONE {hz} 1000 -2000 0 0 3000 4000\n'.encode() for hz in (4500,5625,6750))+b'END\n'
 def test_valid(self):
  s,t,imu=parse_stream(io.BytesIO(self.packet()),b'TRACK mask=1 samples=128 rate=24000 skip=64 phase_ticks=32,4294967295\n',1)
  self.assertEqual(t,[32,4294967295]);self.assertEqual(s['1']['analyzed_samples'],64);self.assertAlmostEqual(s['1']['amplitudes'][0]['xyz_counts'][2],5)
 def test_wrong_mask_and_incomplete_rejected(self):
  for header,payload,mask in [(b'TRACK mask=1 samples=128 rate=24000 skip=64 phase_ticks=32,4294967295\n',self.packet(),2),(b'TRACK mask=1 samples=128 rate=24000 skip=64 phase_ticks=32,4294967295\n',self.packet()[:-4],1)]:
   with self.assertRaises(ValueError):parse_stream(io.BytesIO(payload),header,mask)
 def test_missing_phase_remains_missing(self):
  s,_,imu=parse_stream(io.BytesIO(self.packet()),b'TRACK mask=1 samples=128 rate=24000 skip=64 phase_ticks=4294967295,4294967295\n',1)
  self.assertIsNone(s['1']['amplitudes'][0]['xyz_source_counts'])
 def test_combined_imu_packet(self):
  payload=self.packet()[:-4]+b'IMU accel=0,0,4096 gyro=0,0,0 temp_raw=123 time=456\nEND\n'
  s,t,imu=parse_stream(io.BytesIO(payload),b'TRACK mask=1 samples=128 rate=24000 skip=64 phase_ticks=32,4294967295 imu=1\n',1)
  self.assertIn('time=456',imu);self.assertEqual(len(s),1)
 def test_all_five_receivers(self):
  one=self.packet()[:-4];payload=b''.join(one.replace(b'SENSOR 1',f'SENSOR {i}'.encode()) for i in range(1,6))+b'END\n'
  s,_,_=parse_stream(io.BytesIO(payload),b'TRACK mask=31 samples=128 rate=24000 skip=64 phase_ticks=32,40\n',31)
  self.assertEqual(set(s),set('12345'))
 def test_wrong_tone_order_rejected(self):
  with self.assertRaises(ValueError):parse_stream(io.BytesIO(self.packet().replace(b'TONE 4500',b'TONE 5625')),b'TRACK mask=1 samples=128 rate=24000 skip=64 phase_ticks=32,40\n',1)
