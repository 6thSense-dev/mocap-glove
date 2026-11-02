"""Strict compact lock-in packet parser; no raw waveform implied."""
import re,math

def parse_stream(link,header,expected_mask):
 m=re.fullmatch(rb'TRACK mask=(\d+) samples=128 rate=24000 skip=64 phase_ticks=(\d+),(\d+)( imu=1)?\n',header)
 if not m or int(m[1])!=expected_mask:raise ValueError('Invalid streaming header')
 ticks=[int(m[2]),int(m[3])]
 if any(t!=4294967295 and not 0<=t<655360 for t in ticks):raise ValueError('Invalid phase ticks')
 has_imu=m[4] is not None
 spectrum={}
 for sensor in range(1,6):
  if not expected_mask&(1<<(sensor-1)):continue
  m=re.fullmatch(rb'SENSOR (\d) clipped=([01])\n',link.readline())
  if not m or int(m[1])!=sensor:raise ValueError('Missing or out-of-order receiver')
  sp={'clipped':m[2]==b'1','analyzed_samples':64,'settling_samples_ignored':64,'amplitudes':[]}
  for hz in (4500,5625,6750):
   m=re.fullmatch(rb'TONE (\d+) (-?\d+) (-?\d+) (-?\d+) (-?\d+) (-?\d+) (-?\d+)\n',link.readline())
   if not m or int(m[1])!=hz:raise ValueError('Invalid tone packet')
   values=[int(m[i]) for i in range(2,8)]
   if any(abs(v)>65536000 for v in values):raise ValueError('Tone outside ADC range')
   z=[complex(values[i]/1000,values[i+1]/1000) for i in (0,2,4)];tick=ticks[0 if sensor<=3 else 1]
   phase=2*math.pi*hz*(tick/122880000+64/24000) if tick!=4294967295 else None
   correction=complex(math.cos(phase),-math.sin(phase)) if phase is not None else None
   corrected=[v*correction for v in z] if correction is not None else None
   sp['amplitudes'].append({'hz':hz,'xyz_counts':[abs(v) for v in z],'xyz_complex_counts':[[v.real,v.imag] for v in z],'xyz_source_counts':[[v.real,v.imag] for v in corrected] if corrected else None})
  spectrum[str(sensor)]=sp
 imu=None
 if has_imu:
  imu=link.readline().decode('ascii').strip()
  if not re.fullmatch(r'IMU accel=-?\d+,-?\d+,-?\d+ gyro=-?\d+,-?\d+,-?\d+ temp_raw=-?\d+ time=\d+',imu):raise ValueError('Invalid IMU frame')
 if link.readline()!=b'END\n':raise ValueError('Incomplete streaming packet')
 return spectrum,ticks,imu
