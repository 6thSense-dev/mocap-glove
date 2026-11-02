import unittest,ctypes,subprocess,tempfile,pathlib,math
class LockinTest(unittest.TestCase):
 def test_native_demodulator(self):
  root=pathlib.Path(__file__).resolve().parents[1]
  with tempfile.TemporaryDirectory() as d:
   lib=pathlib.Path(d)/'lockin.dylib';subprocess.run(['cc','-shared','-fPIC','-O2','-I'+str(root/'include'),str(root/'src/lockin.c'),'-o',str(lib)],check=True)
   fn=ctypes.CDLL(str(lib)).lockin64
   samples=((ctypes.c_int16*3)*64)();out=(((ctypes.c_int32*2)*3)*3)()
   for n in range(64):
    for ch in range(3):samples[n][ch]=round(1500+1000*math.cos(2*math.pi*(12+3*ch)*n/64+.4))
   fn(samples,out)
   for tone in range(3):
    for ch in range(3):
     expected=complex(1000*math.cos(.4),1000*math.sin(.4)) if tone==ch else 0j
     measured=complex(out[tone][ch][0],out[tone][ch][1])/1000
     self.assertLess(abs(expected-measured),.5)
