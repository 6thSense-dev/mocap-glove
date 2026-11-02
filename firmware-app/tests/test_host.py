import io,sys,pathlib,unittest,struct,ctypes,subprocess,tempfile
ROOT=pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'host'))
from glove import crc16,parse_capture,capture_metadata
class Tests(unittest.TestCase):
    def fixture(self):
        data=struct.pack('>Hhhhh',0,32767,-32768,-1,0)
        frame=data+crc16(data).to_bytes(2,'big')
        return b'SENSOR 1 bytes=3072\n'+frame*256+b'\nEND\n'
    def test_crc_reference(self): self.assertEqual(crc16(b'123456789'),0x29b1)
    def test_roundtrip_signed(self):
        rows=parse_capture(io.BytesIO(self.fixture()),b'CAPTURE mask=1 samples=256 rate=24000 format=ADS16_BE12\n')
        self.assertEqual(len(rows),256); self.assertEqual(rows[-1],(1,255,0,32767,-32768,-1,0))
    def test_phase_header(self):
        h=b'CAPTURE mask=1 samples=256 rate=24000 format=ADS16_BE12 phase_ticks=655359,4294967295\n'
        self.assertEqual(capture_metadata(h)['phase_ticks'],[655359,4294967295])
        self.assertEqual(len(parse_capture(io.BytesIO(self.fixture()),h)),256)
        with self.assertRaises(ValueError):capture_metadata(h.replace(b'655359',b'655360'))
    def test_truncated(self):
        with self.assertRaises(ValueError): parse_capture(io.BytesIO(self.fixture()[:100]),b'CAPTURE mask=1 samples=256 rate=24000 format=ADS16_BE12\n')
    def test_corruption(self):
        raw=bytearray(self.fixture());raw[30]^=1
        with self.assertRaises(ValueError):parse_capture(io.BytesIO(raw),b'CAPTURE mask=1 samples=256 rate=24000 format=ADS16_BE12\n')
    def test_sensor_mismatch(self):
        with self.assertRaises(ValueError):parse_capture(io.BytesIO(self.fixture()),b'CAPTURE mask=2 samples=256 rate=24000 format=ADS16_BE12\n')
    def test_firmware_crc(self):
        with tempfile.TemporaryDirectory() as tmp:
            lib=pathlib.Path(tmp)/'crc.so'
            subprocess.run(['cc','-shared','-fPIC',str(ROOT/'src/crc16.c'),'-o',str(lib)],check=True)
            crc=ctypes.CDLL(str(lib)).crc16;crc.argtypes=[ctypes.c_char_p,ctypes.c_uint];crc.restype=ctypes.c_uint16
            for data in (b'123456789',bytes(range(256)),b'\x00'*10,b'\xff'*15):self.assertEqual(crc(data,len(data)),crc16(data))
    def test_flash_vectors(self):
        blob=(ROOT/'build/glove-v6-diagnostics.bin').read_bytes();sp,reset=struct.unpack('<II',blob[:8])
        self.assertEqual(sp,0x20018000);self.assertEqual(reset&1,1);self.assertTrue(0x08000000<=reset<0x08000000+len(blob));self.assertLess(len(blob),512*1024)
if __name__=='__main__':unittest.main(verbosity=2)
