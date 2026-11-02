import unittest,tempfile,pathlib,subprocess
ROOT=pathlib.Path(__file__).resolve().parents[1]
class SourceGuard(unittest.TestCase):
 def test_actual_source_driver_with_mock_peripherals(self):
  with tempfile.TemporaryDirectory() as tmp:
   exe=pathlib.Path(tmp)/'source-test'
   subprocess.run(['cc','-Wno-pointer-to-int-cast','-I'+str(ROOT/'tests/source_mock'),'-I'+str(ROOT/'include'),str(ROOT/'src/source.c'),str(ROOT/'tests/source_mock/test_source.c'),'-o',str(exe)],check=True)
   subprocess.run([str(exe)],check=True)
