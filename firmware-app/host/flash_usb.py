#!/usr/bin/env python3
"""Flash only the V6 STM32G474RE hub. Backs up main flash and verifies readback."""
import argparse, datetime, hashlib, pathlib, re, shutil, struct, subprocess, sys
ROOT=pathlib.Path(__file__).resolve().parents[1]

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dfu-util');p.add_argument('--serial');p.add_argument('--flash',action='store_true')
    args=p.parse_args()
    tool=args.dfu_util or shutil.which('dfu-util')
    if not tool:
        paths=sorted((pathlib.Path.home()/'Library/Arduino15/packages/arduino/tools/dfu-util').glob('*/dfu-util'))
        if paths: tool=str(paths[-1])
    if not tool: raise RuntimeError('dfu-util not found. Use STM32CubeProgrammer as explained in FIRST-FLASH.txt.')
    listing=subprocess.run([tool,'-d','0483:df11','-a','0','-l'],capture_output=True,text=True)
    text=listing.stdout+listing.stderr
    print(text)
    if listing.returncode: raise RuntimeError('DFU listing failed')
    devices=[]
    for line in text.splitlines():
        if 'Found DFU:' in line and 'alt=0,' in line and 'Internal Flash' in line:
            match=re.search(r'serial="([^"]+)"',line)
            if match: devices.append(match[1])
    devices=sorted(set(devices))
    serial=args.serial
    if serial and serial not in devices: raise RuntimeError('Requested DFU serial is not connected')
    if not serial:
        if len(devices)!=1: raise RuntimeError('Connect one glove in ROM DFU mode, or choose --serial from the listing.')
        serial=devices[0]
    if not args.flash:
        print(f'Found DFU serial {serial}. No flash written. Run again with --flash after checking the V6 board.');return
    firmware=ROOT/'build/glove-v6-diagnostics.bin';blob=firmware.read_bytes()
    hashes=(ROOT/'SHA256SUMS.txt').read_text().splitlines()
    expected=next(line.split()[0] for line in hashes if line.endswith('build/glove-v6-diagnostics.bin'))
    if hashlib.sha256(blob).hexdigest()!=expected: raise RuntimeError('Firmware checksum mismatch')
    sp,reset=struct.unpack('<II',blob[:8])
    if sp!=0x20018000 or not reset&1 or not 0x08000000<=reset<0x08000000+len(blob): raise RuntimeError('Invalid V6 flash image')
    print('Diagnostics 0.2.7: transmitter tests are available, but remain OFF until explicitly requested.')
    print('Confirm: V6 hub, STM32G474RET6; coil outputs remain off during flashing. Keep attached coil leads insulated.')
    print(f'Target serial: {serial}. This writes application flash at 0x08000000.')
    if input('Type FLASH to continue: ').strip()!='FLASH': print('Cancelled.');return
    folder=ROOT/'flash-records'/datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
    folder.mkdir(parents=True)
    base=[tool,'-d','0483:df11','-S',serial,'-a','0']
    def run(extra,name):
        result=subprocess.run(base+extra,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
        print(result.stdout);(folder/name).write_text(result.stdout)
        if result.returncode: raise RuntimeError(f'DFU operation failed. See {folder}. Do not change option bytes or unprotect.')
    backup=folder/'original-main-flash.bin'
    run(['-s','0x08000000:524288','-U',str(backup)],'01-backup.log')
    if backup.stat().st_size!=524288: raise RuntimeError('Incomplete flash backup; programming cancelled')
    run(['-s','0x08000000','-D',str(firmware)],'02-program.log')
    readback=folder/'verified-readback.bin'
    run(['-s',f'0x08000000:{len(blob)}','-U',str(readback)],'03-readback.log')
    if readback.read_bytes()!=blob: raise RuntimeError('Readback differs from image. Keep board in DFU and investigate.')
    (folder/'VERIFIED.txt').write_text(f'Serial {serial}\nSHA256 {expected}\nByte-for-byte readback matched. Hardware function not tested.\n')
    print('Flash readback VERIFIED. Release BOOT; tap RESET. Then run Start-console-Mac.command.')

if __name__=='__main__':
    try: main()
    except (RuntimeError,OSError,StopIteration,EOFError) as e: sys.exit(str(e))
