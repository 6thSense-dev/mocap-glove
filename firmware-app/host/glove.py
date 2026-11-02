#!/usr/bin/env python3
"""Persistent USB console and loss-checked bench capture reader. Python 3.9+."""
import argparse, csv, datetime, pathlib, re, struct, sys

def crc16(data):
    value = 0xffff
    for byte in data:
        value ^= byte << 8
        for _ in range(8):
            value = ((value << 1) ^ (0x1021 if value & 0x8000 else 0)) & 0xffff
    return value

def exact(stream, length):
    result = bytearray()
    while len(result) < length:
        part = stream.read(length-len(result))
        if not part:
            raise ValueError('Truncated capture or USB timeout; no valid capture saved')
        result.extend(part)
    return bytes(result)

def capture_metadata(header):
    match=re.fullmatch(rb'CAPTURE mask=(\d+) samples=256 rate=24000 format=ADS16_BE12(?: phase_ticks=(\d+),(\d+))?\n',header)
    if not match or not 1<=int(match[1])<=31:raise ValueError('Unexpected capture header')
    ticks=[int(match[2]),int(match[3])] if match[2] else None
    if ticks and any(t!=4294967295 and not 0<=t<655360 for t in ticks):raise ValueError('Invalid source phase timestamp')
    return {'mask':int(match[1]),'phase_ticks':ticks}

def parse_capture(stream, header):
    mask=capture_metadata(header)['mask']; rows=[]
    for sensor in range(1, 6):
        if not mask & (1 << (sensor-1)): continue
        if stream.readline() != f'SENSOR {sensor} bytes=3072\n'.encode():
            raise ValueError('Sensor order or byte count mismatch')
        raw = exact(stream, 3072)
        if exact(stream, 1) != b'\n': raise ValueError('Missing data separator')
        for index in range(256):
            frame = raw[index*12:(index+1)*12]
            if crc16(frame[:10]) != int.from_bytes(frame[10:], 'big'):
                raise ValueError(f'CRC mismatch: sensor {sensor}, sample {index}')
            status, x,y,z,aux,_ = struct.unpack('>HhhhhH', frame)
            rows.append((sensor,index,status,x,y,z,aux))
    if stream.readline() != b'END\n': raise ValueError('Missing capture end marker')
    return rows

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', help='USB serial port, e.g. /dev/cu.usbmodem... or COM5')
    parser.add_argument('--list',action='store_true')
    args=parser.parse_args()
    try:
        import serial
        from serial.tools import list_ports
    except ImportError:
        raise SystemExit('Install the dependency first: python3 -m pip install pyserial==3.5')
    ports=list(list_ports.comports())
    if args.list:
        for p in ports: print(p.device, p.description, p.hwid)
        return
    port=args.port
    if not port:
        candidates=[p.device for p in ports if p.vid==0x0483 and p.pid==0x5740]
        if len(candidates)!=1:
            raise SystemExit('Cannot uniquely select the glove. Run --list, then --port PORT.')
        port=candidates[0]
    with serial.Serial(port,115200,timeout=3,write_timeout=3) as connection:
        connection.dtr=True
        import time
        time.sleep(.3); connection.reset_input_buffer()
        connection.write(b'status\n')
        status=connection.readline().decode(errors='replace').strip()
        print(status)
        if not status.startswith(('OK firmware=0.1.0-bench ','OK firmware=0.2.0-diagnostics ')):
            raise ValueError('Unexpected firmware response; no sensor commands sent')
        print('Type help for commands; exit to disconnect. Diagnostics firmware permits bounded coil tests; read START-HERE before use.')
        print('First test: sensors on, probe, read 1. Capture 1 tests J1; capture 31 tests all five.')
        try:
            while True:
                command=input('glove> ').strip()
                if command in ('exit','quit'): break
                if not command: continue
                connection.write((command+'\n').encode())
                if command=='reset': print('Board resetting; reconnect the console.'); break
                header=connection.readline()
                if not header: raise ValueError('No USB response in 3 seconds')
                if header.startswith(b'CAPTURE'):
                    rows=parse_capture(connection,header)
                    path=pathlib.Path('captures')/('capture-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f')+'.csv')
                    path.parent.mkdir(exist_ok=True)
                    with path.open('w',newline='') as file:
                        writer=csv.writer(file); writer.writerow(['sensor','sample','status','x_raw','y_raw','z_raw','aux_raw']);writer.writerows(rows)
                    print(f'Saved {len(rows)} CRC-checked rows: {path.resolve()}')
                else: print(header.decode(errors='replace').strip())
        except (KeyboardInterrupt,EOFError): pass
        finally:
            if connection.is_open:
                try: connection.write(b'sensors off\n'); connection.flush()
                except serial.SerialException: pass

if __name__=='__main__':
    try: main()
    except (ValueError,OSError) as exc: sys.exit(str(exc))
