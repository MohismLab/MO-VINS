#!/usr/bin/env python3
"""Exercise real serial bytes over a PTY, no device or pymavlink dependency."""
import os
import pty
import select
import struct
import subprocess
import tempfile
import time
import tty
from pathlib import Path

EXTRAS = {0: 50, 20: 214, 21: 159, 22: 220, 23: 168}

def crc(data):
    value = 0xffff
    for b in data:
        t = b ^ (value & 255)
        t ^= (t << 4) & 255
        value = (value >> 8) ^ (t << 8) ^ (t << 3) ^ (t >> 4)
    return value

def packet(mid, payload):
    header = bytes([len(payload), 0, 255, 190, mid])
    return b'\xfe' + header + payload + struct.pack('<H', crc(header + payload + bytes([EXTRAS[mid]])))

class Stream:
    def __init__(self, fd):
        self.fd = fd
        self.buffer = bytearray()

    def receive(self, timeout=2):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            while self.buffer:
                magic = self.buffer[0]
                if magic not in (0xfd, 0xfe):
                    del self.buffer[0]
                    continue
                head = 10 if magic == 0xfd else 6
                if len(self.buffer) < head:
                    break
                length = self.buffer[1]
                total = head + length + 2
                if len(self.buffer) < total:
                    break
                raw = bytes(self.buffer[:total])
                del self.buffer[:total]
                mid = int.from_bytes(raw[7:10], 'little') if magic == 0xfd else raw[5]
                assert crc(raw[1:-2] + bytes([EXTRAS[mid]])) == int.from_bytes(raw[-2:], 'little')
                yield mid, raw[head:head+length]
            if select.select([self.fd], [], [], max(0, end-time.monotonic()))[0]:
                self.buffer.extend(os.read(self.fd, 4096))

def main():
    master, slave = pty.openpty()
    tty.setraw(slave)
    stream = Stream(master)
    with tempfile.TemporaryDirectory() as directory:
        config = Path(directory) / 'config.yml'
        params = Path(directory) / 'params.yml'
        config.write_text(f'%YAML:1.0\ncalibrated: 0\nserial: "{os.ttyname(slave)}"\nuvc: ""\nparams: "{params}"\n')
        def launch():
            return subprocess.Popen(['out/host/mohism-vins', str(config)], stderr=subprocess.DEVNULL)
        process = launch()
        try:
            time.sleep(.1)
            os.write(master, packet(21, bytes([1,197])))
            values = {}
            for mid, payload in stream.receive(.4):
                if mid == 22:
                    payload = payload.ljust(25,b'\0')
                    value, count, index, name, kind = struct.unpack('<fHH16sB', payload)
                    values[name.rstrip(b'\0').decode()] = value
            assert len(values) == 8 and values['VIO_VALID'] == 0, values
            assert values['VIO_ENABLE'] == 1
            def set_value(name, value):
                os.write(master, packet(23, struct.pack('<fBB16sB', value, 1, 197, name.encode(), 9)))
                result = None
                for mid, payload in stream.receive(.15):
                    if mid == 22:
                        val, _, _, ident, _ = struct.unpack('<fHH16sB', payload.ljust(25,b'\0'))
                        if ident.rstrip(b'\0').decode() == name:
                            result = val
                return result
            assert set_value('SYNC_MAX_MS', 3.5) == 3.5
            assert set_value('SYNC_MAX_MS', float('nan')) == 3.5
            assert set_value('SYNC_MAX_MS', 100) == 3.5
            assert set_value('VIO_VALID', 1) == 0
            assert set_value('VIO_ENABLE', 0) == 0
            assert abs(set_value('ACC_NOISE', .25)-.25) < 1e-6
            assert abs(set_value('GYRO_NOISE', -1)-.01) < 1e-6
            process.terminate();process.wait(timeout=2)
            process = launch();time.sleep(.1)
            os.write(master, packet(21, bytes([1,197])))
            persisted = {}
            for mid, payload in stream.receive(.3):
                if mid == 22:
                    val, _, _, ident, _ = struct.unpack('<fHH16sB', payload.ljust(25,b'\0'))
                    persisted[ident.rstrip(b'\0').decode()] = val
            assert persisted['VIO_ENABLE'] == 0 and persisted['SYNC_MAX_MS'] == 3.5, persisted
            assert persisted['ACC_NOISE'] == .25
            print('PASS MAVLink wire CRC, enumeration, write/read-only/range checks, persistence across restart')
        finally:
            process.terminate();process.wait(timeout=2)
    os.close(master);os.close(slave)

if __name__ == '__main__':
    main()
