#!/usr/bin/env python3
"""Laptop control and lossless stereo/depth unpacking for MOHISM profile v1."""
import argparse
from pathlib import Path
import time

def unpack(raw):
    import numpy as np
    data = np.asarray(raw, dtype=np.uint8)
    if data.size != 384000:
        raise ValueError(f'Expected 384000 raw YUYV bytes, got {data.size}; disable RGB conversion')
    data = data.reshape(200, 1920)
    left = data[:, :640:2].copy()
    right = data[:, 640:1280:2].copy()
    depth = data[:, 1280:].copy().view('<u2').reshape(200, 320)
    return left, right, depth

def images(args):
    import cv2
    import numpy as np
    cap = cv2.VideoCapture(args.video, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 200)
    cap.set(cv2.CAP_PROP_FPS, 15)
    if not cap.set(cv2.CAP_PROP_CONVERT_RGB, 0):
        raise RuntimeError('Driver/backend cannot disable RGB conversion; depth would be corrupted')
    if not cap.isOpened():
        raise RuntimeError('Cannot open UVC capture device')
    args.output.mkdir(parents=True, exist_ok=True)
    try:
        for index in range(args.frames):
            ok, raw = cap.read()
            if not ok:
                raise RuntimeError('USB frame read failed')
            left, right, depth = unpack(raw)
            for name, data in [('left', left), ('right', right), ('depth-mm', depth)]:
                if not cv2.imwrite(str(args.output / f'{index:06d}-{name}.png'), data):
                    raise RuntimeError('Image write failed')
            if args.show:
                preview = np.hstack((left, right, np.minimum(depth/20, 255).astype(np.uint8)))
                cv2.imshow('left | right | depth preview (PNG retains uint16 mm)', preview)
                if cv2.waitKey(1) == 27:
                    break
    finally:
        cap.release()
        cv2.destroyAllWindows()

def control(args):
    from pymavlink import mavutil
    connection = mavutil.mavlink_connection(args.serial, baud=115200, source_system=255)
    heartbeat = connection.wait_heartbeat(timeout=5)
    if heartbeat is None:
        raise TimeoutError('No MOHISM MAVLink heartbeat received')
    system, component = heartbeat.get_srcSystem(), heartbeat.get_srcComponent()
    if args.command == 'params':
        connection.mav.param_request_list_send(system, component)
        params = {}
        deadline = time.monotonic()+5
        while time.monotonic() < deadline:
            msg = connection.recv_match(type='PARAM_VALUE', blocking=True, timeout=.5)
            if msg is not None:
                params[msg.param_id] = msg.param_value
                if len(params) == msg.param_count:
                    break
        if len(params) != 8:
            raise TimeoutError(f'Incomplete parameter list: {params}')
        for key, value in params.items():
            print(f'{key}={value}')
    elif args.command == 'set':
        connection.mav.param_set_send(system, component, args.name.encode('ascii'), args.value, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        deadline = time.monotonic()+3
        while time.monotonic() < deadline:
            msg = connection.recv_match(type='PARAM_VALUE', blocking=True, timeout=.5)
            if msg is not None and msg.param_id == args.name:
                print(f'{msg.param_id}={msg.param_value}')
                if abs(msg.param_value-args.value) > 1e-5:
                    raise ValueError('Device rejected requested value')
                return
        raise TimeoutError('No parameter acknowledgement')
    else:
        while True:
            msg = connection.recv_match(type=['ODOMETRY','HEARTBEAT'], blocking=True, timeout=3)
            if msg is None:
                raise TimeoutError('MAVLink stream timed out')
            print(msg)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--serial', default='/dev/ttyACM0')
    sub = parser.add_subparsers(dest='command', required=True)
    sub.add_parser('params')
    setting = sub.add_parser('set')
    setting.add_argument('name'); setting.add_argument('value', type=float)
    sub.add_parser('odometry')
    capture = sub.add_parser('images')
    capture.add_argument('--video', default='/dev/video0')
    capture.add_argument('--output', type=Path, default=Path('capture'))
    capture.add_argument('--frames', type=int, default=30)
    capture.add_argument('--show', action='store_true')
    args = parser.parse_args()
    images(args) if args.command == 'images' else control(args)

if __name__ == '__main__':
    main()
