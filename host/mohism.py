#!/usr/bin/env python3
"""Laptop control and lossless stereo/depth unpacking for MOHISM dual-UVC profile v2."""
import argparse
from pathlib import Path
import time

def unpack_stereo(raw):
    import numpy as np
    data = np.asarray(raw)
    if data.dtype != np.uint8 or data.nbytes != 256000:
        raise ValueError('Expected 256000 raw YUYV bytes; disable RGB conversion')
    data = data.reshape(200, 1280)
    return data[:, :640:2].copy(), data[:, 640::2].copy()


def unpack_depth(raw):
    import numpy as np
    data = np.asarray(raw)
    if data.dtype not in (np.dtype('uint8'), np.dtype('<u2')) or data.nbytes != 128000:
        raise ValueError('Expected 128000 raw Z16 bytes')
    return np.ascontiguousarray(data).view('<u2').reshape(200, 320).copy()


def images(args):
    import cv2
    import threading
    import concurrent.futures
    if not args.stereo and not args.depth:
        raise ValueError('Specify --stereo and/or --depth capture device')
    if args.stereo and args.stereo == args.depth:
        raise ValueError('Stereo and depth require different devices')
    if args.frames <= 0:
        raise ValueError('--frames must be positive')
    stop = threading.Event()

    def capture(path, kind, fps):
        cap = cv2.VideoCapture(path, cv2.CAP_V4L2)
        directory = args.output / kind
        try:
            if not cap.isOpened():
                raise RuntimeError(f'Cannot open {kind} UVC device: {path}')
            fourcc = 'YUYV' if kind == 'stereo' else 'Z16 '
            for key, value in [(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc)),
                               (cv2.CAP_PROP_FRAME_WIDTH, 640 if kind == 'stereo' else 320),
                               (cv2.CAP_PROP_FRAME_HEIGHT, 200), (cv2.CAP_PROP_FPS, fps),
                               (cv2.CAP_PROP_CONVERT_RGB, 0)]:
                if not cap.set(key, value):
                    raise RuntimeError(f'{kind} backend rejected raw format/property {key}')
            if int(cap.get(cv2.CAP_PROP_FOURCC)) != cv2.VideoWriter_fourcc(*fourcc):
                raise RuntimeError(f'{kind} backend negotiated an unexpected pixel format')
            directory.mkdir(parents=True, exist_ok=True)
            # Separate counters and host arrival times: these are not exposure timestamps.
            with (directory / 'arrival.csv').open('w') as log:
                log.write('index,host_monotonic_ns\n')
                for index in range(args.frames):
                    if stop.is_set():
                        break
                    ok, raw = cap.read()
                    arrival = time.monotonic_ns()
                    if not ok:
                        raise RuntimeError(f'{kind} USB frame read failed')
                    if kind == 'stereo':
                        left, right = unpack_stereo(raw)
                        frames = [('left', left), ('right', right)]
                    else:
                        frames = [('depth-mm', unpack_depth(raw))]
                    for name, data in frames:
                        if not cv2.imwrite(str(directory / f'{index:06d}-{name}.png'), data):
                            raise RuntimeError('Image write failed')
                    log.write(f'{index},{arrival}\n')
        except BaseException:
            stop.set()
            raise
        finally:
            cap.release()

    # Read both independent endpoints concurrently so a depth read cannot stall stereo.
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
        jobs = []
        if args.stereo:
            jobs.append(pool.submit(capture, args.stereo, 'stereo', args.fps))
        if args.depth:
            jobs.append(pool.submit(capture, args.depth, 'depth', args.depth_fps))
        for job in concurrent.futures.as_completed(jobs):
            job.result()


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
    capture.add_argument('--stereo', help='Stereo YUYV capture node')
    capture.add_argument('--depth', help='Depth Z16 capture node')
    capture.add_argument('--depth-fps', type=int, choices=[15, 30, 60], default=30)
    capture.add_argument('--output', type=Path, default=Path('capture'))
    capture.add_argument('--fps', type=int, choices=[15, 30, 60], default=60)
    capture.add_argument('--frames', type=int, default=30)
    args = parser.parse_args()
    images(args) if args.command == 'images' else control(args)

if __name__ == '__main__':
    main()
