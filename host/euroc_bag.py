#!/usr/bin/env python3
"""Convert a EuRoC ROS1 bag into the replay CSV/PGM layout, without ROS.
Requires rosbags. Images and inertial samples retain their message timestamps.
"""
import argparse
import csv
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--seconds', type=float, help='Extract a prefix, measured from first image')
    args = parser.parse_args()
    from rosbags.highlevel import AnyReader
    args.output.mkdir(parents=True, exist_ok=True)
    files = {}
    writers = {}
    counts = {'cam0': 0, 'cam1': 0, 'imu0': 0, 'leica': 0}
    previous = {}
    for sensor in counts:
        directory = args.output / sensor
        (directory / 'data').mkdir(parents=True, exist_ok=True)
        files[sensor] = (directory / 'data.csv').open('w', newline='')
        writers[sensor] = csv.writer(files[sensor])
        writers[sensor].writerow(['#timestamp_ns', 'filename'] if sensor.startswith('cam') else
                                ['#timestamp_ns', 'wx', 'wy', 'wz', 'ax', 'ay', 'az'] if sensor == 'imu0' else
                                ['#timestamp_ns', 'x', 'y', 'z'])
    topics = {'/cam0/image_raw': 'cam0', '/cam1/image_raw': 'cam1', '/imu0': 'imu0', '/leica/position': 'leica'}
    image_start = None
    try:
        with AnyReader([args.bag]) as reader:
            connections = [c for c in reader.connections if c.topic in topics]
            for connection, _, raw in reader.messages(connections=connections):
                sensor = topics[connection.topic]
                message = reader.deserialize(raw, connection.msgtype)
                stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
                if image_start is not None and args.seconds is not None and stamp > image_start + args.seconds * 1e9:
                    continue
                if sensor in previous and stamp <= previous[sensor]:
                    raise ValueError(f'{sensor} timestamps are not strictly increasing')
                previous[sensor] = stamp
                if sensor.startswith('cam'):
                    if image_start is None:
                        image_start = stamp
                    if message.encoding != 'mono8' or message.step < message.width:
                        raise ValueError('Expected EuRoC mono8 images')
                    filename = f'{stamp}.pgm'
                    pixels = memoryview(message.data).tobytes()
                    if len(pixels) != message.height * message.step:
                        raise ValueError('Invalid image stride/data length')
                    with (args.output / sensor / 'data' / filename).open('wb') as out:
                        out.write(f'P5\n{message.width} {message.height}\n255\n'.encode())
                        for y in range(message.height):
                            out.write(pixels[y*message.step:y*message.step+message.width])
                    writers[sensor].writerow([stamp, filename])
                elif sensor == 'imu0':
                    w, a = message.angular_velocity, message.linear_acceleration
                    writers[sensor].writerow([stamp, w.x, w.y, w.z, a.x, a.y, a.z])
                else:
                    p = message.point
                    writers[sensor].writerow([stamp, p.x, p.y, p.z])
                counts[sensor] += 1
    finally:
        for stream in files.values():
            stream.close()
    (args.output / 'extraction.json').write_text(json.dumps({'bag': str(args.bag), 'counts': counts,
        'prefix_seconds': args.seconds, 'image_timestamp_start_ns': image_start}, indent=2)+'\n')
    print(json.dumps(counts))


if __name__ == '__main__':
    main()
