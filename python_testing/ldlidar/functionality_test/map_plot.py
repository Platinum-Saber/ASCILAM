import serial
import struct
import matplotlib.pyplot as plt
import numpy as np
from collections import deque

PORT = "COM2"
BAUDRATE = 230400
POINTS_PER_PACK = 12
HEADER = 0x54

# Store up to 2000 recent points (~several full scans)
history = deque(maxlen=2000)

def read_frame(ser):
    while True:
        byte = ser.read(1)
        if byte and byte[0] == HEADER:
            rest = ser.read(45)
            if len(rest) == 45 and rest[0] == 0x2C:
                return byte + rest

def parse_packet(packet):
    start_angle = struct.unpack_from("<H", packet, 4)[0] / 100.0
    points = []
    for i in range(POINTS_PER_PACK):
        offset = 6 + i * 3
        dist = struct.unpack_from("<H", packet, offset)[0]
        intensity = packet[offset + 2]
        points.append((dist, intensity))
    end_angle = struct.unpack_from("<H", packet, 42)[0] / 100.0
    return start_angle, end_angle, points

def interpolate_angles(start_angle, end_angle):
    step = (end_angle - start_angle) / (POINTS_PER_PACK - 1)
    return [start_angle + step * i for i in range(POINTS_PER_PACK)]

def polar_to_cartesian(dist_mm, angle_deg):
    r = dist_mm / 1000.0
    theta = np.deg2rad(angle_deg)
    return r * np.cos(theta), r * np.sin(theta)

def main():
    with serial.Serial(PORT, BAUDRATE, timeout=1) as ser:
        plt.ion()
        fig, ax = plt.subplots()

        while True:
            packet = read_frame(ser)
            if not packet:
                continue

            start_angle, end_angle, points = parse_packet(packet)
            angles = interpolate_angles(start_angle, end_angle)

            for (dist, _), angle in zip(points, angles):
                if 0 < dist <= 1000:
                    x, y = polar_to_cartesian(dist, angle)
                    history.append((x, y))

            # Plot history
            ax.clear()
            xs, ys = zip(*history) if history else ([], [])
            ax.scatter(xs, ys, s=2, c='blue')
            ax.set_xlim(-1.2, 1.2)
            ax.set_ylim(-1.2, 1.2)
            ax.set_aspect("equal")
            ax.grid(True)
            ax.set_title("LD19 LiDAR - Cumulative Map (≤1m)")
            plt.pause(0.001)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Stopped.")
