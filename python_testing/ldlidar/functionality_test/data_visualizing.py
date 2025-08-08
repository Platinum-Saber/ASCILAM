import serial
import struct
import matplotlib.pyplot as plt
import numpy as np

PORT = "COM2"
BAUDRATE = 230400

HEADER = 0x54
POINTS_PER_PACK = 12

def read_frame(ser):
    while True:
        byte = ser.read(1)
        if byte and byte[0] == HEADER:
            rest = ser.read(45)  # read remaining 45 bytes of frame
            if len(rest) == 45 and rest[0] == 0x2C:
                return byte + rest

def parse_packet(packet):
    _, ver_len = packet[0], packet[1]
    speed = struct.unpack_from("<H", packet, 2)[0]
    start_angle = struct.unpack_from("<H", packet, 4)[0] / 100.0
    points = []
    for i in range(POINTS_PER_PACK):
        offset = 6 + i * 3
        distance = struct.unpack_from("<H", packet, offset)[0]
        intensity = packet[offset + 2]
        points.append((distance, intensity))
    end_angle = struct.unpack_from("<H", packet, 42)[0] / 100.0
    return start_angle, end_angle, points

def interpolate_angles(start_angle, end_angle):
    step = (end_angle - start_angle) / (POINTS_PER_PACK - 1)
    return [start_angle + step * i for i in range(POINTS_PER_PACK)]

def polar_to_cartesian(distance_mm, angle_deg):
    r = distance_mm / 1000.0  # mm to meters
    theta = np.deg2rad(angle_deg)
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    return x, y

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
            xs, ys = [], []
            for (dist, _), angle in zip(points, angles):
                if dist > 0 and dist <= 1000:
                    x, y = polar_to_cartesian(dist, angle)
                    xs.append(x)
                    ys.append(y)

            ax.clear()
            ax.set_xlim(-1.2, 1.2)
            ax.set_ylim(-1.2, 1.2)
            ax.set_aspect("equal")
            ax.grid(True)
            ax.set_title("LD19 LiDAR - Points within 1m")
            ax.scatter(xs, ys, s=5, c='blue')
            plt.pause(0.01)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Stopped.")
