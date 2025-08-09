#!/usr/bin/env python3
# sim_ld19_udp.py
# Send synthetic LD19 LiDAR frames over UDP to test your coordinator/receiver.

import argparse
import math
import random
import socket
import struct
import time

HEADER = 0x54
VERLEN = 0x2C       # 12 points per packet
POINTS = 12
FRAME_LEN = 47      # bytes
TS_MAX = 30000

# CRC-8 table from LD19 manual
CRC_TABLE = [
    0x00,0x4D,0x9A,0xD7,0x79,0x34,0xE3,0xAE,0xF2,0xBF,0x68,0x25,0x8B,0xC6,0x11,0x5C,
    0xA9,0xE4,0x33,0x7E,0xD0,0x9D,0x4A,0x07,0x5B,0x16,0xC1,0x8C,0x22,0x6F,0xB8,0xF5,
    0x1F,0x52,0x85,0xC8,0x66,0x2B,0xFC,0xB1,0xED,0xA0,0x77,0x3A,0x94,0xD9,0x0E,0x43,
    0xB6,0xFB,0x2C,0x61,0xCF,0x82,0x55,0x18,0x44,0x09,0xDE,0x93,0x3D,0x70,0xA7,0xEA,
    0x3E,0x73,0xA4,0xE9,0x47,0x0A,0xDD,0x90,0xCC,0x81,0x56,0x1B,0xB5,0xF8,0x2F,0x62,
    0x97,0xDA,0x0D,0x40,0xEE,0xA3,0x74,0x39,0x65,0x28,0xFF,0xB2,0x1C,0x51,0x86,0xCB,
    0x21,0x6C,0xBB,0xF6,0x58,0x15,0xC2,0x8F,0xD3,0x9E,0x49,0x04,0xAA,0xE7,0x30,0x7D,
    0x88,0xC5,0x12,0x5F,0xF1,0xBC,0x6B,0x26,0x7A,0x37,0xE0,0xAD,0x03,0x4E,0x99,0xD4,
    0x7C,0x31,0xE6,0xAB,0x05,0x48,0x9F,0xD2,0x8E,0xC3,0x14,0x59,0xF7,0xBA,0x6D,0x20,
    0xD5,0x98,0x4F,0x02,0xAC,0xE1,0x36,0x7B,0x27,0x6A,0xBD,0xF0,0x5E,0x13,0xC4,0x89,
    0x63,0x2E,0xF9,0xB4,0x1A,0x57,0x80,0xCD,0x91,0xDC,0x0B,0x46,0xE8,0xA5,0x72,0x3F,
    0xCA,0x87,0x50,0x1D,0xB3,0xFE,0x29,0x64,0x38,0x75,0xA2,0xEF,0x41,0x0C,0xDB,0x96,
    0x42,0x0F,0xD8,0x95,0x3B,0x76,0xA1,0xEC,0xB0,0xFD,0x2A,0x67,0xC9,0x84,0x53,0x1E,
    0xEB,0xA6,0x71,0x3C,0x92,0xDF,0x08,0x45,0x19,0x54,0x83,0xCE,0x60,0x2D,0xFA,0xB7,
    0x5D,0x10,0xC7,0x8A,0x24,0x69,0xBE,0xF3,0xAF,0xE2,0x35,0x78,0xD6,0x9B,0x4C,0x01,
    0xF4,0xB9,0x6E,0x23,0x8D,0xC0,0x17,0x5A,0x06,0x4B,0x9C,0xD1,0x7F,0x32,0xE5,0xA8
]

def crc8(data: bytes) -> int:
    c = 0
    for b in data:
        c = CRC_TABLE[(c ^ b) & 0xFF]
    return c

def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v

def synthetic_range(angle_deg: float, base_m=3.0) -> float:
    """
    Very simple world:
      - base range ~3.0 m with small noise
      - a short obstacle at ~30° (±6°) at 1.2 m
      - a closer obstacle at ~200° (±8°) at 0.8 m
      - a medium obstacle at ~300° (±10°) at 1.5 m
    """
    a = (angle_deg % 360.0)
    r = base_m + 0.05 * math.sin(math.radians(a*3)) + random.uniform(-0.03, 0.03)

    def bump(center, width, val):
        nonlocal r
        # shortest angular distance
        d = abs((a - center + 180) % 360 - 180)
        if d <= width:
            # blend toward val
            w = (width - d) / width
            r = r * (1 - w) + val * w

    bump(30.0, 6.0, 1.2)
    bump(200.0, 8.0, 0.8)
    bump(300.0, 10.0, 1.5)

    return clamp(r, 0.1, 12.0)

def make_frame(start_deg: float, span_deg: float, dps: float, ts_ms: int):
    """
    Build one 47-byte LD19 packet with 12 points.
    Angles are in degrees (CW, left-handed as per LD19), so we encode start/end in 0.01°.
    """
    # Header + verlen + speed + start_angle
    buf = bytearray()
    buf += struct.pack('<B', HEADER)
    buf += struct.pack('<B', VERLEN)
    # speed in degrees per second (uint16 little-endian)
    buf += struct.pack('<H', int(dps) & 0xFFFF)
    # start angle in centi-degrees, 0..36000 (wrap handled with %360)
    start_cd = int(round((start_deg % 360.0) * 100.0)) & 0xFFFF
    buf += struct.pack('<H', start_cd)

    # Points
    if POINTS > 1:
        step = span_deg / (POINTS - 1)
    else:
        step = 0.0

    for i in range(POINTS):
        ang = (start_deg + i * step) % 360.0
        rng_m = synthetic_range(ang)
        dist_mm = int(round(rng_m * 1000.0))
        dist_mm = clamp(dist_mm, 0, 0xFFFF)
        # make intensity loosely inversely proportional to distance
        inten = int(clamp(255.0 * (1.8 / max(rng_m, 0.1)), 5, 255))
        buf += struct.pack('<HB', dist_mm, inten)

    # End angle
    end_deg = (start_deg + span_deg) % 360.0
    end_cd = int(round(end_deg * 100.0)) & 0xFFFF
    buf += struct.pack('<H', end_cd)

    # Timestamp (uint16, wraps at 30000)
    ts = ts_ms % TS_MAX
    buf += struct.pack('<H', ts)

    # CRC over all previous bytes
    c = crc8(bytes(buf))
    buf += struct.pack('<B', c)

    assert len(buf) == FRAME_LEN, f"frame len {len(buf)} != 47"
    return bytes(buf)

def main():
    ap = argparse.ArgumentParser(description="LD19 UDP simulator")
    ap.add_argument('--ip',   default='127.0.0.1', help='target IP (coordinator)')
    ap.add_argument('--port', type=int, default=6001, help='target UDP port')
    ap.add_argument('--hz',   type=float, default=10.0, help='spin rate (rev/s), LD19 ~10')
    ap.add_argument('--span', type=float, default=6.0, help='degrees per packet (typical small sector)')
    ap.add_argument('--jitter', type=float, default=0.5, help='random timing jitter in ms')
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    addr = (args.ip, args.port)

    dps = 360.0 * args.hz              # degrees per second field
    frames_per_rev = max(1, int(round(360.0 / args.span)))
    sleep_per_frame = 1.0 / (args.hz * frames_per_rev)

    print(f"[sim] Sending to {addr}, {args.hz} Hz, {frames_per_rev} frames/rev, span={args.span}°")
    start_deg = 0.0
    ts0 = time.time()

    try:
        while True:
            # timestamp in ms since start
            ts_ms = int((time.time() - ts0) * 1000.0)
            frame = make_frame(start_deg, args.span, dps, ts_ms)
            sock.sendto(frame, addr)

            # advance start angle by span (CW)
            start_deg = (start_deg + args.span) % 360.0

            # pacing with a touch of jitter
            jitter = random.uniform(-args.jitter, args.jitter) / 1000.0
            time.sleep(max(0.0, sleep_per_frame + jitter))
    except KeyboardInterrupt:
        print("\n[sim] stopped.")
    finally:
        sock.close()

if __name__ == '__main__':
    main()
