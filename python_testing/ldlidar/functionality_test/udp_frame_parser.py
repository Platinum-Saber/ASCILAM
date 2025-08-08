#!/usr/bin/env python3
"""
LD19 LIDAR UDP Frame Parser
Receives UDP packets from ESP32 and parses them according to LD19 frame format.

Frame Structure (47 bytes):
- Byte 0: Header (0x54)
- Byte 1: VerLen (0x2C)
- Bytes 2-3: Speed (degrees per second)
- Bytes 4-5: Start angle (hundredths of degree)
- Bytes 6-41: 12 measurement points (3 bytes each: 2 bytes distance + 1 byte intensity)
- Bytes 42-43: End angle (hundredths of degree)
- Bytes 44-45: Timestamp (milliseconds)
- Byte 46: CRC8 checksum
"""

import socket
import struct
import sys
from datetime import datetime
import math

# Configuration
UDP_IP = "0.0.0.0"  # Listen on all interfaces
UDP_PORT = 5005     # Must match ESP32's UDP_PORT

# LD19 Frame Constants
LD19_HEADER = 0x54
LD19_VERLEN = 0x2C
LD19_FRAME_SIZE = 47
LD19_POINTS_PER_FRAME = 12

class LD19Point:
    """Represents a single measurement point from the LIDAR"""
    def __init__(self, distance_mm, intensity):
        self.distance_mm = distance_mm
        self.intensity = intensity
        
    def __str__(self):
        return f"{self.distance_mm:4d}mm (I:{self.intensity:3d})"

class LD19Frame:
    """Represents a complete LD19 LIDAR frame"""
    def __init__(self):
        self.header = 0
        self.verlen = 0
        self.speed_dps = 0
        self.start_angle = 0  # hundredths of degree
        self.end_angle = 0    # hundredths of degree
        self.timestamp_ms = 0
        self.crc8 = 0
        self.points = []
        self.valid = False
        self.receive_time = datetime.now()  # Initialize with current time
        
    def start_angle_deg(self):
        """Get start angle in degrees"""
        return self.start_angle / 100.0
        
    def end_angle_deg(self):
        """Get end angle in degrees"""
        return self.end_angle / 100.0
        
    def angle_range_deg(self):
        """Get the angular range covered by this frame"""
        start = self.start_angle_deg()
        end = self.end_angle_deg()
        if end < start:
            end += 360.0  # Handle wraparound
        return end - start
        
    def point_angle_deg(self, point_index):
        """Calculate the angle for a specific point"""
        if not (0 <= point_index < len(self.points)):
            return None
            
        if len(self.points) <= 1:
            return self.start_angle_deg()
            
        # Linear interpolation between start and end angles
        angle_step = self.angle_range_deg() / (len(self.points) - 1)
        angle = self.start_angle_deg() + (point_index * angle_step)
        return angle % 360.0
    
    def __str__(self):
        status = "VALID" if self.valid else "INVALID"
        return (f"LD19Frame[{status}]: Speed={self.speed_dps}dps, "
                f"Angles={self.start_angle_deg():.2f}°-{self.end_angle_deg():.2f}°, "
                f"Points={len(self.points)}, TS={self.timestamp_ms}ms")

class LD19Parser:
    """Parser for LD19 LIDAR UDP frames"""
    
    def __init__(self):
        self.frames_received = 0
        self.frames_valid = 0
        self.frames_invalid = 0
        
    def parse_frame(self, data):
        """Parse raw UDP data into LD19Frame object"""
        frame = LD19Frame()
        frame.receive_time = datetime.now()
        
        # Check frame size
        if len(data) != LD19_FRAME_SIZE:
            print(f"Warning: Frame size {len(data)} != expected {LD19_FRAME_SIZE}")
            frame.valid = False
            self.frames_invalid += 1
            return frame
            
        # Parse header
        frame.header = data[0]
        frame.verlen = data[1]
        
        # Validate header and verlen
        if frame.header != LD19_HEADER:
            print(f"Warning: Invalid header 0x{frame.header:02X} != 0x{LD19_HEADER:02X}")
            frame.valid = False
            self.frames_invalid += 1
            return frame
            
        if frame.verlen != LD19_VERLEN:
            print(f"Warning: Invalid verlen 0x{frame.verlen:02X} != 0x{LD19_VERLEN:02X}")
            frame.valid = False
            self.frames_invalid += 1
            return frame
        
        # Parse frame data using little-endian format
        try:
            # Speed (bytes 2-3)
            frame.speed_dps = struct.unpack('<H', data[2:4])[0]
            
            # Start angle (bytes 4-5)
            frame.start_angle = struct.unpack('<H', data[4:6])[0]
            
            # Points (bytes 6-41, 12 points * 3 bytes each)
            frame.points = []
            for i in range(LD19_POINTS_PER_FRAME):
                offset = 6 + (i * 3)
                distance = struct.unpack('<H', data[offset:offset+2])[0]
                intensity = data[offset+2]
                frame.points.append(LD19Point(distance, intensity))
            
            # End angle (bytes 42-43)
            frame.end_angle = struct.unpack('<H', data[42:44])[0]
            
            # Timestamp (bytes 44-45)
            frame.timestamp_ms = struct.unpack('<H', data[44:46])[0]
            
            # CRC8 (byte 46)
            frame.crc8 = data[46]
            
            frame.valid = True
            self.frames_valid += 1
            
        except (struct.error, IndexError) as e:
            print(f"Error parsing frame: {e}")
            frame.valid = False
            self.frames_invalid += 1
            
        self.frames_received += 1
        return frame
    
    def print_statistics(self):
        """Print parsing statistics"""
        print(f"\n=== Parser Statistics ===")
        print(f"Total frames received: {self.frames_received}")
        print(f"Valid frames: {self.frames_valid}")
        print(f"Invalid frames: {self.frames_invalid}")
        if self.frames_received > 0:
            validity_rate = (self.frames_valid / self.frames_received) * 100
            print(f"Validity rate: {validity_rate:.1f}%")
        print(f"========================")

def print_frame_summary(frame):
    """Print a summary of the frame"""
    ts = frame.receive_time.strftime("%H:%M:%S.%f")[:-3]
    print(f"\n[{ts}] {frame}")
    
    if not frame.valid:
        return
        
    print(f"  Angular range: {frame.angle_range_deg():.2f}° "
          f"({frame.start_angle_deg():.2f}° to {frame.end_angle_deg():.2f}°)")
    print(f"  CRC8: 0x{frame.crc8:02X}")

def print_frame_detailed(frame):
    """Print detailed frame information including all points"""
    print_frame_summary(frame)
    
    if not frame.valid:
        return
        
    print("  Points:")
    for i, point in enumerate(frame.points):
        angle = frame.point_angle_deg(i)
        print(f"    Point {i:2d}: {point} at {angle:6.2f}°")

def print_frame_compact(frame):
    """Print compact frame information"""
    ts = frame.receive_time.strftime("%H:%M:%S.%f")[:-3]
    status = "✓" if frame.valid else "✗"
    
    if frame.valid:
        # Find closest and farthest points
        distances = [p.distance_mm for p in frame.points if p.distance_mm > 0]
        if distances:
            min_dist = min(distances)
            max_dist = max(distances)
            avg_dist = sum(distances) / len(distances)
            print(f"[{ts}] {status} Speed:{frame.speed_dps:4d}dps "
                  f"Range:{frame.start_angle_deg():6.2f}°-{frame.end_angle_deg():6.2f}° "
                  f"Dist: min={min_dist:4d}mm avg={avg_dist:4.0f}mm max={max_dist:4d}mm")
        else:
            print(f"[{ts}] {status} Speed:{frame.speed_dps:4d}dps "
                  f"Range:{frame.start_angle_deg():6.2f}°-{frame.end_angle_deg():6.2f}° "
                  f"(No valid distances)")
    else:
        print(f"[{ts}] {status} Invalid frame")

def print_frame_polar_visualization(frame):
    """Print a simple ASCII polar visualization"""
    if not frame.valid:
        return
        
    print(f"\n  Polar view (distances in meters):")
    print(f"  Angle range: {frame.start_angle_deg():.1f}° - {frame.end_angle_deg():.1f}°")
    
    # Create a simple ASCII representation
    valid_points = [(i, p) for i, p in enumerate(frame.points) if p.distance_mm > 0]
    
    if not valid_points:
        print("    (No valid measurements)")
        return
        
    for i, point in valid_points:
        angle = frame.point_angle_deg(i)
        distance_m = point.distance_mm / 1000.0
        intensity_bar = "█" * min(10, point.intensity // 10)
        print(f"    {angle:6.1f}° : {distance_m:5.2f}m [{intensity_bar:10s}] I={point.intensity}")

def main():
    print("LD19 LIDAR UDP Frame Parser")
    print("=" * 40)
    print(f"Listening on {UDP_IP}:{UDP_PORT}")
    print("Output modes:")
    print("  's' - Summary mode (default)")
    print("  'd' - Detailed mode (all points)")
    print("  'c' - Compact mode")
    print("  'p' - Polar visualization mode")
    print("  'q' - Quit")
    print("=" * 40)
    
    # Create socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
    sock.bind((UDP_IP, UDP_PORT))
    
    # Create parser
    parser = LD19Parser()
    
    # Display mode
    display_mode = 's'  # 's'=summary, 'd'=detailed, 'c'=compact, 'p'=polar
    
    try:
        while True:
            # Set a timeout so we can check for keyboard input
            sock.settimeout(1.0)
            
            try:
                data, addr = sock.recvfrom(2048)
                
                # Parse the frame
                frame = parser.parse_frame(data)
                
                # Display based on mode
                if display_mode == 's':
                    print_frame_summary(frame)
                elif display_mode == 'd':
                    print_frame_detailed(frame)
                elif display_mode == 'c':
                    print_frame_compact(frame)
                elif display_mode == 'p':
                    print_frame_summary(frame)
                    if frame.valid:
                        print_frame_polar_visualization(frame)
                        
            except socket.timeout:
                # Check if user wants to change mode or quit
                if sys.stdin.readable():
                    try:
                        user_input = input().strip().lower()
                        if user_input == 'q':
                            break
                        elif user_input in ['s', 'd', 'c', 'p']:
                            display_mode = user_input
                            mode_names = {'s': 'Summary', 'd': 'Detailed', 'c': 'Compact', 'p': 'Polar'}
                            print(f"Switched to {mode_names[display_mode]} mode")
                    except:
                        pass
                continue
                
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        parser.print_statistics()
        sock.close()

if __name__ == "__main__":
    try:
        main()
    except PermissionError:
        print("Permission error: try a different port (>1024) or run with proper rights.", 
              file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)
