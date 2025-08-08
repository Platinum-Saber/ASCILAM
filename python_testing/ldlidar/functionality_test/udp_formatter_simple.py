#!/usr/bin/env python3
"""
Simple LD19 LIDAR UDP Data Formatter
Receives UDP packets from ESP32 and formats them according to LD19 frame structure.
This is a straightforward script that focuses on clear, formatted output.
"""

import socket
import struct
import sys
from datetime import datetime

# Configuration
UDP_IP = "0.0.0.0"  # Listen on all interfaces
UDP_PORT = 5005     # Must match ESP32's UDP_PORT

# LD19 Frame Constants
LD19_HEADER = 0x54
LD19_VERLEN = 0x2C
LD19_FRAME_SIZE = 47

def format_frame_data(data, addr, receive_time):
    """Format the received UDP data according to LD19 frame structure"""
    
    # Basic info
    timestamp = receive_time.strftime("%H:%M:%S.%f")[:-3]
    print(f"\n{'='*80}")
    print(f"Frame received at [{timestamp}] from {addr[0]}:{addr[1]}")
    print(f"Data length: {len(data)} bytes (expected: {LD19_FRAME_SIZE})")
    
    # Check frame size
    if len(data) != LD19_FRAME_SIZE:
        print(f"⚠️  WARNING: Frame size mismatch!")
        print(f"Raw data (first 32 bytes): {' '.join(f'{b:02X}' for b in data[:32])}")
        return
    
    # Parse header
    header = data[0]
    verlen = data[1]
    
    print(f"Header: 0x{header:02X} {'✓' if header == LD19_HEADER else '✗ (expected 0x54)'}")
    print(f"VerLen: 0x{verlen:02X} {'✓' if verlen == LD19_VERLEN else '✗ (expected 0x2C)'}")
    
    if header != LD19_HEADER or verlen != LD19_VERLEN:
        print("⚠️  Invalid frame header - showing raw data:")
        print(f"Raw: {' '.join(f'{b:02X}' for b in data)}")
        return
    
    # Parse frame data (little-endian format)
    try:
        # Speed (bytes 2-3)
        speed_dps = struct.unpack('<H', data[2:4])[0]
        
        # Start angle (bytes 4-5) 
        start_angle_raw = struct.unpack('<H', data[4:6])[0]
        start_angle_deg = start_angle_raw / 100.0
        
        # End angle (bytes 42-43)
        end_angle_raw = struct.unpack('<H', data[42:44])[0] 
        end_angle_deg = end_angle_raw / 100.0
        
        # Timestamp (bytes 44-45)
        timestamp_ms = struct.unpack('<H', data[44:46])[0]
        
        # CRC8 (byte 46)
        crc8 = data[46]
        
        # Frame info
        print(f"Speed: {speed_dps} degrees/second")
        print(f"Start Angle: {start_angle_deg:.2f}° (raw: {start_angle_raw})")
        print(f"End Angle: {end_angle_deg:.2f}° (raw: {end_angle_raw})")
        
        # Calculate angular range
        angle_range = end_angle_deg - start_angle_deg
        if angle_range < 0:
            angle_range += 360.0  # Handle wraparound
        print(f"Angular Range: {angle_range:.2f}°")
        
        print(f"Timestamp: {timestamp_ms} ms")
        print(f"CRC8: 0x{crc8:02X}")
        
        # Parse measurement points (bytes 6-41, 12 points * 3 bytes each)
        print(f"\nMeasurement Points (12 points):")
        print(f"{'Point':<5} {'Angle':<8} {'Distance':<10} {'Intensity':<9} {'Status'}")
        print(f"{'-'*5} {'-'*8} {'-'*10} {'-'*9} {'-'*10}")
        
        valid_points = 0
        min_distance = float('inf')
        max_distance = 0
        total_distance = 0
        
        for i in range(12):
            offset = 6 + (i * 3)
            distance_mm = struct.unpack('<H', data[offset:offset+2])[0]
            intensity = data[offset+2]
            
            # Calculate point angle (linear interpolation)
            if i == 0:
                point_angle = start_angle_deg
            elif i == 11:
                point_angle = end_angle_deg
            else:
                point_angle = start_angle_deg + (angle_range * i / 11)
            
            # Point status
            status = "Valid" if distance_mm > 0 else "No echo"
            
            print(f"{i:2d}    {point_angle:7.2f}° {distance_mm:6d} mm  {intensity:7d}     {status}")
            
            if distance_mm > 0:
                valid_points += 1
                min_distance = min(min_distance, distance_mm)
                max_distance = max(max_distance, distance_mm)
                total_distance += distance_mm
        
        # Statistics
        print(f"\nPoint Statistics:")
        print(f"Valid points: {valid_points}/12")
        if valid_points > 0:
            avg_distance = total_distance / valid_points
            print(f"Distance range: {min_distance} mm to {max_distance} mm")
            print(f"Average distance: {avg_distance:.1f} mm")
            print(f"Closest object: {min_distance} mm ({min_distance/10:.1f} cm)")
        else:
            print("No valid distance measurements in this frame")
            
        # Visual representation for close objects
        print(f"\nClose Objects (< 1 meter):")
        close_objects = []
        for i in range(12):
            offset = 6 + (i * 3)
            distance_mm = struct.unpack('<H', data[offset:offset+2])[0]
            if 0 < distance_mm < 1000:
                point_angle = start_angle_deg + (angle_range * i / 11) if i > 0 and i < 11 else (start_angle_deg if i == 0 else end_angle_deg)
                close_objects.append((i, point_angle, distance_mm))
        
        if close_objects:
            for point_idx, angle, distance in close_objects:
                bars = "█" * min(20, distance // 25)  # Visual bar proportional to distance
                print(f"  Point {point_idx:2d} at {angle:6.1f}°: {distance:3d}mm |{bars:<20}|")
        else:
            print("  No objects detected within 1 meter")
            
    except (struct.error, IndexError) as e:
        print(f"❌ Error parsing frame data: {e}")
        print(f"Raw data: {' '.join(f'{b:02X}' for b in data)}")

def main():
    print("LD19 LIDAR UDP Data Formatter")
    print("=" * 50)
    print(f"Listening on {UDP_IP}:{UDP_PORT}")
    print("Press Ctrl+C to stop")
    print("=" * 50)
    
    # Create and configure socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
    sock.bind((UDP_IP, UDP_PORT))
    
    frame_count = 0
    
    try:
        while True:
            # Receive UDP packet
            data, addr = sock.recvfrom(2048)
            frame_count += 1
            receive_time = datetime.now()
            
            print(f"\n🔄 Processing frame #{frame_count}")
            
            # Format and display the frame data
            format_frame_data(data, addr, receive_time)
            
            # Add a separator for readability
            print(f"{'='*80}")
            
    except KeyboardInterrupt:
        print(f"\n\n📊 Session Summary:")
        print(f"Total frames received: {frame_count}")
        print("Stopped by user.")
    except Exception as e:
        print(f"\n❌ Error: {e}")
    finally:
        sock.close()
        print("Socket closed.")

if __name__ == "__main__":
    try:
        main()
    except PermissionError:
        print("❌ Permission error: try a different port (>1024) or run with administrator rights.", 
              file=sys.stderr)
        sys.exit(1)
