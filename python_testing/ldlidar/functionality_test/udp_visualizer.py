#!/usr/bin/env python3
"""
LD19 LIDAR Real-time UDP Visualizer
Provides a compact, real-time view of LIDAR data received via UDP.
"""

import socket
import struct
import sys
import os
from datetime import datetime
import time

# Configuration
UDP_IP = "0.0.0.0"
UDP_PORT = 5005
LD19_FRAME_SIZE = 47

def clear_screen():
    """Clear the terminal screen"""
    os.system('cls' if os.name == 'nt' else 'clear')

def create_polar_display(points_data, start_angle, end_angle):
    """Create a simple ASCII polar display"""
    display_lines = []
    
    # Header
    display_lines.append(f"Polar View: {start_angle:.1f}° to {end_angle:.1f}°")
    display_lines.append("═" * 60)
    
    # Create a 360-degree view with distance indicators
    angle_range = end_angle - start_angle
    if angle_range < 0:
        angle_range += 360
    
    for i, (distance_mm, intensity) in enumerate(points_data):
        if i == 0:
            point_angle = start_angle
        elif i == 11:
            point_angle = end_angle
        else:
            point_angle = start_angle + (angle_range * i / 11)
        
        # Normalize angle to 0-360
        point_angle = point_angle % 360
        
        if distance_mm > 0:
            distance_m = distance_mm / 1000.0
            # Create visual representation
            if distance_m < 0.5:
                symbol = "██"  # Very close
                color = "🔴"
            elif distance_m < 1.0:
                symbol = "▓▓"  # Close
                color = "🟡"
            elif distance_m < 2.0:
                symbol = "▒▒"  # Medium
                color = "🟢"
            else:
                symbol = "░░"  # Far
                color = "🔵"
            
            display_lines.append(f"{color} {point_angle:6.1f}° │{symbol}│ {distance_m:5.2f}m (I:{intensity:3d})")
        else:
            display_lines.append(f"⚫ {point_angle:6.1f}° │  │ No echo")
    
    return display_lines

def parse_and_display_frame(data, frame_number):
    """Parse frame and create real-time display"""
    clear_screen()
    
    timestamp = datetime.now().strftime("%H:%M:%S")
    
    print(f"🎯 LD19 LIDAR Real-time Display")
    print(f"Frame #{frame_number} at {timestamp}")
    print("═" * 70)
    
    if len(data) != LD19_FRAME_SIZE:
        print(f"❌ Invalid frame size: {len(data)} (expected {LD19_FRAME_SIZE})")
        return
    
    # Validate header
    if data[0] != 0x54 or data[1] != 0x2C:
        print(f"❌ Invalid frame header: 0x{data[0]:02X} 0x{data[1]:02X}")
        return
    
    try:
        # Parse basic frame info
        speed_dps = struct.unpack('<H', data[2:4])[0]
        start_angle = struct.unpack('<H', data[4:6])[0] / 100.0
        end_angle = struct.unpack('<H', data[42:44])[0] / 100.0
        timestamp_ms = struct.unpack('<H', data[44:46])[0]
        
        # Parse points
        points_data = []
        valid_points = 0
        min_dist = float('inf')
        max_dist = 0
        
        for i in range(12):
            offset = 6 + (i * 3)
            distance_mm = struct.unpack('<H', data[offset:offset+2])[0]
            intensity = data[offset+2]
            points_data.append((distance_mm, intensity))
            
            if distance_mm > 0:
                valid_points += 1
                min_dist = min(min_dist, distance_mm)
                max_dist = max(max_dist, distance_mm)
        
        # Display frame info
        print(f"🔄 Speed: {speed_dps:4d} dps │ Range: {start_angle:6.1f}° - {end_angle:6.1f}°")
        print(f"📊 Valid points: {valid_points}/12 │ Timestamp: {timestamp_ms} ms")
        
        if valid_points > 0:
            print(f"📏 Distance: {min_dist:4d}mm - {max_dist:4d}mm │ Closest: {min_dist/10:.1f}cm")
        else:
            print("📏 No valid measurements")
        
        print()
        
        # Create and display polar view
        polar_display = create_polar_display(points_data, start_angle, end_angle)
        for line in polar_display:
            print(line)
        
        # Alert for very close objects
        close_objects = [i for i, (dist, _) in enumerate(points_data) if 0 < dist < 300]
        if close_objects:
            print("\n⚠️  ALERT: Very close objects detected!")
            for i in close_objects:
                dist, intensity = points_data[i]
                angle = start_angle + ((end_angle - start_angle) * i / 11) if i > 0 and i < 11 else (start_angle if i == 0 else end_angle)
                print(f"   Point {i}: {dist}mm at {angle:.1f}°")
        
    except Exception as e:
        print(f"❌ Error parsing frame: {e}")

def main():
    print("Starting LD19 LIDAR Real-time Visualizer...")
    print(f"Listening on {UDP_IP}:{UDP_PORT}")
    print("Press Ctrl+C to stop")
    time.sleep(2)
    
    # Create socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
    sock.bind((UDP_IP, UDP_PORT))
    
    frame_count = 0
    last_update = time.time()
    
    try:
        while True:
            sock.settimeout(1.0)  # 1 second timeout
            
            try:
                data, addr = sock.recvfrom(2048)
                frame_count += 1
                
                # Update display (limit to reasonable refresh rate)
                current_time = time.time()
                if current_time - last_update > 0.1:  # Update max 10 times per second
                    parse_and_display_frame(data, frame_count)
                    last_update = current_time
                    
            except socket.timeout:
                # Show timeout message
                clear_screen()
                print("🎯 LD19 LIDAR Real-time Display")
                print(f"Waiting for data... (Frame #{frame_count})")
                print("═" * 70)
                print("⏳ No data received in the last second")
                print("   Check ESP32 connection and WiFi")
                print(f"   Listening on {UDP_IP}:{UDP_PORT}")
                continue
                
    except KeyboardInterrupt:
        clear_screen()
        print("🛑 Stopping LIDAR visualizer...")
        print(f"📊 Total frames processed: {frame_count}")
    finally:
        sock.close()

if __name__ == "__main__":
    try:
        main()
    except PermissionError:
        print("❌ Permission denied. Try running with administrator privileges.")
        sys.exit(1)
    except Exception as e:
        print(f"❌ Error: {e}")
        sys.exit(1)
