#!/usr/bin/env python3
"""
LD19 LIDAR ASCII 2D Map Visualizer
Creates a text-based 2D map from UDP LIDAR data - no external dependencies required.
"""

import socket
import struct
import sys
import os
import time
import math
from datetime import datetime

# Configuration
UDP_IP = "0.0.0.0"
UDP_PORT = 5005
LD19_FRAME_SIZE = 47

# Map parameters
MAP_SIZE_M = 8  # Map size in meters (8x8 meters)
MAP_WIDTH = 80  # ASCII map width in characters
MAP_HEIGHT = 40  # ASCII map height in characters
MAP_RESOLUTION = MAP_SIZE_M / MAP_WIDTH  # meters per character
MAP_CENTER_X = MAP_WIDTH // 2
MAP_CENTER_Y = MAP_HEIGHT // 2

class ASCIILidarMapper:
    def __init__(self):
        self.map_grid = [[' ' for _ in range(MAP_WIDTH)] for _ in range(MAP_HEIGHT)]
        self.occupancy_count = [[0 for _ in range(MAP_WIDTH)] for _ in range(MAP_HEIGHT)]
        self.frame_count = 0
        self.total_points = 0
        self.valid_points = 0
        
    def clear_screen(self):
        """Clear the terminal screen"""
        os.system('cls' if os.name == 'nt' else 'clear')
    
    def polar_to_cartesian(self, distance_m, angle_deg):
        """Convert polar coordinates to Cartesian coordinates"""
        angle_rad = math.radians(angle_deg)
        x = distance_m * math.cos(angle_rad)
        y = distance_m * math.sin(angle_rad)
        return x, y
    
    def cartesian_to_map_coords(self, x, y):
        """Convert Cartesian coordinates to map grid coordinates"""
        map_x = int(MAP_CENTER_X + x / MAP_RESOLUTION)
        map_y = int(MAP_CENTER_Y - y / MAP_RESOLUTION)  # Flip Y axis
        return map_x, map_y
    
    def get_distance_symbol(self, distance_m, intensity):
        """Get ASCII symbol based on distance and intensity"""
        if distance_m < 0.5:
            return '█'  # Very close - solid block
        elif distance_m < 1.0:
            return '▓'  # Close - dark shade
        elif distance_m < 2.0:
            return '▒'  # Medium - medium shade
        elif distance_m < 4.0:
            return '░'  # Far - light shade
        else:
            return '·'  # Very far - dot
    
    def add_point_to_map(self, x, y, distance_m, intensity):
        """Add a point to the ASCII map"""
        map_x, map_y = self.cartesian_to_map_coords(x, y)
        
        # Check bounds
        if 0 <= map_x < MAP_WIDTH and 0 <= map_y < MAP_HEIGHT:
            self.occupancy_count[map_y][map_x] += 1
            symbol = self.get_distance_symbol(distance_m, intensity)
            
            # Update symbol based on occupancy count (more hits = darker)
            count = self.occupancy_count[map_y][map_x]
            if count > 10:
                self.map_grid[map_y][map_x] = '█'
            elif count > 5:
                self.map_grid[map_y][map_x] = '▓'
            elif count > 2:
                self.map_grid[map_y][map_x] = '▒'
            else:
                self.map_grid[map_y][map_x] = symbol
            
            return True
        return False
    
    def process_frame(self, data):
        """Process a LIDAR frame and extract points"""
        if len(data) != LD19_FRAME_SIZE:
            return None
        
        # Validate header
        if data[0] != 0x54 or data[1] != 0x2C:
            return None
        
        try:
            # Parse frame data
            speed_dps = struct.unpack('<H', data[2:4])[0]
            start_angle = struct.unpack('<H', data[4:6])[0] / 100.0
            end_angle = struct.unpack('<H', data[42:44])[0] / 100.0
            timestamp_ms = struct.unpack('<H', data[44:46])[0]
            
            # Calculate angle range
            angle_range = end_angle - start_angle
            if angle_range < 0:
                angle_range += 360.0
            
            self.frame_count += 1
            frame_points = []
            
            # Parse points
            for i in range(12):
                offset = 6 + (i * 3)
                distance_mm = struct.unpack('<H', data[offset:offset+2])[0]
                intensity = data[offset+2]
                
                if distance_mm > 0 and distance_mm < 8000:  # Valid range: 0-8m
                    self.total_points += 1
                    
                    # Calculate point angle
                    if i == 0:
                        point_angle = start_angle
                    elif i == 11:
                        point_angle = end_angle
                    else:
                        point_angle = start_angle + (angle_range * i / 11)
                    
                    # Convert to Cartesian coordinates
                    distance_m = distance_mm / 1000.0
                    x, y = self.polar_to_cartesian(distance_m, point_angle)
                    
                    # Add to map
                    if self.add_point_to_map(x, y, distance_m, intensity):
                        self.valid_points += 1
                        frame_points.append({
                            'x': x, 'y': y, 'distance': distance_m,
                            'angle': point_angle, 'intensity': intensity
                        })
            
            return {
                'speed': speed_dps,
                'start_angle': start_angle,
                'end_angle': end_angle,
                'timestamp': timestamp_ms,
                'points': frame_points
            }
            
        except Exception as e:
            print(f"Error processing frame: {e}")
            return None
    
    def draw_map(self, frame_info=None):
        """Draw the ASCII map"""
        self.clear_screen()
        
        print("🎯 LD19 LIDAR ASCII Map Visualizer")
        print("=" * MAP_WIDTH)
        
        if frame_info:
            print(f"Frame #{self.frame_count} | Speed: {frame_info['speed']} dps | "
                  f"Range: {frame_info['start_angle']:.1f}°-{frame_info['end_angle']:.1f}° | "
                  f"Points: {len(frame_info['points'])}")
        else:
            print(f"Frame #{self.frame_count} | Waiting for data...")
        
        print(f"Valid points: {self.valid_points}/{self.total_points} | "
              f"Map: {MAP_SIZE_M}x{MAP_SIZE_M}m @ {MAP_RESOLUTION*100:.1f}cm/char")
        print("=" * MAP_WIDTH)
        
        # Draw the map
        for y in range(MAP_HEIGHT):
            line = ""
            for x in range(MAP_WIDTH):
                # Mark LIDAR position
                if x == MAP_CENTER_X and y == MAP_CENTER_Y:
                    line += 'L'  # LIDAR position
                else:
                    line += self.map_grid[y][x]
            print(line)
        
        # Add scale and legend
        print("=" * MAP_WIDTH)
        print("Legend: L=LIDAR █=Very close ▓=Close ▒=Medium ░=Far ·=Very far")
        print(f"Scale: {MAP_SIZE_M}m x {MAP_SIZE_M}m | Center = LIDAR position")
        print("Controls: Ctrl+C to exit, 'c' to clear map (if supported)")
        
        # Show current frame details
        if frame_info and frame_info['points']:
            print(f"\nCurrent scan details:")
            close_objects = [p for p in frame_info['points'] if p['distance'] < 1.0]
            if close_objects:
                print(f"⚠️  {len(close_objects)} objects within 1m:")
                for i, obj in enumerate(close_objects[:5]):  # Show max 5
                    print(f"  {i+1}. {obj['distance']*100:.0f}cm at {obj['angle']:.1f}° "
                          f"(I:{obj['intensity']})")
                if len(close_objects) > 5:
                    print(f"  ... and {len(close_objects)-5} more")
    
    def clear_map(self):
        """Clear the accumulated map"""
        self.map_grid = [[' ' for _ in range(MAP_WIDTH)] for _ in range(MAP_HEIGHT)]
        self.occupancy_count = [[0 for _ in range(MAP_WIDTH)] for _ in range(MAP_HEIGHT)]
        self.total_points = 0
        self.valid_points = 0
        print("Map cleared!")
    
    def save_map(self):
        """Save map to text file"""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f"lidar_ascii_map_{timestamp}.txt"
        
        try:
            with open(filename, 'w', encoding='utf-8') as f:
                f.write(f"LD19 LIDAR ASCII Map - {datetime.now()}\n")
                f.write(f"Map size: {MAP_SIZE_M}x{MAP_SIZE_M}m\n")
                f.write(f"Resolution: {MAP_RESOLUTION*100:.1f}cm per character\n")
                f.write(f"Total points: {self.valid_points}/{self.total_points}\n")
                f.write("=" * MAP_WIDTH + "\n")
                
                for y in range(MAP_HEIGHT):
                    line = ""
                    for x in range(MAP_WIDTH):
                        if x == MAP_CENTER_X and y == MAP_CENTER_Y:
                            line += 'L'
                        else:
                            line += self.map_grid[y][x]
                    f.write(line + "\n")
                
                f.write("=" * MAP_WIDTH + "\n")
                f.write("Legend: L=LIDAR █=Very close ▓=Close ▒=Medium ░=Far ·=Very far\n")
            
            print(f"Map saved to {filename}")
        except Exception as e:
            print(f"Error saving map: {e}")

def main():
    print("LD19 LIDAR ASCII Map Visualizer")
    print("=" * 50)
    print(f"Map size: {MAP_SIZE_M}x{MAP_SIZE_M} meters")
    print(f"Resolution: {MAP_RESOLUTION*100:.1f} cm per character")
    print(f"Listening on {UDP_IP}:{UDP_PORT}")
    print("Press Ctrl+C to stop")
    print("=" * 50)
    
    # Create mapper
    mapper = ASCIILidarMapper()
    
    # Create socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
    sock.bind((UDP_IP, UDP_PORT))
    sock.settimeout(2.0)  # 2 second timeout
    
    last_update = time.time()
    
    try:
        while True:
            try:
                data, addr = sock.recvfrom(2048)
                frame_info = mapper.process_frame(data)
                
                # Update display every 0.2 seconds (5 FPS)
                current_time = time.time()
                if current_time - last_update > 0.2:
                    mapper.draw_map(frame_info)
                    last_update = current_time
                
            except socket.timeout:
                # Show timeout status
                mapper.draw_map()
                print("\n⏳ No data received in the last 2 seconds")
                print("   Check ESP32 connection and WiFi settings")
                continue
                
    except KeyboardInterrupt:
        mapper.clear_screen()
        print("\n🛑 Stopping LIDAR mapper...")
        print(f"📊 Session Summary:")
        print(f"   Total frames: {mapper.frame_count}")
        print(f"   Valid points: {mapper.valid_points}/{mapper.total_points}")
        if mapper.total_points > 0:
            validity_rate = mapper.valid_points / mapper.total_points * 100
            print(f"   Validity rate: {validity_rate:.1f}%")
        
        # Offer to save map
        try:
            if mapper.valid_points > 0:
                save_choice = input("\nSave ASCII map to file? (y/n): ").strip().lower()
                if save_choice == 'y':
                    mapper.save_map()
        except:
            pass
        
    finally:
        sock.close()

if __name__ == "__main__":
    try:
        main()
    except PermissionError:
        print("❌ Permission denied. Try running with administrator privileges or use a port > 1024.")
        sys.exit(1)
    except Exception as e:
        print(f"❌ Error: {e}")
        sys.exit(1)
