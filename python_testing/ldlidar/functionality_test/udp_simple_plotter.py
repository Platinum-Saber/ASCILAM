#!/usr/bin/env python3
"""
LD19 LIDAR Simple 2D Plotter
Creates basic 2D plots from UDP LIDAR data with optional matplotlib support.
Falls back to text-based visualization if matplotlib is not available.
"""

import socket
import struct
import sys
import math
import time
from datetime import datetime

# Try to import matplotlib, fall back gracefully
try:
    import matplotlib.pyplot as plt
    import numpy as np
    MATPLOTLIB_AVAILABLE = True
    print("📊 Matplotlib found - will use graphical plotting")
except ImportError:
    MATPLOTLIB_AVAILABLE = False
    print("⚠️ Matplotlib not found - using text-based plotting")
    print("Install with: pip install matplotlib numpy")

# Configuration
UDP_IP = "0.0.0.0"
UDP_PORT = 5005
LD19_FRAME_SIZE = 47

class SimpleLidarPlotter:
    def __init__(self, use_matplotlib=True):
        self.use_matplotlib = use_matplotlib and MATPLOTLIB_AVAILABLE
        self.points_history = []
        self.max_history = 100  # Keep last 100 frames
        self.frame_count = 0
        
        if self.use_matplotlib:
            self.setup_matplotlib()
    
    def setup_matplotlib(self):
        """Set up matplotlib plotting"""
        plt.ion()  # Interactive mode
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.ax.set_xlim(-6, 6)
        self.ax.set_ylim(-6, 6)
        self.ax.set_xlabel('X (meters)')
        self.ax.set_ylabel('Y (meters)')
        self.ax.set_title('LD19 LIDAR 2D Plot')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_aspect('equal')
        
        # Add range circles
        for radius in [1, 2, 3, 4, 5]:
            circle = plt.Circle((0, 0), radius, fill=False, color='gray', alpha=0.3, linestyle='--')
            self.ax.add_patch(circle)
        
        # LIDAR position
        self.ax.plot(0, 0, 'ro', markersize=10, label='LIDAR')
        self.ax.legend()
        
        plt.show()
    
    def polar_to_cartesian(self, distance_m, angle_deg):
        """Convert polar coordinates to Cartesian coordinates"""
        angle_rad = math.radians(angle_deg)
        x = distance_m * math.cos(angle_rad)
        y = distance_m * math.sin(angle_rad)
        return x, y
    
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
            points = []
            
            # Parse points
            for i in range(12):
                offset = 6 + (i * 3)
                distance_mm = struct.unpack('<H', data[offset:offset+2])[0]
                intensity = data[offset+2]
                
                if distance_mm > 0 and distance_mm < 6000:  # Valid range: 0-6m
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
                    
                    points.append({
                        'x': x, 'y': y, 'distance': distance_m,
                        'angle': point_angle, 'intensity': intensity
                    })
            
            # Add to history
            frame_data = {
                'frame': self.frame_count,
                'timestamp': datetime.now(),
                'speed': speed_dps,
                'start_angle': start_angle,
                'end_angle': end_angle,
                'points': points
            }
            
            self.points_history.append(frame_data)
            if len(self.points_history) > self.max_history:
                self.points_history.pop(0)
            
            return frame_data
            
        except Exception as e:
            print(f"Error processing frame: {e}")
            return None
    
    def plot_matplotlib(self, frame_data):
        """Plot using matplotlib"""
        if not self.use_matplotlib or not frame_data:
            return
        
        # Clear previous plot
        self.ax.clear()
        self.ax.set_xlim(-6, 6)
        self.ax.set_ylim(-6, 6)
        self.ax.set_xlabel('X (meters)')
        self.ax.set_ylabel('Y (meters)')
        self.ax.set_title(f'LD19 LIDAR 2D Plot - Frame #{frame_data["frame"]}')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_aspect('equal')
        
        # Add range circles
        for radius in [1, 2, 3, 4, 5]:
            circle = plt.Circle((0, 0), radius, fill=False, color='gray', alpha=0.3, linestyle='--')
            self.ax.add_patch(circle)
        
        # Plot LIDAR position
        self.ax.plot(0, 0, 'ro', markersize=10, label='LIDAR')
        
        # Plot current frame points
        if frame_data['points']:
            x_coords = [p['x'] for p in frame_data['points']]
            y_coords = [p['y'] for p in frame_data['points']]
            distances = [p['distance'] for p in frame_data['points']]
            intensities = [p['intensity'] for p in frame_data['points']]
            
            # Color by distance, size by intensity
            sizes = [max(20, i/5) for i in intensities]
            scatter = self.ax.scatter(x_coords, y_coords, c=distances, s=sizes, 
                                    cmap='viridis', alpha=0.8, edgecolors='black', linewidth=0.5)
            
            plt.colorbar(scatter, ax=self.ax, label='Distance (m)')
        
        # Plot history (faded)
        if len(self.points_history) > 1:
            for i, old_frame in enumerate(self.points_history[:-1]):  # Exclude current frame
                if old_frame['points']:
                    alpha = 0.1 + 0.3 * (i / len(self.points_history))  # Fade older points
                    x_coords = [p['x'] for p in old_frame['points']]
                    y_coords = [p['y'] for p in old_frame['points']]
                    self.ax.scatter(x_coords, y_coords, c='lightgray', s=10, alpha=alpha)
        
        # Add info text
        info_text = (f"Speed: {frame_data['speed']} dps\n"
                    f"Range: {frame_data['start_angle']:.1f}° - {frame_data['end_angle']:.1f}°\n"
                    f"Points: {len(frame_data['points'])}")
        self.ax.text(0.02, 0.98, info_text, transform=self.ax.transAxes, 
                    verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        plt.draw()
        plt.pause(0.01)
    
    def plot_text(self, frame_data):
        """Plot using text output"""
        import os
        os.system('cls' if os.name == 'nt' else 'clear')
        
        print("🎯 LD19 LIDAR Simple 2D Plot (Text Mode)")
        print("=" * 60)
        
        if frame_data:
            print(f"Frame #{frame_data['frame']} | Speed: {frame_data['speed']} dps")
            print(f"Range: {frame_data['start_angle']:.1f}° - {frame_data['end_angle']:.1f}°")
            print(f"Points: {len(frame_data['points'])}")
            print(f"Time: {frame_data['timestamp'].strftime('%H:%M:%S')}")
        else:
            print("Waiting for data...")
        
        print("=" * 60)
        
        if frame_data and frame_data['points']:
            print("Current Points (Polar Coordinates):")
            print(f"{'Point':<5} {'Angle':<8} {'Distance':<10} {'X':<8} {'Y':<8} {'Intensity'}")
            print("-" * 60)
            
            for i, point in enumerate(frame_data['points']):
                print(f"{i+1:<5} {point['angle']:7.1f}° {point['distance']*1000:7.0f}mm "
                      f"{point['x']:7.2f}m {point['y']:7.2f}m {point['intensity']:>9}")
            
            # Simple ASCII polar plot
            print("\nSimple Polar Plot (top-down view):")
            print("     -3m  -2m  -1m   0   +1m  +2m  +3m")
            
            for y_line in range(13):  # 13 lines for -3m to +3m
                y_pos = 3 - (y_line * 0.5)
                line = f"{y_pos:+4.1f} "
                
                for x_char in range(25):  # 25 chars for -3m to +3m
                    x_pos = -3 + (x_char * 0.25)
                    
                    # Check if any point is near this position
                    symbol = ' '
                    for point in frame_data['points']:
                        if (abs(point['x'] - x_pos) < 0.2 and 
                            abs(point['y'] - y_pos) < 0.2):
                            if point['distance'] < 1.0:
                                symbol = '●'  # Close
                            elif point['distance'] < 2.0:
                                symbol = '○'  # Medium
                            else:
                                symbol = '·'  # Far
                            break
                    
                    # Mark LIDAR center
                    if abs(x_pos) < 0.1 and abs(y_pos) < 0.1:
                        symbol = 'L'
                    
                    line += symbol
                
                print(line)
            
            print("Legend: L=LIDAR, ●=<1m, ○=<2m, ·=<3m")
        
        print("\nPress Ctrl+C to stop")

def main():
    print("LD19 LIDAR Simple 2D Plotter")
    print("=" * 50)
    print(f"Listening on {UDP_IP}:{UDP_PORT}")
    
    if MATPLOTLIB_AVAILABLE:
        use_matplotlib = True
        print("Using matplotlib for graphical plotting")
    else:
        use_matplotlib = False
        print("Using text-based plotting (install matplotlib for graphics)")
    
    print("=" * 50)
    
    # Create plotter
    plotter = SimpleLidarPlotter(use_matplotlib)
    
    # Create socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
    sock.bind((UDP_IP, UDP_PORT))
    sock.settimeout(1.0)
    
    last_update = time.time()
    
    try:
        while True:
            try:
                data, addr = sock.recvfrom(2048)
                frame_data = plotter.process_frame(data)
                
                # Update display
                current_time = time.time()
                if current_time - last_update > 0.1:  # 10 FPS max
                    if plotter.use_matplotlib:
                        plotter.plot_matplotlib(frame_data)
                    else:
                        plotter.plot_text(frame_data)
                    last_update = current_time
                
            except socket.timeout:
                # Show timeout status
                if not plotter.use_matplotlib:
                    plotter.plot_text(None)
                print("⏳ No data received - check ESP32 connection")
                continue
                
    except KeyboardInterrupt:
        print("\n🛑 Stopping plotter...")
        print(f"📊 Total frames processed: {plotter.frame_count}")
        
        # Save data if available
        if plotter.points_history and plotter.use_matplotlib:
            try:
                save_choice = input("Save plot as PNG? (y/n): ").strip().lower()
                if save_choice == 'y':
                    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                    filename = f"lidar_plot_{timestamp}.png"
                    plt.savefig(filename, dpi=300, bbox_inches='tight')
                    print(f"Plot saved as {filename}")
            except:
                pass
        
    finally:
        sock.close()
        if plotter.use_matplotlib:
            plt.close('all')

if __name__ == "__main__":
    try:
        main()
    except PermissionError:
        print("❌ Permission denied. Try running with administrator privileges or use a port > 1024.")
        sys.exit(1)
    except Exception as e:
        print(f"❌ Error: {e}")
        sys.exit(1)
