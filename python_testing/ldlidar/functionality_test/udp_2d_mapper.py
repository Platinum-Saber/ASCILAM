#!/usr/bin/env python3
"""
LD19 LIDAR 2D Mapping Visualizer
Receives UDP packets from ESP32 and creates a real-time 2D map of the environment.
"""

import socket
import struct
import sys
import threading
import queue
import math
from datetime import datetime

try:
    import numpy as np
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    from matplotlib.patches import Circle
except ImportError as e:
    print("❌ Missing required packages. Install with:")
    print("pip install matplotlib numpy")
    print(f"Error: {e}")
    sys.exit(1)

# Configuration
UDP_IP = "0.0.0.0"
UDP_PORT = 5005
LD19_FRAME_SIZE = 47

# Mapping parameters
MAP_SIZE_M = 10  # Map size in meters (10x10 meters)
MAP_RESOLUTION = 0.05  # Resolution in meters per pixel (5cm)
MAP_PIXELS = int(MAP_SIZE_M / MAP_RESOLUTION)  # 200x200 pixels
MAP_CENTER = MAP_PIXELS // 2

class LidarMapper:
    def __init__(self):
        self.map_data = np.zeros((MAP_PIXELS, MAP_PIXELS), dtype=np.float32)
        self.current_points = []
        self.frame_count = 0
        self.data_queue = queue.Queue()
        self.running = True
        
        # Statistics
        self.total_points = 0
        self.valid_points = 0
        
    def polar_to_cartesian(self, distance_m, angle_deg):
        """Convert polar coordinates to Cartesian coordinates"""
        angle_rad = math.radians(angle_deg)
        x = distance_m * math.cos(angle_rad)
        y = distance_m * math.sin(angle_rad)
        return x, y
    
    def cartesian_to_map_coords(self, x, y):
        """Convert Cartesian coordinates to map pixel coordinates"""
        # Convert from meters to pixels and center on map
        map_x = int(MAP_CENTER + x / MAP_RESOLUTION)
        map_y = int(MAP_CENTER - y / MAP_RESOLUTION)  # Flip Y axis
        return map_x, map_y
    
    def add_point_to_map(self, x, y, intensity=1.0):
        """Add a point to the occupancy map"""
        map_x, map_y = self.cartesian_to_map_coords(x, y)
        
        # Check bounds
        if 0 <= map_x < MAP_PIXELS and 0 <= map_y < MAP_PIXELS:
            # Accumulate intensity (for multiple measurements at same location)
            self.map_data[map_y, map_x] += intensity / 255.0  # Normalize intensity
            # Cap at 1.0 to prevent overflow
            self.map_data[map_y, map_x] = min(self.map_data[map_y, map_x], 1.0)
            return True
        return False
    
    def process_frame(self, data):
        """Process a single LIDAR frame and extract points"""
        if len(data) != LD19_FRAME_SIZE:
            return []
        
        # Validate header
        if data[0] != 0x54 or data[1] != 0x2C:
            return []
        
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
            
            points = []
            self.frame_count += 1
            
            # Parse points
            for i in range(12):
                offset = 6 + (i * 3)
                distance_mm = struct.unpack('<H', data[offset:offset+2])[0]
                intensity = data[offset+2]
                
                if distance_mm > 0 and distance_mm < 12000:  # Valid range: 0-12m
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
                    if self.add_point_to_map(x, y, intensity):
                        self.valid_points += 1
                        points.append({
                            'x': x, 'y': y, 'distance': distance_m, 
                            'angle': point_angle, 'intensity': intensity
                        })
            
            self.current_points = points
            return points
            
        except Exception as e:
            print(f"Error processing frame: {e}")
            return []

class UDPReceiver:
    def __init__(self, mapper):
        self.mapper = mapper
        self.sock = None
        
    def start_receiving(self):
        """Start UDP receiver in a separate thread"""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
        self.sock.bind((UDP_IP, UDP_PORT))
        self.sock.settimeout(1.0)
        
        print(f"UDP receiver started on {UDP_IP}:{UDP_PORT}")
        
        try:
            while self.mapper.running:
                try:
                    data, addr = self.sock.recvfrom(2048)
                    points = self.mapper.process_frame(data)
                    if points:
                        self.mapper.data_queue.put({
                            'points': points,
                            'timestamp': datetime.now(),
                            'frame_count': self.mapper.frame_count
                        })
                except socket.timeout:
                    continue
        except Exception as e:
            print(f"UDP receiver error: {e}")
        finally:
            if self.sock:
                self.sock.close()

def create_visualization():
    """Create and run the matplotlib visualization"""
    mapper = LidarMapper()
    
    # Start UDP receiver in background thread
    receiver = UDPReceiver(mapper)
    udp_thread = threading.Thread(target=receiver.start_receiving, daemon=True)
    udp_thread.start()
    
    # Set up the plot
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 7))
    
    # Map view (left subplot)
    ax1.set_title("2D LIDAR Map")
    ax1.set_xlabel("X (meters)")
    ax1.set_ylabel("Y (meters)")
    ax1.set_xlim(-MAP_SIZE_M/2, MAP_SIZE_M/2)
    ax1.set_ylim(-MAP_SIZE_M/2, MAP_SIZE_M/2)
    ax1.grid(True, alpha=0.3)
    ax1.set_aspect('equal')
    
    # Add range circles
    for radius in [1, 2, 3, 4, 5]:
        circle = Circle((0, 0), radius, fill=False, color='gray', alpha=0.3, linestyle='--')
        ax1.add_patch(circle)
    
    # Real-time points view (right subplot)
    ax2.set_title("Current LIDAR Scan")
    ax2.set_xlabel("X (meters)")
    ax2.set_ylabel("Y (meters)")
    ax2.set_xlim(-6, 6)
    ax2.set_ylim(-6, 6)
    ax2.grid(True, alpha=0.3)
    ax2.set_aspect('equal')
    
    # Add LIDAR position marker
    ax1.plot(0, 0, 'ro', markersize=8, label='LIDAR Position')
    ax2.plot(0, 0, 'ro', markersize=8, label='LIDAR Position')
    
    # Initialize plot elements
    im1 = ax1.imshow(mapper.map_data, extent=[-MAP_SIZE_M/2, MAP_SIZE_M/2, -MAP_SIZE_M/2, MAP_SIZE_M/2],
                     origin='lower', cmap='hot', alpha=0.7, vmin=0, vmax=1)
    
    scatter2 = ax2.scatter([], [], c=[], s=[], cmap='viridis', alpha=0.8)
    
    # Add colorbar
    cbar = plt.colorbar(im1, ax=ax1, label='Occupancy Intensity')
    
    # Status text
    status_text = fig.suptitle("Waiting for LIDAR data...")
    distance_cbar = None  # Store colorbar reference
    
    def update_plot(frame):
        """Update the plot with new data"""
        try:
            # Get latest data if available
            latest_data = None
            while not mapper.data_queue.empty():
                latest_data = mapper.data_queue.get_nowait()
            
            if latest_data:
                points = latest_data['points']
                timestamp = latest_data['timestamp']
                frame_count = latest_data['frame_count']
                
                # Update accumulated map
                im1.set_array(mapper.map_data)
                im1.set_clim(vmin=0, vmax=np.max(mapper.map_data) if np.max(mapper.map_data) > 0 else 1)
                
                # Update current scan points
                if points:
                    x_coords = [p['x'] for p in points]
                    y_coords = [p['y'] for p in points]
                    intensities = [p['intensity'] for p in points]
                    distances = [p['distance'] for p in points]
                    
                    # Clear previous scatter plot
                    ax2.clear()
                    ax2.set_title(f"Current LIDAR Scan (Frame #{frame_count})")
                    ax2.set_xlabel("X (meters)")
                    ax2.set_ylabel("Y (meters)")
                    ax2.set_xlim(-6, 6)
                    ax2.set_ylim(-6, 6)
                    ax2.grid(True, alpha=0.3)
                    ax2.plot(0, 0, 'ro', markersize=8, label='LIDAR')
                    
                    # Plot current points with size based on intensity and color based on distance
                    sizes = [max(20, i/5) for i in intensities]  # Scale intensity to point size
                    scatter = ax2.scatter(x_coords, y_coords, c=distances, s=sizes, 
                                        cmap='viridis', alpha=0.8, edgecolors='black', linewidth=0.5)
                    
                    # Add colorbar for distance if not exists
                    nonlocal distance_cbar
                    if distance_cbar is None:
                        distance_cbar = plt.colorbar(scatter, ax=ax2, label='Distance (m)')
                
                # Update status
                validity_rate = (mapper.valid_points / mapper.total_points * 100) if mapper.total_points > 0 else 0
                status_text.set_text(f"Frame #{frame_count} | {len(points)} points | "
                                   f"Total: {mapper.valid_points}/{mapper.total_points} ({validity_rate:.1f}%) | "
                                   f"{timestamp.strftime('%H:%M:%S')}")
            
            else:
                # No new data
                status_text.set_text(f"Waiting for data... (Frame #{mapper.frame_count})")
        
        except Exception as e:
            print(f"Plot update error: {e}")
        
        return [im1]
    
    # Start animation
    ani = animation.FuncAnimation(fig, update_plot, interval=100, blit=False, cache_frame_data=False)
    
    # Add keyboard shortcuts
    def on_key(event):
        if event.key == 'c':
            # Clear map
            mapper.map_data.fill(0)
            mapper.total_points = 0
            mapper.valid_points = 0
            print("Map cleared!")
        elif event.key == 's':
            # Save map
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f"lidar_map_{timestamp}.png"
            plt.savefig(filename, dpi=300, bbox_inches='tight')
            print(f"Map saved as {filename}")
        elif event.key == 'q':
            # Quit
            mapper.running = False
            plt.close()
    
    fig.canvas.mpl_connect('key_press_event', on_key)
    
    # Add instructions
    plt.figtext(0.02, 0.02, "Controls: 'c' = clear map, 's' = save map, 'q' = quit", 
                fontsize=10, style='italic')
    
    plt.tight_layout()
    plt.show()
    
    # Clean up
    mapper.running = False

def main():
    print("LD19 LIDAR 2D Mapping Visualizer")
    print("=" * 50)
    print(f"Map size: {MAP_SIZE_M}x{MAP_SIZE_M} meters")
    print(f"Resolution: {MAP_RESOLUTION*100:.1f} cm per pixel")
    print(f"Listening on {UDP_IP}:{UDP_PORT}")
    print("\nControls:")
    print("  'c' - Clear accumulated map")
    print("  's' - Save map as PNG")
    print("  'q' - Quit application")
    print("=" * 50)
    
    try:
        create_visualization()
    except KeyboardInterrupt:
        print("\nStopped by user.")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    try:
        main()
    except ImportError as e:
        print("❌ Missing required packages. Install with:")
        print("pip install matplotlib numpy")
        print(f"Error: {e}")
        sys.exit(1)
    except PermissionError:
        print("❌ Permission denied. Try running with administrator privileges or use a port > 1024.")
        sys.exit(1)
