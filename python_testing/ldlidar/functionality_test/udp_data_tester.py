#!/usr/bin/env python3
"""
LD19 LIDAR UDP Data Validator and Tester
Comprehensive tool for validating and testing UDP LIDAR data format.
"""

import socket
import struct
import sys
from datetime import datetime
import json

# Configuration
UDP_IP = "0.0.0.0"
UDP_PORT = 5005
LD19_FRAME_SIZE = 47

class LD19Validator:
    def __init__(self):
        self.total_frames = 0
        self.valid_frames = 0
        self.invalid_frames = 0
        self.error_counts = {
            'size_mismatch': 0,
            'header_invalid': 0,
            'verlen_invalid': 0,
            'parse_error': 0
        }
        self.speed_stats = {'min': float('inf'), 'max': 0, 'sum': 0, 'count': 0}
        self.distance_stats = {'min': float('inf'), 'max': 0, 'sum': 0, 'count': 0}
        
    def validate_frame(self, data, addr):
        """Validate and analyze a single frame"""
        self.total_frames += 1
        receive_time = datetime.now()
        
        result = {
            'timestamp': receive_time.isoformat(),
            'source': f"{addr[0]}:{addr[1]}",
            'size': len(data),
            'valid': False,
            'errors': [],
            'data': {}
        }
        
        # Check frame size
        if len(data) != LD19_FRAME_SIZE:
            self.error_counts['size_mismatch'] += 1
            result['errors'].append(f"Size mismatch: {len(data)} != {LD19_FRAME_SIZE}")
            self.invalid_frames += 1
            return result
        
        # Check header
        header = data[0]
        verlen = data[1]
        
        if header != 0x54:
            self.error_counts['header_invalid'] += 1
            result['errors'].append(f"Invalid header: 0x{header:02X}")
        
        if verlen != 0x2C:
            self.error_counts['verlen_invalid'] += 1
            result['errors'].append(f"Invalid verlen: 0x{verlen:02X}")
        
        if result['errors']:
            self.invalid_frames += 1
            return result
        
        # Parse frame data
        try:
            # Basic frame info
            speed_dps = struct.unpack('<H', data[2:4])[0]
            start_angle = struct.unpack('<H', data[4:6])[0]
            end_angle = struct.unpack('<H', data[42:44])[0]
            timestamp_ms = struct.unpack('<H', data[44:46])[0]
            crc8 = data[46]
            
            # Parse points
            points = []
            valid_points = 0
            for i in range(12):
                offset = 6 + (i * 3)
                distance_mm = struct.unpack('<H', data[offset:offset+2])[0]
                intensity = data[offset+2]
                points.append({'distance_mm': distance_mm, 'intensity': intensity})
                
                if distance_mm > 0:
                    valid_points += 1
                    # Update distance stats
                    self.distance_stats['min'] = min(self.distance_stats['min'], distance_mm)
                    self.distance_stats['max'] = max(self.distance_stats['max'], distance_mm)
                    self.distance_stats['sum'] += distance_mm
                    self.distance_stats['count'] += 1
            
            # Update speed stats
            if speed_dps > 0:
                self.speed_stats['min'] = min(self.speed_stats['min'], speed_dps)
                self.speed_stats['max'] = max(self.speed_stats['max'], speed_dps)
                self.speed_stats['sum'] += speed_dps
                self.speed_stats['count'] += 1
            
            result['valid'] = True
            result['data'] = {
                'speed_dps': speed_dps,
                'start_angle': start_angle / 100.0,
                'end_angle': end_angle / 100.0,
                'timestamp_ms': timestamp_ms,
                'crc8': f"0x{crc8:02X}",
                'valid_points': valid_points,
                'points': points
            }
            
            self.valid_frames += 1
            
        except Exception as e:
            self.error_counts['parse_error'] += 1
            result['errors'].append(f"Parse error: {str(e)}")
            self.invalid_frames += 1
            
        return result
    
    def get_statistics(self):
        """Get comprehensive statistics"""
        stats = {
            'total_frames': self.total_frames,
            'valid_frames': self.valid_frames,
            'invalid_frames': self.invalid_frames,
            'validity_rate': (self.valid_frames / self.total_frames * 100) if self.total_frames > 0 else 0,
            'error_breakdown': self.error_counts.copy()
        }
        
        if self.speed_stats['count'] > 0:
            stats['speed_stats'] = {
                'min_dps': self.speed_stats['min'],
                'max_dps': self.speed_stats['max'],
                'avg_dps': self.speed_stats['sum'] / self.speed_stats['count']
            }
        
        if self.distance_stats['count'] > 0:
            stats['distance_stats'] = {
                'min_mm': self.distance_stats['min'],
                'max_mm': self.distance_stats['max'],
                'avg_mm': self.distance_stats['sum'] / self.distance_stats['count']
            }
        
        return stats

def print_frame_analysis(result):
    """Print detailed frame analysis"""
    timestamp = datetime.fromisoformat(result['timestamp']).strftime("%H:%M:%S.%f")[:-3]
    status = "✅ VALID" if result['valid'] else "❌ INVALID"
    
    print(f"\n[{timestamp}] Frame from {result['source']} - {status}")
    print(f"Size: {result['size']} bytes")
    
    if result['errors']:
        print("Errors:")
        for error in result['errors']:
            print(f"  - {error}")
    
    if result['valid'] and result['data']:
        data = result['data']
        print(f"Speed: {data['speed_dps']} dps")
        print(f"Angles: {data['start_angle']:.2f}° to {data['end_angle']:.2f}°")
        print(f"Timestamp: {data['timestamp_ms']} ms")
        print(f"CRC8: {data['crc8']}")
        print(f"Valid points: {data['valid_points']}/12")
        
        # Show point summary
        distances = [p['distance_mm'] for p in data['points'] if p['distance_mm'] > 0]
        if distances:
            print(f"Distance range: {min(distances)} - {max(distances)} mm")
            print(f"Average distance: {sum(distances)/len(distances):.1f} mm")

def print_statistics(validator):
    """Print comprehensive statistics"""
    stats = validator.get_statistics()
    
    print(f"\n{'='*60}")
    print(f"COMPREHENSIVE STATISTICS")
    print(f"{'='*60}")
    print(f"Total frames: {stats['total_frames']}")
    print(f"Valid frames: {stats['valid_frames']}")
    print(f"Invalid frames: {stats['invalid_frames']}")
    print(f"Validity rate: {stats['validity_rate']:.1f}%")
    
    print(f"\nError breakdown:")
    for error_type, count in stats['error_breakdown'].items():
        print(f"  {error_type}: {count}")
    
    if 'speed_stats' in stats:
        speed = stats['speed_stats']
        print(f"\nSpeed statistics:")
        print(f"  Range: {speed['min_dps']} - {speed['max_dps']} dps")
        print(f"  Average: {speed['avg_dps']:.1f} dps")
    
    if 'distance_stats' in stats:
        dist = stats['distance_stats']
        print(f"\nDistance statistics:")
        print(f"  Range: {dist['min_mm']} - {dist['max_mm']} mm")
        print(f"  Average: {dist['avg_mm']:.1f} mm")
        print(f"  Range (meters): {dist['min_mm']/1000:.2f} - {dist['max_mm']/1000:.2f} m")

def save_results_to_file(results, filename):
    """Save analysis results to JSON file"""
    try:
        with open(filename, 'w') as f:
            json.dump(results, f, indent=2)
        print(f"Results saved to {filename}")
    except Exception as e:
        print(f"Error saving results: {e}")

def main():
    print("LD19 LIDAR UDP Data Validator and Tester")
    print("="*50)
    print(f"Listening on {UDP_IP}:{UDP_PORT}")
    print("Options:")
    print("  Press 's' + Enter to show statistics")
    print("  Press 'q' + Enter to quit")
    print("  Press 'd' + Enter to toggle detailed output")
    print("="*50)
    
    # Create socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
    sock.bind((UDP_IP, UDP_PORT))
    sock.settimeout(1.0)  # 1 second timeout for checking user input
    
    validator = LD19Validator()
    detailed_output = False
    all_results = []
    
    try:
        while True:
            try:
                # Receive data
                data, addr = sock.recvfrom(2048)
                
                # Validate frame
                result = validator.validate_frame(data, addr)
                all_results.append(result)
                
                # Print analysis based on mode
                if detailed_output:
                    print_frame_analysis(result)
                else:
                    # Compact output
                    timestamp = datetime.fromisoformat(result['timestamp']).strftime("%H:%M:%S")
                    status = "✅" if result['valid'] else "❌"
                    print(f"[{timestamp}] {status} Frame #{validator.total_frames} from {result['source']}")
                
                # Show periodic statistics
                if validator.total_frames % 50 == 0:
                    print(f"\n--- Quick Stats (Frame #{validator.total_frames}) ---")
                    print(f"Validity: {validator.valid_frames}/{validator.total_frames} ({validator.valid_frames/validator.total_frames*100:.1f}%)")
                    
            except socket.timeout:
                # Check for user input
                try:
                    if sys.stdin.readable():
                        user_input = input().strip().lower()
                        if user_input == 'q':
                            break
                        elif user_input == 's':
                            print_statistics(validator)
                        elif user_input == 'd':
                            detailed_output = not detailed_output
                            print(f"Detailed output: {'ON' if detailed_output else 'OFF'}")
                        elif user_input == 'save':
                            filename = f"lidar_analysis_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
                            save_results_to_file(all_results, filename)
                except:
                    pass
                continue
                
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        print_statistics(validator)
        
        # Offer to save results
        if all_results:
            try:
                save_choice = input("\nSave detailed results to file? (y/n): ").strip().lower()
                if save_choice == 'y':
                    filename = f"lidar_analysis_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
                    save_results_to_file(all_results, filename)
            except:
                pass
        
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
