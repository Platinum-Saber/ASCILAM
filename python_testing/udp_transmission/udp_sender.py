import socket
import json
from rplidar import RPLidar

# ==== CONFIG ====
LIDAR_PORT = 'COM9'  
UDP_IP = '192.168.69.60'  #Raspberry Pi IP
UDP_PORT = 5005

# ==== INIT LIDAR ====
lidar = RPLidar(LIDAR_PORT)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print(f"Streaming LIDAR data (<=1m) to {UDP_IP}:{UDP_PORT}... Press Ctrl+C to stop.")

try:
    for scan in lidar.iter_scans():
        # Only keep points <= 1m
        scan_data = [
            (round(angle, 2), round(distance, 2))
            for _, angle, distance in scan
            if 0 < distance <= 1000  # Filter out >1 meter
        ]

        if not scan_data:
            continue  # Skip empty scans

        message = json.dumps(scan_data)
        sock.sendto(message.encode('utf-8'), (UDP_IP, UDP_PORT))

except KeyboardInterrupt:
    print("\nStopping...")

finally:
    lidar.stop()
    lidar.disconnect()
    sock.close()
    print("LIDAR disconnected. UDP socket closed.")
