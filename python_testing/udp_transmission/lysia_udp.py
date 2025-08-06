import socket
import json
import numpy as np
import matplotlib.pyplot as plt

# ==== CONFIG ====
UDP_IP = '0.0.0.0'
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.settimeout(2.0)

# ==== PLOT SETUP ====
fig = plt.figure()
ax = plt.subplot(1, 1, 1, projection='polar')
plt.title("Real-time LIDAR Scan (1m radius via UDP)")
ax.set_theta_zero_location('N')  # type: ignore
ax.set_theta_direction(-1)       # type: ignore
ax.set_ylim(0, 1000)

print(f"Listening for filtered LIDAR data on {UDP_IP}:{UDP_PORT}...")
print(f"Socket bound successfully. Waiting for data...")

def update_plot(data):
    ax.clear()
    ax.set_theta_zero_location('N')  # type: ignore
    ax.set_theta_direction(-1)       # type: ignore
    ax.set_ylim(0, 1000)
    ax.grid(True)

    if not data:
        return

    angles = np.radians([angle for angle, dist in data])
    distances = [dist for angle, dist in data]
    ax.scatter(angles, distances, s=3, c='green')
    plt.pause(0.001)

# ==== MAIN LOOP ====
try:
    while True:
        try:
            data, addr = sock.recvfrom(65535)
            print(f"Received data from {addr}: {len(data)} bytes")  # Debug output
            scan_data = json.loads(data.decode('utf-8'))
            print(f"Parsed {len(scan_data)} data points")  # Debug output
            update_plot(scan_data)
        except socket.timeout:
            print("Timeout - no data received")  # Debug output
            continue
        except Exception as e:
            print("Error:", e)

except KeyboardInterrupt:
    print("\nStopped by user.")

finally:
    sock.close()
    print("Socket closed.")