import socket
import json

UDP_IP = "192.168.69.60"  # Raspberry Pi's IP
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
test_data = [[0, 500], [90, 750], [180, 300]]
message = json.dumps(test_data)
sock.sendto(message.encode('utf-8'), (UDP_IP, UDP_PORT))
sock.close()
print(f"Test message sent to {UDP_IP}:{UDP_PORT}")