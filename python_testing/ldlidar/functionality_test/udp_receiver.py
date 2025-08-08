# udp_recv_ld19.py
import socket
import sys
from datetime import datetime

UDP_IP   = "0.0.0.0"   # listen on all interfaces
UDP_PORT = 5005        # must match the ESP32's UDP_PORT

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # Increase buffer if you want (not strictly necessary here):
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
    sock.bind((UDP_IP, UDP_PORT))

    print(f"Listening on {UDP_IP}:{UDP_PORT} ...")
    try:
        while True:
            data, addr = sock.recvfrom(2048)  # LD19 frame is 47 bytes
            ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            # Pretty print: first 32 hex bytes (or fewer)
            preview = " ".join(f"{b:02X}" for b in data[:32])
            print(f"[{ts}] from {addr[0]}:{addr[1]} len={len(data)}  {preview}")
    except KeyboardInterrupt:
        print("\nStopped.")

if __name__ == "__main__":
    try:
        main()
    except PermissionError:
        print("Permission error: try a different port (>1024) or run with proper rights.", file=sys.stderr)
