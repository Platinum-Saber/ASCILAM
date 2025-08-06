import socket
import json

# ==== CONFIG ====
UDP_IP = '0.0.0.0'  # Listen on all interfaces
UDP_PORT = 5005

# ==== SETUP SOCKET ====
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.settimeout(5.0)  # 5 second timeout

print(f"UDP Test Receiver listening on {UDP_IP}:{UDP_PORT}")
print("Waiting for test messages... Press Ctrl+C to stop.")

# ==== MAIN LOOP ====
message_count = 0
try:
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            message_count += 1
            
            print(f"\n--- Message #{message_count} ---")
            print(f"From: {addr}")
            print(f"Raw data length: {len(data)} bytes")
            print(f"Raw data: {data}")
            
            try:
                # Try to decode as JSON
                decoded_message = data.decode('utf-8')
                print(f"Decoded string: {decoded_message}")
                
                parsed_data = json.loads(decoded_message)
                print(f"Parsed JSON: {parsed_data}")
                print(f"Data type: {type(parsed_data)}")
                
                if isinstance(parsed_data, list):
                    print(f"Number of data points: {len(parsed_data)}")
                    if len(parsed_data) > 0:
                        print(f"First data point: {parsed_data[0]}")
                        if len(parsed_data) > 1:
                            print(f"Last data point: {parsed_data[-1]}")
                            
            except json.JSONDecodeError as e:
                print(f"JSON decode error: {e}")
            except UnicodeDecodeError as e:
                print(f"Unicode decode error: {e}")
                
        except socket.timeout:
            print(".", end="", flush=True)  # Print dot every timeout to show it's alive
            continue
            
        except Exception as e:
            print(f"Unexpected error: {e}")

except KeyboardInterrupt:
    print(f"\n\nReceived {message_count} messages total.")
    print("Stopped by user.")

finally:
    sock.close()
    print("Socket closed.")
