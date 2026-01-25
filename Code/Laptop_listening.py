import socket

# Configuration
UDP_IP = "0.0.0.0" # Listen on all available network interfaces
UDP_PORT = 4210    # Must match the port in your ESP32 code

# Create the socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening for data on port {UDP_PORT}...")

try:
    while True:
        # Buffer size is 1024 bytes
        data, addr = sock.recvfrom(1024) 
        message = data.decode('utf-8')
        print(f"From ESP32 ({addr[0]}): {message}")
        
except KeyboardInterrupt:
    print("\nStopped.")
    sock.close()