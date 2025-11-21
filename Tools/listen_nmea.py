import socket

UDP_IP = "0.0.0.0"   # listen on all interfaces
UDP_PORT = 19550

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening for NMEA sentences on UDP {UDP_PORT}...\n")

while True:
    data, addr = sock.recvfrom(1024)  # buffer size
    print(data.decode('ascii', errors='ignore').strip())
