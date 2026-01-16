import socket

def nmea_checksum(sentence: str) -> str:
    csum = 0
    for ch in sentence:
        csum ^= ord(ch)
    return f"{csum:02X}"

# Build body without $ or *hh
body = "VMVBW,0.0,0.0,A,0.0,0.0,A"
gps_en = "PAPGPSCTRL,1"
gps_dis = "PAPGPSCTRL,0"
gps_pos = "PAPPOS,,37.335390,-122.026130,12.30,2.0,3.5,2"
attitude = "PAPRPH,203600.00,0.80,1.50,273.20,0.20,0.20,0.50,2"
autocal_en = "PAPAUTOCAL,1"
autocal_dis = "PAPAUTOCAL,0"

checksum = nmea_checksum(autocal_en)
msg = f"${autocal_en}*{checksum}\r\n"

UDP_IP = "192.168.0.3"
UDP_PORT = 19551

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(msg.encode('ascii'), (UDP_IP, UDP_PORT))
print("Sent:", msg)

