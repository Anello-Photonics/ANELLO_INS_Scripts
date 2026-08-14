import socket
import argparse

def nmea_checksum(sentence: str) -> str:
    csum = 0
    for ch in sentence:
        csum ^= ord(ch)
    return f"{csum:02X}"

def main():
    parser = argparse.ArgumentParser(description="Toggle GPS via NMEA command")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--enable", action="store_true", help="Enable GPS")
    group.add_argument("--disable", action="store_true", help="Disable GPS")
    group.add_argument("--disable_internal", action="store_true", help="Disable internal GPS")
    args = parser.parse_args()

    # Determine command
    if args.enable:
        gps_tog = "PAPGPSCTRL,1"
    if args.disable:
        gps_tog = "PAPGPSCTRL,0"
    if args.disable_internal:
        gps_tog = "PAPGPSCTRL,2"

    checksum = nmea_checksum(gps_tog)
    msg = f"${gps_tog}*{checksum}\r\n"

    UDP_IP = "192.168.0.3"
    UDP_PORT = 19551

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(msg.encode('ascii'), (UDP_IP, UDP_PORT))

    print("Sent:", msg.strip())

if __name__ == "__main__":
    main()