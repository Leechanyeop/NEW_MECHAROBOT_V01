# udp_servo_bin.py
import socket
import sys

ESP32_IP = "192.168.0.66"  # <-- 실제 IP로 변경
PORT = 4210
TIMEOUT = 2.0

def send_binary(header: int, servo_id: int, angle: int):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(TIMEOUT)
    pkt = bytes([header & 0xFF, servo_id & 0xFF, angle & 0xFF])
    try:
        sock.sendto(pkt, (ESP32_IP, PORT))
        data, addr = sock.recvfrom(1024)
        print("Binary response from", addr, ":", data)
    except socket.timeout:
        print("No response (timeout).")
    finally:
        sock.close()

if __name__ == "__main__":
    if len(sys.argv) >= 3:
        sid = int(sys.argv[1])   # 1 or 2
        angle = int(sys.argv[2])
        send_binary(0x10, sid, angle)
    else:
        print("Usage: python udp_servo_bin.py 1 90")