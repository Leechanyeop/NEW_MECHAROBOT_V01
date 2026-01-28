# send_udp_frame.py
import socket, struct, binascii

def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc

udp_ip = "192.168.0.74"  # ESP32 IP
udp_port = 8888

# payload 예: Cmd=0x10 (SET_SERVO), servoId=1, angle=90
payload = bytes([0x10, 0x01, 90])
frame = bytearray()
frame += b'\xAA'            # Start
frame += b'\x01'            # Version
frame += b'\x01'            # MsgType CMD
frame += b'\x05'            # Seq
frame += bytes([len(payload)]) 
frame += payload
crc = crc16(frame[:5+len(payload)])
frame += struct.pack('>H', crc)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(frame, (udp_ip, udp_port))
print("Sent", binascii.hexlify(frame))