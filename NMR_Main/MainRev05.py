import socket
import struct

ESP32_IP = "192.168.0.78"   # ESP32 IP (시리얼 모니터에서 확인)
ESP32_PORT = 4210           # ESP32 UDP 포트
LOCAL_PORT = 4211           # PC 포트

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("", LOCAL_PORT))

print("UDP 클라이언트 시작. ESP32에서 IR 6개 값 요청 중...")

try:
    while True:
        # ESP32에 'I' 명령 보내기 (IR 전체값 요청)
        sock.sendto(b'I', (ESP32_IP, ESP32_PORT))

        data, addr = sock.recvfrom(1024)
        if len(data) == 12:  # 6개 센서 * 2바이트
            values = struct.unpack("<6H", data)  # 리틀엔디안 uint16_t 6개
            print("IR Sensors:", values)
        else:
            print("수신 데이터:", data)

except KeyboardInterrupt:
    print("종료합니다.")
finally:
    sock.close()