import socket

# ESP32의 IP와 포트 (ESP32 UDP 서버가 열려 있어야 함)
ESP32_IP = "192.168.35.172"   # ESP32가 연결된 Wi-Fi IP
ESP32_PORT = 8888           # ESP32에서 열어둔 포트

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print("ESP32 UDP 송신 시작. 문자열 입력 후 Enter를 누르세요.")
while True:
    msg = input(">> ")  # 키보드 입력
    if msg.lower() == "exit":
        break
    sock.sendto(msg.encode(), (ESP32_IP, ESP32_PORT))

sock.close()