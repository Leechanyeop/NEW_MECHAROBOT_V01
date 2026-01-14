import socket, struct, time

ESP_IP = "192.168.0.66"
ESP_PORT = 4210

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.settimeout(3)
while True:
    try:
        sock.connect((ESP_IP, ESP_PORT))
        break
    except Exception:
        time.sleep(1)

try:
    while True:
        # 예: 200ms 간격으로 IR 요청
        sock.sendall(b'I\n')  # 서버가 '\n' 기준으로 처리하면 텍스트로 보냄
        data = sock.recv(12)
        if len(data) == 12:
            vals = struct.unpack('<6H', data)
            print("IR:", vals)
        time.sleep(0.2)
except KeyboardInterrupt:
    pass
finally:
    sock.close()