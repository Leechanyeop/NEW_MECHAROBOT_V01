import socket

# 서버 설정
HOST = '0.0.0.0'
PORT = 8888

def start_server():
    # 소켓 생성
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    
    # 포트 재사용 설정 (이미 사용 중인 포트 에러 방지 유용)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    try:
        s.bind((HOST, PORT))
        s.listen()
        print(f"Server started on port {PORT}. Waiting for ESP32...")

        while True:
            conn, addr = s.accept()
            print(f"\n[Connected] by {addr}")
            with conn:
                while True:
                    try:
                        data = conn.recv(1024)
                        if not data:
                            print("[Disconnected] Client closed connection.")
                            break
                        
                        message = data.decode('utf-8').strip()
                        print(f"Received: {message}")
                    except ConnectionResetError:
                        print("[Disconnected] Connection was reset by client.")
                        break
    except Exception as e:
        print(f"Error: {e}")
    finally:
        s.close()

if __name__ == "__main__":
    start_server()
