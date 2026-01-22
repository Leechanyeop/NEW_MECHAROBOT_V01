import socket
import sys
import threading

# 서버 설정
HOST = '0.0.0.0'
PORT = 8888

def handle_keyboard_input(conn):
    """키보드 입력을 읽어 클라이언트(ESP32)에게 전송하는 함수"""
    # 전송 지연을 없애기 위해 TCP_NODELAY 설정
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    print("Keyboard Control Active: Type w, a, s, d, x, 1 and press Enter.")
    try:
        while True:
            cmd = sys.stdin.readline().strip()
            if cmd:
                # 명령 뒤에 \n을 붙여서 전송 (전송 보장 및 구분용)
                # 이전 수정에서 발생한 s"\n" 오타를 수정합니다.
                conn.sendall((cmd + "\n").encode('utf-8'))
                print(f"[REPLY] Sent to ESP32: {cmd}")
    except Exception as e:
        print(f"Keyboard thread error: {e}")

def start_server():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.settimeout(1.0)
    
    try:
        s.bind((HOST, PORT))
        s.listen()
        print(f"Server started on port {PORT}. Waiting for ESP32...")

        while True:
            try:
                conn, addr = s.accept()
                print(f"\n[Connected] by {addr}")
                
                # 키보드 입력을 위한 쓰레드 시작
                input_thread = threading.Thread(target=handle_keyboard_input, args=(conn,), daemon=True)
                input_thread.start()

                with conn:
                    conn.settimeout(None) 
                    while True:
                        try:
                            # 로봇으로부터의 데이터 수신
                            data = conn.recv(1024)
                            if not data:
                                print("\n[Disconnected] Client closed connection.")
                                break
                            
                            message = data.decode('utf-8', errors='ignore').strip()
                            # 받은 좌표값 출력 (새 줄로 출력하여 겹침 방지)
                            if message:
                                print(f"[RECEIVE] {message}")
                                
                        except ConnectionResetError:
                            print("\n[Disconnected] Connection reset by client.")
                            break
            except socket.timeout:
                continue 
                
    except KeyboardInterrupt:
        print("\n[STOP] Server stopping by user...")
    finally:
        s.close()
        print("Exit.")

if __name__ == "__main__":
    start_server()
