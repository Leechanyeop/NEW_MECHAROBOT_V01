import socket
import threading
import sys

HOST = '0.0.0.0'
PORT = 5000

# Global active connection
active_conn = None

def input_thread():
    global active_conn
    print("Commands: w, a, s, d, x (or any string). Type 'q' to quit server.")
    while True:
        try:
            # 파이썬 input()은 블로킹되므로 별도 쓰레드에서 실행
            cmd = input() 
            if cmd == 'q':
                print("Stopping server...")
                if active_conn:
                    active_conn.close()
                # Main thread will handle exit
                import os
                os._exit(0)
            
            if active_conn:
                # Send command with newline
                try:
                    active_conn.sendall((cmd + '\n').encode('utf-8'))
                    print(f"Sent: {cmd}")
                except Exception as e:
                    print("Send error:", e)
                    active_conn = None
            else:
                print("No client connected. Command ignored.")
        except EOFError:
            break

def parse_frame(line):
    # 예: "<P,X:123.45,Y:67.89,T:1.570>"
    try:
        line = line.strip()
        if not line.startswith("<P") or not line.endswith(">"):
            return None
        body = line[1:-1]  # remove < and >
        parts = body.split(',')
        data = {}
        for p in parts:
            if ':' in p:
                k, v = p.split(':', 1)
                data[k.strip()] = v.strip()
        # X key is "X", Y key is "Y", T key is "T"
        x = float(data.get('X', 0.0))
        y = float(data.get('Y', 0.0))
        t = float(data.get('T', 0.0))
        return (x, y, t)
    except Exception as e:
        # print("Parse error:", e) # Ignore partial frames
        return None

# Start input thread
t = threading.Thread(target=input_thread)
t.daemon = True
t.start()

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen(1)
    print("Server listening on", PORT)
    
    while True:
        print("Waiting for connection...")
        conn, addr = s.accept()
        active_conn = conn
        with conn:
            print('Connected by', addr)
            buffer = b""
            while True:
                try:
                    data = conn.recv(1024)
                    if not data:
                        print("Client disconnected")
                        active_conn = None
                        break
                    
                    buffer += data
                    buffer += data
                    # < ... > 패턴을 처리하기 위해 '>' 로 분할 시도
                    while b'>' in buffer:
                        # '>' 기준으로 자름 (line에는 '>'가 포함되지 않음)
                        line_chunk, buffer = buffer.split(b'>', 1)
                        # 다시 '>'를 붙여서 온전한 프레임으로 만듦
                        line_str = line_chunk.decode('utf-8', errors='ignore') + ">"
                        
                        # 혹시 앞부분에 '<' 이전의 쓰레기 값이 있을 수 있으므로 '<' 위치 찾기
                        idx = line_str.find('<')
                        if idx != -1:
                            line_str = line_str[idx:] # < 부터 시작하도록 정리

                            parsed = parse_frame(line_str)
                            if parsed:
                                x, y, t = parsed
                                print(f"Pose: X={x:.2f}, Y={y:.2f}, T={t:.3f}")
                            else:
                                if line_str.strip():
                                    print("Raw:", line_str)
                        else:
                            # < 가 없으면 그냥 출력 (디버깅)
                             if line_str.strip() != ">":
                                print("Raw (no start):", line_str)
                except ConnectionResetError:
                    print("Connection reset")
                    active_conn = None
                    break