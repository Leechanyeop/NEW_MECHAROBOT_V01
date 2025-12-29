# udp_keys_control.py
# Windows 전용: CMD에서 실행 (msvcrt 사용)
import socket
import sys
import time
import msvcrt

# 설정: 실제 ESP32 IP로 변경
ESP32_IP = "192.168.35.147"
PORT = 4210
RECV_TIMEOUT = 0.08  # 수신 대기 시간(초)

# 상태
servo1_angle = 90
servo2_angle = 90
speed_value = 150  # 0-255

# UDP 소켓 재사용
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(RECV_TIMEOUT)

def send_raw_bytes(b: bytes, expect_response=False):
    try:
        sock.sendto(b, (ESP32_IP, PORT))
        if expect_response:
            try:
                data, addr = sock.recvfrom(1024)
                print("Resp:", data)
            except socket.timeout:
                print("No response")
    except Exception as e:
        print("Send error:", e)

def send_text(cmd: str, expect_response=False):
    send_raw_bytes(cmd.encode('utf-8'), expect_response)

def send_speed(sp: int):
    global speed_value
    speed_value = max(0, min(255, sp))
    # ESP32/Mega가 SPD: 형식을 처리하면 사용, 아니면 map to digit
    send_text(f"SPD:{speed_value}", expect_response=False)
    print(f"Speed set -> {speed_value}")

def map_speed_to_digit(sp: int) -> str:
    n = max(0, min(255, sp))
    digit = int(round(n / 255 * 9))
    return str(digit)

def print_help():
    print("Controls:")
    print("  Arrow Up    : forward (w)")
    print("  Arrow Down  : backward (s)")
    print("  Arrow Left  : left (a)")
    print("  Arrow Right : right (d)")
    print("  Space       : stop (x)")
    print("  0-9         : speed preset (sends '0'..'9')")
    print("  + / -       : increase/decrease speed by 10 (sends SPD:<n>)")
    print("  o / p       : servo1 -/+ 5deg (SV1:<angle>)")
    print("  k / l       : servo2 -/+ 5deg (SV2:<angle>)")
    print("  r           : request sensors (R)")
    print("  q           : quit")
    print("")

def read_key():
    # msvcrt.getch returns bytes
    if not msvcrt.kbhit():
        return None
    ch = msvcrt.getch()
    # arrow keys: first b'\xe0' or b'\x00', then code
    if ch in (b'\x00', b'\xe0'):
        ch2 = msvcrt.getch()
        return (ch, ch2)
    return ch

def main_loop():
    global servo1_angle, servo2_angle, speed_value
    print("UDP key control started. ESP32:", ESP32_IP, "port:", PORT)
    print_help()
    try:
        while True:
            k = read_key()
            if k is None:
                time.sleep(0.01)
                continue

            # Arrow keys
            if isinstance(k, tuple):
                code = k[1]
                if code == b'H':       # up
                    send_text('w')
                    print("-> forward")
                elif code == b'P':     # down
                    send_text('s')
                    print("-> backward")
                elif code == b'K':     # left
                    send_text('a')
                    print("-> left")
                elif code == b'M':     # right
                    send_text('d')
                    print("-> right")
                else:
                    print("Unknown special key:", k)
                continue

            # Single-byte keys
            key = k.decode('utf-8', errors='ignore')
            if key == ' ':
                send_text('x')
                print("-> stop")
            elif key in '0123456789':
                # send single-digit preset (legacy)
                send_text(key)
                print(f"-> preset speed digit {key}")
            elif key == '+':
                speed_value = min(255, speed_value + 10)
                send_speed(speed_value)
            elif key == '-':
                speed_value = max(0, speed_value - 10)
                send_speed(speed_value)
            elif key.lower() == 'o':
                servo1_angle = max(0, servo1_angle - 5)
                send_text(f"SV1:{servo1_angle}")
                print(f"-> SV1 {servo1_angle}")
            elif key.lower() == 'p':
                servo1_angle = min(180, servo1_angle + 5)
                send_text(f"SV1:{servo1_angle}")
                print(f"-> SV1 {servo1_angle}")
            elif key.lower() == 'k':
                servo2_angle = max(0, servo2_angle - 5)
                send_text(f"SV2:{servo2_angle}")
                print(f"-> SV2 {servo2_angle}")
            elif key.lower() == 'l':
                servo2_angle = min(180, servo2_angle + 5)
                send_text(f"SV2:{servo2_angle}")
                print(f"-> SV2 {servo2_angle}")
            elif key.lower() == 'r':
                send_text('R', expect_response=True)
                print("-> sensor request sent")
            elif key.lower() == 'q':
                print("Quitting.")
                break
            else:
                # raw send for any other typed command (e.g., "SV1:90" typed and Enter not needed)
                send_text(key)
                print("Sent raw:", repr(key))

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        sock.close()

if __name__ == "__main__":
    main_loop()