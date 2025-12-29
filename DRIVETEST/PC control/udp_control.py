# udp_control.py
import socket
import sys
import time

ESP32_IP = "192.168.35.147"   # <-- 실제 ESP32 IP로 변경
PORT = 4210
TIMEOUT = 2.0

def send_and_recv(payload: bytes, expect_binary=False):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(TIMEOUT)
    try:
        sock.sendto(payload, (ESP32_IP, PORT))
        data, addr = sock.recvfrom(1024)
        if expect_binary:
            print("Binary response from", addr, ":", data)
        else:
            try:
                print("Response from", addr, ":", data.decode('utf-8', errors='ignore'))
            except:
                print("Response (raw):", data)
    except socket.timeout:
        print("No response (timeout).")
    finally:
        sock.close()

def send_text(cmd: str):
    send_and_recv(cmd.encode('utf-8'))

def send_speed_value(speed: int):
    # 명시적 속도 명령: "SPD:150"
    cmd = f"SPD:{int(speed)}"
    send_text(cmd)

def send_direction(dir_char: str):
    # 단일 문자 명령: 'w','s','a','d','x'
    send_text(dir_char)

def send_servo(servo_id: int, angle: int):
    # 텍스트 서보 명령: SV1:90 또는 SV2:45
    cmd = f"SV{servo_id}:{angle}"
    send_text(cmd)

def send_binary_motor(header: int, cmd_id: int, value: int):
    # 예: 바이너리 포맷 [header, cmd_id, value]
    pkt = bytes([header & 0xFF, cmd_id & 0xFF, value & 0xFF])
    send_and_recv(pkt, expect_binary=True)

def map_speed_to_digit(speed_0_255: int) -> str:
    # 기존 Mega 스케치가 0..9로 매핑하는 경우 사용
    # 0->'0', 255->'9'
    n = max(0, min(255, speed_0_255))
    digit = int(round(n / 255 * 9))
    return str(digit)

def interactive_mode():
    print("Interactive mode. Type commands and Enter.")
    print("  w/s/a/d/x  -> forward/back/left/right/stop")
    print("  0-9        -> speed preset (0->0 ... 9->255)")
    print("  SPD <0-255>-> set explicit speed")
    print("  SV1 <angle>, SV2 <angle> -> servo control")
    print("  R          -> request sensors")
    print("  QUIT       -> exit")
    while True:
        try:
            line = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nExiting interactive mode.")
            break
        if not line:
            continue
        parts = line.split()
        cmd = parts[0].upper()
        if cmd in ("QUIT","EXIT"):
            break
        if cmd in ("W","S","A","D","X"):
            send_direction(cmd.lower())
            continue
        if len(cmd) == 1 and cmd.isdigit():
            send_text(cmd)
            continue
        if cmd == "SPD" and len(parts) >= 2:
            try:
                sp = int(parts[1])
                sp = max(0, min(255, sp))
                send_speed_value(sp)
            except:
                print("Invalid speed. Use 0-255.")
            continue
        if cmd.startswith("SV") and len(parts) >= 2:
            try:
                sid = int(cmd[2:])
                angle = int(parts[1])
                send_servo(sid, angle)
            except:
                print("Usage: SV1 90")
            continue
        if cmd == "R":
            send_text("R")
            continue
        # allow direct raw text like "SV1:90" or "w"
        send_text(line)

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage examples:")
        print("  python udp_control.py w                # forward")
        print("  python udp_control.py x                # stop")
        print("  python udp_control.py 1 120            # servo1 -> 120 (alternative)")
        print("  python udp_control.py SPD 150          # set speed 150 (0-255)")
        print("  python udp_control.py INTERACTIVE      # enter interactive mode")
        print("  python udp_control.py BIN 0x10 0x01 90 # binary header,id,value")
        sys.exit(0)

    cmd = sys.argv[1].upper()

    # Interactive mode
    if cmd == "INTERACTIVE":
        interactive_mode()
        sys.exit(0)

    # 방향/모터 문자 (w,s,a,d,x) 처리
    if cmd in ("W","S","A","D","X"):
        send_direction(cmd.lower())
        sys.exit(0)

    # 숫자 한 글자로 속도(0-9) 전송 (기존 스케치와 호환)
    if len(cmd) == 1 and cmd.isdigit():
        send_text(cmd)
        sys.exit(0)

    # SPD 명시적 속도
    if cmd == "SPD" and len(sys.argv) >= 3:
        speed = int(sys.argv[2])
        speed = max(0, min(255, speed))
        # 기존 Mega 스케치가 '0'..'9'만 처리한다면 map to digit:
        # digit = map_speed_to_digit(speed)
        # send_text(digit)
        # 여기서는 SPD:xxx 텍스트로 보냄 (ESP32/Mega가 처리하면 됨)
        send_speed_value(speed)
        sys.exit(0)

    # 서보 텍스트: SV1 90  또는 SV2 45  또는 SV1:90
    if cmd.startswith("SV"):
        if len(sys.argv) >= 3:
            try:
                servo_id = int(cmd[2:])
                angle = int(sys.argv[2])
                send_servo(servo_id, angle)
                sys.exit(0)
            except:
                print("Usage: python udp_control.py SV1 90")
                sys.exit(1)
        else:
            # allow "SV1:90" raw
            raw = " ".join(sys.argv[1:])
            send_text(raw)
            sys.exit(0)

    # 바이너리 전송: BIN header id value
    if cmd == "BIN" and len(sys.argv) >= 4:
        header = int(sys.argv[2], 0)
        cmd_id = int(sys.argv[3], 0)
        value = int(sys.argv[4], 0)
        send_binary_motor(header, cmd_id, value)
        sys.exit(0)

    # 센서 요청 'R' 또는 기타 원시 텍스트
    raw = " ".join(sys.argv[1:])
    send_text(raw)