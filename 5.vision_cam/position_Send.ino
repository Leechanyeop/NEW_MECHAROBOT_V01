# DFR1154 (MicroPython 예제)
import machine, time

uart = machine.UART(1, baudrate=115200, tx=17, rx=16)

while True:
    x, y = 120, 80   # 인식된 객체 좌표 (예시)
    msg = "{},{}\n".format(x, y)
    uart.write(msg)
    time.sleep(0.1)