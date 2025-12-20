#get_position

import serial

ser = serial.Serial("COM3", 115200)
while True:
    line = ser.readline().decode().strip()
    print("ESP32 수신:", line)