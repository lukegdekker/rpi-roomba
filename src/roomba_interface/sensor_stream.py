#!/usr/bin/env python3
import serial
import time
import rospy

ser = serial.Serial(port="/dev/ttyUSB0", baudrate=115200)

print("Starting OI...")
ser.write(bytes([128]))
time.sleep(1)
print("Setting OI Mode to Safe...")
ser.write(bytes([131]))
time.sleep(1)

while True:
    ser.write(bytes([142,100]))
    byte = []
    val = 0
    while ser.in_waiting:
        byte.append(ser.read())
    if byte:
        print("====================================== BYTE ==================================")
        print(byte)
        #val = int.from_bytes(, "big", signed=False)
        #print(val)
    
    ser.write(bytes([137,0,100,128,0]))
    time.sleep(0.2)

ser.close()
