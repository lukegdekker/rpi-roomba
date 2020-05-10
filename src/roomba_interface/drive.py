#!/usr/bin/env python3
import serial
import time

# Open a serial connection to Roomba
ser = serial.Serial(port='/dev/ttyUSB0', baudrate=115200)

# Drive
ser.write(bytes([137,0,100,128,0]))
time.sleep(1)
ser.write(bytes([137,0,0,0,0]))

# Close the serial port; we're done for now.
ser.close()

