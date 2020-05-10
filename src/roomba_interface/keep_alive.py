#!/usr/bin/env python3
import RPi.GPIO as GPIO
from time import sleep

GPIO.setmode(GPIO.BOARD)   # BCM for GPIO numbering  
GPIO.setup(11, GPIO.OUT) # Make pin 11 (which is hooked up to the BRC pin) an output

while True:
    # Pulse the BRC pin at a low duty cycle to keep Roomba awake.
    GPIO.output(11, False)
    sleep(1)
    GPIO.output(11, True)
    sleep(59)
