

import RPi.GPIO as GPIO
import numpy as np
import time

ERR_PIN = 6
RUN_PIN = 5
RDY_PIN = 25
REC_PIN = 22
BTN_PIN = 12

GPIO.cleanup()

GPIO.setmode(GPIO.BCM)
GPIO.setup(ERR_PIN, GPIO.OUT)
GPIO.setup(RUN_PIN, GPIO.OUT)
GPIO.setup(RDY_PIN, GPIO.OUT)
GPIO.setup(REC_PIN, GPIO.OUT)
GPIO.setup(BTN_PIN, GPIO.IN)

mode = np.array([0, 1, 2, 3])

GPIO.output(ERR_PIN, GPIO.HIGH)
GPIO.output(RUN_PIN, GPIO.HIGH)
GPIO.output(RDY_PIN, GPIO.HIGH)
GPIO.output(REC_PIN, GPIO.HIGH)

while True:
    if GPIO.input(BTN_PIN):
        mode = (mode+1) % 4
        GPIO.output(ERR_PIN, not int(mode[0]))
        GPIO.output(RUN_PIN, not int(mode[1]))
        GPIO.output(RDY_PIN, not int(mode[2]))
        GPIO.output(REC_PIN, not int(mode[3]))
    time.sleep(0.2)

GPIO.cleanup()
