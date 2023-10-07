import RPi.GPIO as GPIO
import time
DIR = 8
STEP = 9
GPIO.setmode(GPIO.BCM)
GPIO.setup(DIR, GPIO.OUT)
GPIO.setup(STEP, GPIO.OUT)
GPIO.output(DIR, GPIO.HIGH)
try:
    while True:
        if input() == '1':
            for i in range(200):
                GPIO.output(STEP, GPIO.HIGH)
                time.sleep(0.0008)
                GPIO.output(STEP, GPIO.LOW)
                time.sleep(0.001)
        elif input() == '2':
            GPIO.output(DIR, GPIO.LOW)
            for i in range(200):
                GPIO.output(STEP, GPIO.HIGH)
                time.sleep(0.001)
                GPIO.output(STEP, GPIO.LOW)
                time.sleep(0.001)
        time.sleep(0.5)
except KeyboardInterrupt:
    pass
GPIO.cleanup()
