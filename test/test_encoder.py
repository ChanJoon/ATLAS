import RPi.GPIO as GPIO
import time
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
from encoder import Encoder

def measure(value, direction):
	print(f"Value: {value}, Direction: {direction}")

# Encoder
EncoderA = 17    # White
EncoderB = 22    # Orange
# EncoderA = 24    # White
# EncoderB = 25    # Orange
GPIO.setmode(GPIO.BCM)

enc = Encoder(EncoderA, EncoderB, measure)

try:
	while True:
		time.sleep(5)
except Exception:
  pass
  
GPIO.cleanup()
