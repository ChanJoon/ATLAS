import RPi.GPIO as GPIO
from Encoder import Encoder
import time

def measure(value, direction):
	print(f"Value: {value}, Direction: {direction}")

# Encoder
EncoderA = 2    # White
EncoderB = 3    # Orange
GPIO.setmode(GPIO.BCM)

enc = Encoder(EncoderA, EncoderB, measure)

try:
	while True:
		time.sleep(5)
except Exception:
  pass
  
GPIO.cleanup()
