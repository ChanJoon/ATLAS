import RPi.GPIO as GPIO
import time

def stopSwitchCallback(channel):
	if GPIO.input(stopSwitchRed):
		print(f"ON {GPIO.input(stopSwitchRed)}")
	# elif GPIO.input(stopSwitchRed) == 0:
		# print(f"Hello I'm {GPIO.input(stopSwitchRed)}")


GPIO.setmode(GPIO.BCM)

stopSwitchRed = 23
GPIO.setup(stopSwitchRed, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.add_event_detect(stopSwitchRed, GPIO.BOTH, callback=stopSwitchCallback)

try:
	while True:
		time.sleep(5)
except Exception as e:
	print(f"{e}")
	pass

GPIO.cleanup()
