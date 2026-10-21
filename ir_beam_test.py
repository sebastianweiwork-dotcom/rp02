import RPi.GPIO as GPIO
import time

SENSOR = 17

GPIO.setmode(GPIO.BCM)
GPIO.setup(SENSOR, GPIO.IN)

while True:
    if GPIO.input(SENSOR):
        print("光路通")
    else:
        print("被遮挡")
    time.sleep(0.1)
