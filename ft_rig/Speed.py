import RPi.GPIO as GPIO
import time
import sys


input_pin = 29 #CHANGE THIS TO GPIO PORT

GPIO.setmode(GPIO.BOARD)

GPIO.setup(input_pin, GPIO.IN)

in_timer = False
time_lapsed, start_time, end_time = 0, 0, 0
circum =2*0.12*3.141 # ADD CIRCUMFERENCE

while True:
    value = GPIO.input(input_pin)
    
    if value == GPIO.LOW :
        if in_timer == False:
            start_time = time.time()
            time.sleep(0.5)
            in_timer = True
        elif in_timer == True:
            end_time = time.time()
            time_lapsed = end_time - start_time
            speed  = (circum / 12)   / time_lapsed
            print("Time Elapsed: " + str(time_lapsed))
            in_timer = False
            print(speed)
    if value == GPIO.HIGH:
        pass




    
