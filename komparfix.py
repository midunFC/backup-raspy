*-------*-*----------------------------------------------------------------import RPi.GPIO as GPIO
import time

pin = [21,20,16,12,1,7,8,25,24,23,6,5,0,11,9,10,22,27,17,4]
data = 650500


GPIO.setwarnings(False) 
GPIO.setmode(GPIO.BCM)
for x in pin:
    GPIO.setup(x,GPIO.OUT)

a= data

for y in pin:
    if a % 2 ==  1:    
        GPIO.output(y,True)

    else:
        GPIO.output(y,False)    
    a = a//2

