import RPi.GPIO as GPIO
from time import sleep

# Direction pin from controller
DIR1 = 10
DIR2 = 10
DIR3 = 10
DIR4 = 10
# Step pin from controller
STEP1 = 8
STEP2 = 8
STEP3 = 8
STEP4 = 8

# 0/1 used to signify clockwise or counterclockwise.
CW = 1
CCW = 0

# Setup pin layout on PI
GPIO.setmode(GPIO.BOARD)

# Establish Pins in software
GPIO.setup(DIR1, GPIO.OUT)
GPIO.setup(STEP1, GPIO.OUT)
GPIO.setup(DIR2, GPIO.OUT)
GPIO.setup(STEP2, GPIO.OUT)
GPIO.setup(DIR3, GPIO.OUT)
GPIO.setup(STEP3, GPIO.OUT)
GPIO.setup(DIR4, GPIO.OUT)
GPIO.setup(STEP4, GPIO.OUT)

angle1 = 45   # degrees
angle3 = 45
angle4 = 45 
height = 10   # cm

steps1 = abs(20*angle1/360*200)
steps2 = abs(height*1000)
steps3 = abs(16*angle3/360*200)
steps4 = abs(4*angle4/360*200)


try:
	while True:
		sleep(0.5)

		if angle1 > 0:
			GPIO.output(DIR1,CW)
		else:
			GPIO.output(DIR1,CCW)
	
		if height > 0:
			GPIO.output(DIR2,CW)
		else:
			GPIO.output(DIR2,CCW)

		if angle3> 0:
			GPIO.output(DIR3,CW)
		else:
			GPIO.output(DIR3,CCW)

		if angle4 > 0:
			GPIO.output(DIR4,CW)
		else:
			GPIO.output(DIR4,CCW)
	    
		
		for x in range(max[steps1, steps2, steps3, steps4]):
			if x < steps1:
				GPIO.output(STEP1,GPIO.HIGH) 

			if x < steps2:
				GPIO.output(STEP2,GPIO.HIGH)
			        
			if x < steps3:
				GPIO.output(STEP3,GPIO.HIGH)  
			    
			if x < steps4:
				GPIO.output(STEP4,GPIO.HIGH)
			    
			sleep(.005) 
		
			if x < steps1:
				GPIO.output(STEP1,GPIO.LOW)
			if x < steps2:
				GPIO.output(STEP2,GPIO.LOW)    
			if x < steps3:
				GPIO.output(STEP3,GPIO.LOW)
			if x < steps4:
				GPIO.output(STEP4,GPIO.LOW)
			    
			sleep(.005) 
			 
	
except KeyboardInterrupt:
	print("cleanup")
	GPIO.cleanup()