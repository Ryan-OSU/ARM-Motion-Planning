import RPi.GPIO as GPIO
import time
import numpy

# Set the GPIO mode to BOARD
GPIO.setmode(GPIO.BOARD)

# Current position of each joint 
J1_pos = 0
J3_pos = 0
J4_pos = 0
z_pos = 0

object_height = 3 
bin_height = 4 
bin_thickness = 0.5 
clearance = 1 

z_home = object_height + bin_height + clearance
z_pick = object_height
z_pack = object_height + bin_thickness

# Define a function to move the stepper motor
def move_object(objx,objy,objphi,packx,packy,packphi):
    # move above object (x-y)
    # lower down (z)
    # suction on
    # raise up (z)
    # move above packing location (x-y)
    # lower object 
    # suction off
    # raise straight up

def suction(state):
    # GPIO numbers
    vacuum = 0
    solenoid = 0
    
    GPIO.setup(vacuum, GPIO.OUT)
    GPIO.setup(solenoid, GPIO.OUT)
    if state == 1:
       GPIO.output(vacuum, GPIO.HIGH)
       GPIO.output(solenoid, GPIO.HIGH)
    elif state == 0:
       GPIO.output(vacuum, GPIO.LOW)
       GPIO.output(solenoid, GPIO.LOW)



def home_robot():
   # Steps to home angle of each joint
   J1_home_steps = 0
   J3_home_steps = 0
   J4_home_steps = 0
   z_home_steps = 0

   step_z(99999, 1, 16, 18, 400)
   step_z(z_home_steps, 0, 16, 18, 400)

   step_xy(99999, 1, 0, 0, 99999, 1, 0, 0, 99999, 1, 0, 0, 400)
   step_xy(J1_home_steps, 0, 0, 0, J3_home_steps, 0, 0, 0, J4_home_steps, 0, 0, 0, 400)

   J1_pos = 0
   J3_pos = 0
   J4_pos = 0
   z_pos = z_home


def step_xy(nstep1, dir1, dp1, sp1, nstep3, dir3, dp3, sp3, nstep4, dir4, dp4, sp4, sps):

    GPIO.setup(sp1, GPIO.OUT)
    GPIO.setup(dp1, GPIO.OUT)
    GPIO.setup(sp3, GPIO.OUT)
    GPIO.setup(dp3, GPIO.OUT)
    GPIO.setup(sp4, GPIO.OUT)
    GPIO.setup(dp4, GPIO.OUT)
    GPIO.output(dp1, dir1)
    GPIO.output(dp3, dir3)
    GPIO.output(dp4, dir4) 

    stepd = 1/sps
    for x in range (max(2*nstep1, 2*nstep3, 2*nstep4)):
        if x < 2*nstep1 and not limSwitch1:
            GPIO.output(sp1, not GPIO.input(sp1))
        if x < 2*nstep3 and not limSwitch3:
            GPIO.output(sp3, not GPIO.input(sp3))
        if x < 2*nstep4 and not limSwitch4:
            GPIO.output(sp4, not GPIO.input(sp4))
        time.sleep(stepd)

def step_z(nstep2, dir2, dp2, sp2, sps2):
    GPIO.setup(sp2, GPIO.OUT)
    GPIO.setup(dp2, GPIO.OUT)
    GPIO.output(dp2, dir2)

    stepd2 = 1/sps2
    for x in range(2*nstep2):
        while not limSwitch2
            GPIO.output(sp2, not GPIO.input(sp2))
            time.sleep(stepd2)

# Cleanup the GPIO pins
GPIO.cleanup()
