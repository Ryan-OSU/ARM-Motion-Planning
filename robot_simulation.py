import sys
from time import sleep
import numpy as np
import RPi.GPIO as GPIO

class robot_simulate:

    def __init__(self,arm_length,CW,CCW,DIR,STEP):
        self.arm_length = arm_length
        self.CW = CW
        self.CCW = CCW
        self.DIR = DIR
        self.STEP = STEP

    def target_position(self,x,y):
        turning_angle = self.inverse_kinematics(x,y,self.arm_length[0],self.arm_length[1])
        joint_end = []
        joint_end.append(turning_angle[0]);  # -90 to 90 degrees
        joint_end.append((30)*0.1);                # 10 to 29.95 cm
        joint_end.append(turning_angle[1]);  # -90 to 90 degrees
        joint_end.append((-45)*np.pi/180);  # -90 to 90 degrees
        return joint_end       

    def inverse_kinematics(self, target_x, target_y, L1, L2):
        angle = []
        th2 = np.arccos((target_x**2 + target_y**2 - L1**2 - L2**2)/(2*L1*L2))
        th1 = np.arctan(target_y/target_x)-np.arcsin(L2*np.sin(th2)/np.sqrt(target_x**2+target_y**2))
        angle.append(th1)
        angle.append(th2)
        return angle

    def forward_kinematics(self, angle, L1, L2):
        position = []
        x = np.cos(angle[0])*L1+np.cos(angle[0] + angle[1])*L2
        y = np.sin(angle[0])*L1+np.sin(angle[0] + angle[1])*L2
        position.append(x)
        position.append(y)
        return position

    def simulate(self,angle1,height,angle3,angle4,target_position):
        CW = self.CW
        CCW = self.CCW
        DIR = self.DIR
        STEP = self.STEP
        steps1 = abs(20*(target_position[0]-angle1)/360*200)
        steps2 = abs((target_position[1]-height)*1000)
        steps3 = abs(16*(target_position[2]-angle3)/360*200)
        steps4 = abs(4*(target_position[3]-angle4)/360*200)
        # steps = [steps1,steps2,steps3,steps4]

        try:
            while True:
                sleep(0.5)

                if angle1 > 0:
                    GPIO.output(DIR[0],CW)
                else:
                    GPIO.output(DIR[0],CCW)
            
                if height > 0:
                    GPIO.output(DIR[1],CW)
                else:
                    GPIO.output(DIR[1],CCW)

                if angle3> 0:
                    GPIO.output(DIR[2],CW)
                else:
                    GPIO.output(DIR[2],CCW)

                if angle4 > 0:
                    GPIO.output(DIR[3],CW)
                else:
                    GPIO.output(DIR[3],CCW)
                
                max_step = max[steps1, steps2, steps3, steps4]
                for x in range(max[steps1, steps2, steps3, steps4]):
                    if x < steps1:
                        GPIO.output(STEP[0],GPIO.HIGH) 

                    if x < steps2:
                        GPIO.output(STEP[1],GPIO.HIGH)
                            
                    if x < steps3:
                        GPIO.output(STEP[2],GPIO.HIGH)  
                        
                    if x < steps4:
                        GPIO.output(STEP[3],GPIO.HIGH)
                        
                    sleep(.005) 
                
                    if x < steps1:
                        GPIO.output(STEP[0],GPIO.LOW)
                    if x < steps2:
                        GPIO.output(STEP[1],GPIO.LOW)    
                    if x < steps3:
                        GPIO.output(STEP[2],GPIO.LOW)
                    if x < steps4:
                        GPIO.output(STEP[3],GPIO.LOW)
                        
                    sleep(.005) 
                    
            
        except KeyboardInterrupt:
            print("cleanup")
            GPIO.cleanup()