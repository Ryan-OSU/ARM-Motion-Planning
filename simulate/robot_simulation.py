import sys
import numpy as np
import pybullet as sim
import pybullet_data
import time
import numpy

class robot_simulate:

    def __init__(self,arm_length):
        self.arm_length = arm_length

    def target_position(self,x,y):
        scale = 0.1

        turning_angle = self.inverse_kinematics(x,y,self.arm_length[0],self.arm_length[1])
        joint_end = []
        joint_end.append(turning_angle[0]);  # -90 to 90 degrees
        joint_end.append((30)*scale);                # 10 to 29.95 cm
        joint_end.append(turning_angle[1]);  # -90 to 90 degrees
        joint_end.append((-45)*numpy.pi/180);  # -90 to 90 degrees
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

    def simulate(self,robot,joint1_start,joint2_start,joint3_start,joint4_start,join_end):
        
        sim.setJointMotorControlArray(robot, [1, 2, 3, 4], controlMode = sim.POSITION_CONTROL,
                                targetPositions = [joint1_start, joint2_start, joint3_start, joint4_start])
        time.sleep(3)

        j1_temp = joint1_start
        j2_temp = joint2_start
        j3_temp = joint3_start
        j4_temp = joint4_start

        speed1 = .1       # rad/s
        speed2 = .1       # cm/s
        speed3 = .1       # rad/s
        speed4 = .1       # rad/s

        joint1_end = join_end[0]
        joint2_end = join_end[1]
        joint3_end = join_end[2]
        joint4_end = join_end[3]


        step = 0.01    
        step1 = (joint1_end - joint1_start) * step * speed1
        step2 = (joint2_end - joint2_start) * step * speed2
        step3 = (joint3_end - joint3_start) * step * speed3
        step4 = (joint4_end - joint4_start) * step * speed4

        stop1 = False
        stop2 = False
        stop3 = False
        stop4 = False
        
        for i in range(1000):
            if j1_temp < (joint1_end + step1) and j1_temp > (joint1_end - step1) and step1 > 0:
                stop1 = True
            elif j1_temp > (joint1_end + step1) and j1_temp < (joint1_end - step1) and step1 < 0:
                stop1 = True
            if j2_temp < (joint2_end + step2) and j2_temp > (joint2_end - step2) and step2 > 0:
                stop2 = True
            elif j2_temp > (joint2_end + step2) and j2_temp < (joint2_end - step2) and step2 < 0:
                stop2 = True
            if j3_temp < (joint3_end + step3) and j3_temp > (joint3_end - step3) and step3 > 0:
                stop3 = True
            elif j3_temp > (joint3_end + step3) and j3_temp < (joint3_end - step3) and step3 < 0:
                stop3 = True
            if j4_temp < (joint4_end + step4) and j4_temp > (joint4_end - step4) and step4 > 0:
                stop4 = True
            elif j4_temp > (joint4_end + step4) and j4_temp < (joint4_end - step4) and step4 < 0:
                stop4 = True
            
            if stop1 == False:  
                j1_temp = j1_temp + step1
            if stop2 == False:  
                j2_temp = j2_temp + step2
            if stop3 == False:  
                j3_temp = j3_temp + step3
            if stop4 == False:  
                j4_temp = j4_temp + step4
            
        #if stop1 and stop2 and stop3 and stop4:
        #   break
            sim.setJointMotorControlArray(robot, [1, 2, 3, 4], controlMode = sim.POSITION_CONTROL,
                                targetPositions = [j1_temp, j2_temp, j3_temp, j4_temp])
            time.sleep(step)
