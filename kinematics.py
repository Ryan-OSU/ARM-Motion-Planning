import sys
import manipulator
from matplotlib import animation
import numpy as np
import math as M
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import pybullet as p
import pybullet_data
import time


def inverse_kinematics(target_x, target_y, L1, L2):
    angle = []
    th2 = np.arccos((target_x**2 + target_y**2 - L1**2 - L2**2)/(2*L1*L2))
    th1 = np.arctan(target_y/target_x)-np.arcsin(L2*np.sin(th2)/np.sqrt(target_x**2+target_y**2))
    angle.append(th1)
    angle.append(th2)
    return angle

def forward_kinematics(angle, L1, L2):
    position = []
    x = np.cos(angle[0])*L1+np.cos(angle[0] + angle[1])*L2
    y = np.sin(angle[0])*L1+np.sin(angle[0] + angle[1])*L2
    position.append(x)
    position.append(y)
    return position


def main():
    arm1 = 4
    arm2 = 3
    x = 2.5
    y = 2

    turning_angle = inverse_kinematics(x,y,arm1,arm2)
    print (turning_angle)
    target = forward_kinematics(turning_angle,arm1,arm2)
    print (target)

    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    plane = p.loadURDF("plane.urdf")
    robot = p.loadURDF("scara_robot.urdf")
    p.setGravity(0,0,-10)
    for i in range(p.getNumJoints(robot)):
        print(p.getJointInfo(robot, i))

    while True:
        target_prismatic = 5.5
        target_endEffector = 0
        p.setJointMotorControlArray(robot, [1,2,3,4], p.POSITION_CONTROL, targetPositions=[turning_angle[0], target_prismatic, turning_angle[1], target_endEffector])
        p.stepSimulation()
        time.sleep(1./240)

if __name__ == '__main__':
    sys.exit(main())