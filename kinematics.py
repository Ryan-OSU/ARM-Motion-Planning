import sys
import numpy as np

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
