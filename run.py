import pybullet as sim
import pybullet_data
import time
import numpy
import kinematics as K
import continueRun as con

# setup
scale = 0.1
physicsClient = sim.connect(sim.GUI)
sim.setAdditionalSearchPath(pybullet_data.getDataPath())
plane = sim.loadURDF("plane.urdf")
robot = sim.loadURDF("scara_robot.urdf",[0,0,0], globalScaling=scale)
sim.setGravity(0,0,-9.8)
sim.setRealTimeSimulation(True, physicsClient)

# find end positions
arm1 = 16.965
arm2 = 14.565
x = int(input("Enter target x position \n"))
y = int(input("ENter target y position \n"))
joint_end = []
joint_end.append((-90)*numpy.pi/180);  # -90 to 90 degrees
joint_end.append((10)*scale);                # 10 to 29.95 cm
joint_end.append((90)*numpy.pi/180);  # -90 to 90 degrees
joint_end.append(45*numpy.pi/180);  # -90 to 90 degrees

stopornot = input("Do you wish to continue \n")

while stopornot != "no":
    joint1_start = joint_end[0]
    joint2_start = joint_end[1]  
    joint3_start = joint_end[2]
    joint4_start = joint_end[3]
    joint_end = con.target_position(x,y)
    con.simulate(robot,joint1_start,joint2_start,joint3_start,joint4_start,joint_end)
    stopornot = input("Do you wish to continue \n")
    if stopornot == "no":
        break
    x = int(input("Enter target x position \n"))
    y = int(input("ENter target y position \n"))