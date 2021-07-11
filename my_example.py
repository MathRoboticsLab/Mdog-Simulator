#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sun Mar 15 19:51:40 2020

@author: linux-asd
"""

import pybullet as p
import numpy as np
import time
import math
import pybullet_data
from pybullet_debuger import pybulletDebug  
from kinematic_model import robotKinematics
from gaitPlanner import trotGait
from Q_learning import Q_Learning
from Q_function import Q_function
q=Q_Learning()
_q=Q_function()

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.8)
fall=False

cubeStartPos = [0,0,0.2]
FixedBase = False #if fixed no plane is imported
if (FixedBase == False):
    p.loadURDF("plane.urdf")
boxId = p.loadURDF("4leggedRobot.urdf",cubeStartPos, useFixedBase=FixedBase)

jointIds = []
paramIds = [] 
time.sleep(0.5)
for j in range(p.getNumJoints(boxId)):
#    p.changeDynamics(boxId, j, linearDamping=0, angularDamping=0)
    info = p.getJointInfo(boxId, j)
    print(info)
    jointName = info[1]
    jointType = info[2]
    jointIds.append(j)
    
footFR_index = 3
footFL_index = 7
footBR_index = 11
footBL_index = 15   

pybulletDebug = pybulletDebug()
robotKinematics = robotKinematics()
trot = trotGait() 

#robot properties
"""initial foot position"""
#foot separation (Ydist = 0.16 -> tetta=0) and distance to floor
Xdist = 0.20
Ydist = 0.15
height = 0.15
#body frame to foot frame vector
bodytoFeet0 = np.matrix([[ Xdist/2 , -Ydist/2 , -height],
                         [ Xdist/2 ,  Ydist/2 , -height],
                         [-Xdist/2 , -Ydist/2 , -height],
                         [-Xdist/2 ,  Ydist/2 , -height]])

T = 0.5 #period of time (in seconds) of every step
offset = np.array([0.5 , 0. , 0. , 0.5]) #defines the offset between each foot step in this order (FR,FL,BR,BL)

p.setRealTimeSimulation(1)
count=1 


def quaternionToEuler(q):
    q0,q1,q2,q3=q[0],q[1],q[2],q[3]
    phi=math.atan(2*(q2*q3-q0*q1)/(q0*q0-q1*q1-q2*q2-q3*q3))
    theda=math.asin(-2*(q0*q2+q1*q3))
    psi=math.atan(2*(q1*q2-q0*q3)/(q0*q0+q1*q1-q2*q2-q3*q3))
    return [psi,theda,phi]



  
while(True):
    lastTime = time.time()
    pos , orn , L , angle , Lrot , T = pybulletDebug.cam_and_robotstates(boxId)  
    #calculates the feet coord for gait, defining length of the step and direction (0º -> forward; 180º -> backward)
    bodytoFeet , s = trot.loop(L , angle , Lrot , T , offset , bodytoFeet0)

#####################################################################################
#####   kinematics Model: Input body orientation, deviation and foot position    ####
#####   and get the angles, neccesary to reach that position, for every joint    ####
    FR_angles, FL_angles, BR_angles, BL_angles , transformedBodytoFeet = robotKinematics.solve(orn , pos , bodytoFeet) 
    position=p.getBasePositionAndOrientation(boxId)[0]
    FR_position=p.getLinkState(boxId,3)[0]
    FL_position=p.getLinkState(boxId,7)[0]
    BR_position=p.getLinkState(boxId,11)[0]
    BL_position=p.getLinkState(boxId,15)[0]
    if (position[2]<FL_position[2]) and (position[2]<FR_position[2]) and (position[2]<BL_position[2]) and (position[2]<BR_position[2]): fall=True
    else: fall=False
    
    #print(fall)
    #print("{:.3f}".format(position[2]))
    #print("{:.3f}".format(FL_position[2]))
    _q.learning(FR_angles,FL_angles,BR_angles,BL_angles,position)
    rotation=quaternionToEuler(p.getBasePositionAndOrientation(boxId)[1])
    #print("{:.3f}".format(rotation[0]),"{:.3f}".format(rotation[1]),"{:.3f}".format(rotation[2]))
    #print("{:.3f}".format(position[0]),"{:.3f}".format(position[1]),"{:.3f}".format(position[2]))
    #print("{:.3f}".format(p.getLinkState(boxId,7)[0][0]),"{:.3f}".format(p.getLinkState(boxId,7)[1][0]),"{:.3f}".format(p.getLinkState(boxId,7)[0][2]))
    FR_angles, FL_angles, BR_angles, BL_angles=_q.getState()
    _q.update(position)
    #print("iteration:",count)
    count+=1 
    #move movable joints
    for i in range(0, footFR_index):
        p.setJointMotorControl2(boxId, i, p.POSITION_CONTROL, FR_angles[i - footFR_index])
    for i in range(footFR_index + 1, footFL_index):
        p.setJointMotorControl2(boxId, i, p.POSITION_CONTROL, FL_angles[i - footFL_index])
    for i in range(footFL_index + 1, footBR_index):
        p.setJointMotorControl2(boxId, i, p.POSITION_CONTROL, BR_angles[i - footBR_index])
    for i in range(footBR_index + 1, footBL_index):
        p.setJointMotorControl2(boxId, i, p.POSITION_CONTROL, BL_angles[i - footBL_index])
    
    if fall==True:
        p.resetSimulation(boxId)
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
        p.setGravity(0,0,-9.8)
        fall=False

        cubeStartPos = [0,0,0.2]
        FixedBase = False #if fixed no plane is imported
        if (FixedBase == False):
            p.loadURDF("plane.urdf")
        boxId = p.loadURDF("4leggedRobot.urdf",cubeStartPos, useFixedBase=FixedBase)
        _q.nowState= [q.startS1,q.startS2,q.startS1,q.startS2,q.startS1,q.startS2,q.startS1,q.startS2]
        #print(q.nowState)
        #print(q.reward[q.nowState[0]%q.scl][q.nowState[1]%q.scl][q.nowState[2]%q.scl][q.nowState[3]%q.scl][q.nowState[4]%q.scl][q.nowState[5]%q.scl][q.nowState[6]%q.scl][q.nowState[7]%q.scl])
        time.sleep(1)
    #if count==3: break
        
#    print(time.time() - lastTime)
p.disconnect()
