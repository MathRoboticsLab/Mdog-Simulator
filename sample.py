import pybullet as p
import numpy as np
import time
import pybullet_data
from pybullet_debuger import pybulletDebug  
from kinematic_model import robotKinematics
from gaitPlanner import trotGait
import sys, select, termios, tty
import math
##### monitor input function #####
def getKey():
    settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())
    rlist, wlist, xlist = select.select([sys.stdin], [], [], 0.05)
    if(rlist):
        key = sys.stdin.read(1)
    else:
        key = ""
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def quaternionToEuler(q):
    q0, q1, q2, q3 = q[0], q[1], q[2], q[3]
    phi = math.atan(2 * (q2 * q3 - q0 * q1) / (q0 * q0 - q1 * q1 - q2 * q2 - q3 * q3))
    theda = math.asin(-2 * (q0 * q2 + q1 * q3))
    psi = math.atan(2 * (q1 * q2 - q0 * q3) / (q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3))
    return [psi, theda, phi] # yaw, pitch, roll

def motorControl(footFR_index, p, FR_angles, FL_angles, BR_angles, BL_angles):
    for i in range(0, footFR_index):
        p.setJointMotorControl2(boxId, i, p.POSITION_CONTROL, FR_angles[i - footFR_index])
    for i in range(footFR_index + 1, footFL_index):
        p.setJointMotorControl2(boxId, i, p.POSITION_CONTROL, FL_angles[i - footFL_index])
    for i in range(footFL_index + 1, footBR_index):
        p.setJointMotorControl2(boxId, i, p.POSITION_CONTROL, BR_angles[i - footBR_index])
    for i in range(footBR_index + 1, footBL_index):
        p.setJointMotorControl2(boxId, i, p.POSITION_CONTROL, BL_angles[i - footBL_index])

##### initialize section #####
physicsClient = p.connect(p.GUI) # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.8)


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

pos , orn , L , angle , Lrot , T = pybulletDebug.cam_and_robotstates(boxId)  
bodytoFeet , s = trot.loop(L , angle , Lrot , T , offset , bodytoFeet0)

FR_angles, FL_angles, BR_angles, BL_angles , transformedBodytoFeet = robotKinematics.solve(orn , pos , bodytoFeet)


# move movable joints
motorControl(footFR_index, p, FR_angles, FL_angles, BR_angles, BL_angles)

startTime = time.time() # record start time
##### initial section end #####

# main function
while(True):
    # key control: you can use keyboard to control the dog
    motion = getKey()
    if(motion == "c"):
        sys.exit()
    if(motion != ""):
        print "now motion:", motion
    if(motion == "i"):
        L = 0.5
    elif(motion == ","):
        L = -0.5
    if(motion == "j"):
        Lrot = 0.25
    elif(motion == "l"):
        Lrot = -0.25
    if(motion == "k"):
        L = 0
        Lrot = 0
    
    bodytoFeet , s = trot.loop(L , angle , Lrot , T , offset , bodytoFeet0)

    #####################################################################################
    ####   kinematics Model: Input body orientation, deviation and foot position    ####
    ####   and get the angles, neccesary to reach that position, for every joint    ####
    FR_angles, FL_angles, BR_angles, BL_angles , transformedBodytoFeet = robotKinematics.solve(orn , pos , bodytoFeet)
    data = []
    for i in range(4):
        motor = []
        for j in range(3):
            position = p.getJointStates(boxId, [i * 4 + j])[0][0]
            velocity = p.getJointStates(boxId, [i * 4 + j])[0][1]
            motor.append([position, velocity])
        data.append(motor)
    orientation = quaternionToEuler(p.getBasePositionAndOrientation(boxId)[1])
    # motor data format: [shoulder1_position, shoulder1_velocity], [shoulder2_position, shoulder2_velocity], [leg_position, leg_velocity]  (radian format)
    print "The Right front leg radian: [{:.6f}, {:.6f}], [{:.6f}, {:.6f}], [{:.6f}, {:.6f}]".format(data[0][0][0], data[0][0][1], data[0][1][0], data[0][1][1], data[0][2][0], data[0][2][1])
    print "The Left front leg radian: [{:.6f}, {:.6f}], [{:.6f}, {:.6f}], [{:.6f}, {:.6f}]".format(data[1][0][0], data[1][0][1], data[1][1][0], data[1][1][1], data[1][2][0], data[1][2][1])
    print "The Right back leg radian: [{:.6f}, {:.6f}], [{:.6f}, {:.6f}], [{:.6f}, {:.6f}]".format(data[2][0][0], data[2][0][1], data[2][1][0], data[2][1][1], data[2][2][0], data[2][2][1])
    print "The Left back leg radian: [{:.6f}, {:.6f}], [{:.6f}, {:.6f}], [{:.6f}, {:.6f}]".format(data[3][0][0], data[3][0][1], data[3][1][0], data[3][1][1], data[3][2][0], data[3][2][1])
    print "The orientation info: yaw = {:.6f}, pitch = {:.6f}, row = {:.6f}".format(orientation[0], orientation[1], orientation[2])
    print

    # move movable joints
    motorControl(footFR_index, p, FR_angles, FL_angles, BR_angles, BL_angles)
    
    # print(time.time() - startTime)
p.disconnect()
