#############
# Split all functionality from plow_and_obstacle_avoidance.py into separate callable functions
# All code written by Chhavi, organized by Denise
#############
import os

try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')
    

import time as t
import math

#helper functions
def setWheelVelocity(clientID, handle, velocity):
    return sim.simxSetJointTargetVelocity(clientID, handle, velocity, sim.simx_opmode_oneshot)

def setJointVelocity(clientID, handle, velocity):
    return sim.simxSetJointTargetVelocity(clientID, handle, velocity, sim.simx_opmode_oneshot)

def getObjectOrientation(clientID, obj_name, relative_obj_name):
    return sim.simxGetObjectOrientation(clientID, obj_name, relative_obj_name,sim.simx_opmode_blocking)

def getObjectHandle(clientID, obj_name):
    return sim.simxGetObjectHandle(clientID, obj_name, sim.simx_opmode_blocking)

def startSimulation():
    print ('Program started')
    sim.simxFinish(-1)
    return sim.simxStart('127.0.0.1',19997,True,True,5000,5)

def onOpen(clientID):
    # print("Beginning to Open Plow!")
    inputIntegers = []
    inputFloats = []
    inputStrings = []
    inputBuffer = bytearray()
    sim.simxCallScriptFunction(clientID, 'plow_middle_panel', sim.sim_scripttype_childscript, 'onOpen',
                                inputIntegers, inputFloats, inputStrings, inputBuffer, sim.simx_opmode_blocking)
    opening = True
    while opening:
        errorFlag, signalFlag = sim.simxGetStringSignal(clientID, 'openFlag', sim.simx_opmode_blocking)
        flag = signalFlag.decode('utf-8')
        t.sleep(0.15)
        if (flag == "1"):
            opening = False
        
    print("Plow is opened!")
    return True

def onClose(clientID):
    # print("Beginning to Close Plow!")
    inputIntegers = []
    inputFloats = []
    inputStrings = []
    inputBuffer = bytearray()
    sim.simxCallScriptFunction(clientID, 'plow_middle_panel', sim.sim_scripttype_childscript, 'onClose',
                                inputIntegers, inputFloats, inputStrings, inputBuffer, sim.simx_opmode_blocking)

    closing = True
    while closing:
        errorFlage, signalFlag = sim.simxGetStringSignal(clientID, 'closeFlag', sim.simx_opmode_blocking)
        flag = signalFlag.decode('utf-8')
        print(flag)
        t.sleep(0.15)
        if (flag == "1"):
            closing = False
    print("Plow is closed!")
    return False

def moveRobot(clientID, isOpen): #still need to tweak it to integrate
    res, left_joint  = getObjectHandle('left_joint1')
    res, right_joint = getObjectHandle('right_joint1')

    res, leftHinge  = getObjectHandle('plow_left_hinge')
    res, rightHinge = getObjectHandle('plow_right_hinge')

    res, prox_sensor_front= getObjectHandle('Proximity_sensor0')
    res, prox_sensor_right = getObjectHandle('Proximity_sensor1')
    res, prox_sensor_back = getObjectHandle('Proximity_sensor2')
    res, prox_sensor_left= getObjectHandle('Proximity_sensor3')

    res = sim.simxGetObjectGroupData(clientID, 5, 13, sim.simx_opmode_blocking)
    returnCode, detectionState = sim.simxReadProximitySensor(clientID, prox_sensor_front, sim.simx_opmode_streaming)[0:2] #simx_opmode_streaming
    returnCode, detectrightSide = sim.simxReadProximitySensor(clientID, prox_sensor_right, sim.simx_opmode_streaming)[0:2] #simx_opmode_streaming
    returnCode, detectleftSide = sim.simxReadProximitySensor(clientID, prox_sensor_left, sim.simx_opmode_streaming)[0:2] #simx_opmode_streaming
    returnCode, detectBack = sim.simxReadProximitySensor(clientID, prox_sensor_back, sim.simx_opmode_streaming)[0:2] #simx_opmode_streaming

    if detectionState:
    #Make Bot Turn #turning left
        #onClose()

        res = setWheelVelocity(left_joint, 0)
        res = setWheelVelocity(right_joint, 0)
        if isOpen:
            # CLOSE CODE PORTION ----/:
            onClose()
            open = False
        print("turning")
        res = setWheelVelocity(left_joint, 50*math.pi/180)
        res = setWheelVelocity(right_joint, -50*math.pi/180)
        t.sleep(1)

    #if right sensor detects an obstacle, robot should turn left
    elif detectrightSide:
        #Make Bot Turn #turning left
        print("turning")
        res = setWheelVelocity(left_joint, 100*math.pi/180)
        res = setWheelVelocity(right_joint, 50*math.pi/180)
        t.sleep(1)

    elif detectleftSide:
        #Make Bot Turn #turning right
        print("turning")
        res = setWheelVelocity(left_joint, 50*math.pi/180)
        res = setWheelVelocity(right_joint, 100*math.pi/180)
        t.sleep(1)

    elif detectBack:
        #Make Bot Move straight
        res = setWheelVelocity(left_joint, 200*math.pi/-180)
        res = setWheelVelocity(right_joint, 200*math.pi/-180)
    else:
    #Make Bot Move straight
        #onOpen()

        # LOGIC FROM EMMA'S LUA (JUST LEAVE IT)
        # if open:
        #     if rightPanel:
        #         res = setJointVelocity(rightHinge, -0.5)
        #     if leftPanel:
        #         res = setJointVelocity(leftHinge, 0.5)
        #     rotRight = getObjectOrientation(rightPanel,middlePanel)[3] * 57.2957795
        #     if rotRight > 135.0:
        #         res = setJointVelocity(rightHinge, 0)
        #         rightPanelBool = False
        #     rotLeft = getObjectOrientation(leftPanel,middlePanel)[3]*57.2957795
        #     if rotLeft < -135.0:
        #         res = setJointVelocity(leftHinge,0)
        #         leftPanelBool = False
        #     if (not leftPanelBool) and (not rightPanelBool):
        #         open = False
        #         print("Plow finished Opening")
        # OPEN CODE PORTION ----/:

        res = setWheelVelocity(left_joint, 0)
        res = setWheelVelocity(right_joint, 0)
        if (not isOpen):
            isOpen = onOpen()
        #                      :\-------

        res = setWheelVelocity(left_joint, 200*math.pi/-180)
        res = setWheelVelocity(right_joint, 200*math.pi/-180) 