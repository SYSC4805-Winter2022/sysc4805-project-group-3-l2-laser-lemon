'''
SYSC 4805 
Group L2-3
'''

#Try to connect to coppeliasim python API library
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

global clientID

def setWheelVelocity(handle, velocity):
    return sim.simxSetJointTargetVelocity(clientID, handle, velocity, sim.simx_opmode_oneshot)

def setJointVelocity(handle, velocity):
    return sim.simxSetJointTargetVelocity(clientID, handle, velocity, sim.simx_opmode_oneshot)

def getObjectOrientation(obj_name, relative_obj_name):
    return sim.simxGetObjectOrientation(clientID, obj_name, relative_obj_name,sim.simx_opmode_blocking)

def getObjectHandle(obj_name):
    return sim.simxGetObjectHandle(clientID, obj_name, sim.simx_opmode_blocking)

def startSimulation():
    print ('Program started')
    sim.simxFinish(-1)
    return sim.simxStart('127.0.0.1',19999,True,True,5000,5)

def onOpen():
    open = True
    leftPanelBool = True
    rightPanelBool = True
    # print("Beginning to Open Plow!")

def onClose():
    close = True
    leftPanelBool = True
    rightPanelBool = True
    # print("Beginning to Close Plow!")


    

if __name__ == "__main__":

    clientID = startSimulation()
    
    if clientID!=-1:
        print ('Connected to remote API server')
        sim.simxAddStatusbarMessage(clientID,'Python Script Connected.',sim.simx_opmode_oneshot)

        #Get wheel and proximity sensor ObjectHandles
        #res,model = getObjectHandle('robot')

        res, left_joint  = getObjectHandle('left_joint1')
        res, right_joint = getObjectHandle('right_joint1')

        res, leftHinge  = getObjectHandle('plow_left_hinge')
        res, rightHinge = getObjectHandle('plow_right_hinge')

        res, rightPanel = getObjectHandle('plow_right_panel')
        res, leftPanel = getObjectHandle('plow_left_panel')
        res, middlePanel = getObjectHandle('plow_middle_panel')

        res, prox_sensor_front= getObjectHandle('Proximity_sensor0')
        res, prox_sensor_right = getObjectHandle('Proximity_sensor1')
        res, prox_sensor_back = getObjectHandle('Proximity_sensor2')
        res, prox_sensor_left= getObjectHandle('Proximity_sensor3')
        

        open = False
        close = False

        rightPanelBool = False
        leftPanelBool = False
        #res, line_sensor = getObjectHandle('Line_Sensor')

        # OPEN CODE PORTION ----/:
        res = setJointVelocity(rightHinge, -0.5)
        while (getObjectOrientation(rightPanel,middlePanel)[1][2] * 57.2957795 < 135.0):
            print("")
        res = setJointVelocity(rightHinge, 0)

        res = setJointVelocity(leftHinge, 0.5)
        while (getObjectOrientation(leftPanel,middlePanel)[1][2] * 57.2957795 > -135.0):
            print("")
        res = setJointVelocity(leftHinge, 0)
        #                      :\-------

        open = True



        # # CLOSE CODE PORTION ----/:
        # res = setJointVelocity(rightHinge, 0.5)
        # while (getObjectOrientation(rightPanel,middlePanel)[1][2] * 57.2957795 > 2.0):
        #     print("")
        # res = setJointVelocity(rightHinge, 0)

        # res = setJointVelocity(leftHinge, -0.5)
        # while (getObjectOrientation(leftPanel,middlePanel)[1][2] * 57.2957795 < -2):
        #     print("")
        # res = setJointVelocity(leftHinge, 0)
        # #                      :\-------



        #rightPanelBool = False

        while True:

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
                if open:
                    # CLOSE CODE PORTION ----/:
                    res = setJointVelocity(rightHinge, 1.5)
                    while (getObjectOrientation(rightPanel,middlePanel)[1][2] * 57.2957795 > 2.0):
                        print("")
                    res = setJointVelocity(rightHinge, 0)

                    res = setJointVelocity(leftHinge, -1.5)
                    while (getObjectOrientation(leftPanel,middlePanel)[1][2] * 57.2957795 < -2):
                        print("")
                    res = setJointVelocity(leftHinge, 0)
                    #                      :\-------
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
                if (not open):
                    res = setJointVelocity(rightHinge, -1.5)
                    while (getObjectOrientation(rightPanel,middlePanel)[1][2] * 57.2957795 < 135.0):
                        print("")
                    res = setJointVelocity(rightHinge, 0)

                    res = setJointVelocity(leftHinge, 1.5)
                    while (getObjectOrientation(leftPanel,middlePanel)[1][2] * 57.2957795 > -135.0):
                        print("")
                    res = setJointVelocity(leftHinge, 0)
                    open = True
                #                      :\-------
                
                res = setWheelVelocity(left_joint, 200*math.pi/-180)
                res = setWheelVelocity(right_joint, 200*math.pi/-180)   
            
            

        sim.simxGetPingTime(clientID)
        sim.simxFinish(clientID)
    else:
        print ('Failed connecting to remote API server')
    print ('Program ended')