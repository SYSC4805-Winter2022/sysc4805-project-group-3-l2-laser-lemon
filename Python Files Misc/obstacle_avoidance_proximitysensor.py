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

def getObjectHandle(obj_name):
    return sim.simxGetObjectHandle(clientID, obj_name, sim.simx_opmode_blocking)

def startSimulation():
    print ('Program started')
    sim.simxFinish(-1)
    return sim.simxStart('127.0.0.1',19999,True,True,5000,5)


    
# main function - Might rename this later
#renamed and made callable by the main program, commented out some functions
def avoid_obstacle(clientID):

    #clientID = startSimulation()
    
    if clientID!=-1:
        #print ('Connected to remote API server')
        sim.simxAddStatusbarMessage(clientID,'Obstacle avoidance script initiated.',sim.simx_opmode_oneshot)

        #Get wheel and proximity sensor ObjectHandles
        #res,model = getObjectHandle('robot')
        res, left_joint  = getObjectHandle('left_joint1')
        res, right_joint = getObjectHandle('right_joint1')
        res, prox_sensor = getObjectHandle('Proximity_sensor0')
        #res, line_sensor = getObjectHandle('Line_Sensor')

        #while True:
        res = sim.simxGetObjectGroupData(clientID, 5, 13, sim.simx_opmode_blocking)
        returnCode,detectionState = sim.simxReadProximitySensor(clientID, prox_sensor, sim.simx_opmode_streaming)[0:2] #simx_opmode_streaming
        print(detectionState)
        if detectionState:
        #Make Bot Turn #turning left
            print("turning")
            res = setWheelVelocity(left_joint, 100*math.pi/180)
            res = setWheelVelocity(right_joint, 50*math.pi/180)
            t.sleep(2)
        else:
        #Make Bot Move straight
            res = setWheelVelocity(left_joint, 200*math.pi/-180)
            res = setWheelVelocity(right_joint, 200*math.pi/-180)    
        sim.simxGetPingTime(clientID)
        sim.simxFinish(clientID)
    else:
        print ('Failed connecting to remote API server')
    print ('Program ended')