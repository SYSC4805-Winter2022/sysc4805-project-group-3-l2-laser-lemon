'''
SYSC 4805 
Group L2-3
'''


import sim
import time as t
import math

class ObstacleAvoidance:
    def __init__(self, clientId):
        res, left_joint  = sim.simxGetObjectHandle(clientId, 'left_joint1', sim.simx_opmode_blocking)
        res, right_joint = sim.simxGetObjectHandle(clientId, 'right_joint1', sim.simx_opmode_blocking)
        res, prox_sensor = sim.simxGetObjectHandle(clientId, 'Proximity_sensor0', sim.simx_opmode_blocking)
        self.left_joint = left_joint
        self.right_joint = right_joint
        self.prox_sensor = prox_sensor
        self.clientId = clientId
        sim.simxAddStatusbarMessage(self.clientId,'Obstacle avoidance script initiated.',sim.simx_opmode_oneshot)

    def setWheelVelocity(self, handle, velocity):
        return sim.simxSetJointTargetVelocity(self.clientId, handle, velocity, sim.simx_opmode_oneshot)
        
    # main function - Might rename this later
    #renamed and made callable by the main program, commented out some functions
    def avoid_obstacle(self):
        
        #while True:
        res = sim.simxGetObjectGroupData(self.clientId, 5, 13, sim.simx_opmode_blocking)
        returnCode,detectionState = sim.simxReadProximitySensor(self.clientId, self.prox_sensor, sim.simx_opmode_streaming)[0:2] #simx_opmode_streaming
        print(detectionState)
        if detectionState:
        #Make Bot Turn #turning left
            print("turning")
            res = self.setWheelVelocity(self.left_joint, 100*math.pi/180)
            res = self.setWheelVelocity(self.right_joint, 50*math.pi/180)
            t.sleep(2)
        else:
        #Make Bot Move straight
            res = self.setWheelVelocity(self.left_joint, 200*math.pi/-180)
            res = self.setWheelVelocity(self.right_joint, 200*math.pi/-180)    
