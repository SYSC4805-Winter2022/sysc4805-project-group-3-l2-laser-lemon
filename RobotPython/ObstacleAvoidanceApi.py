'''
SYSC 4805 
Group L2-3
'''


import sim
import time as t
import math
import MovementApi
class ObstacleAvoidance:
    def __init__(self, clientId):
        res, left_joint  = sim.simxGetObjectHandle(clientId, 'leftJoint', sim.simx_opmode_blocking)
        res, right_joint = sim.simxGetObjectHandle(clientId, 'rightJoint', sim.simx_opmode_blocking)
        res, prox_sensor_front = sim.simxGetObjectHandle(clientId, 'Proximity_sensor0', sim.simx_opmode_blocking)
        res, prox_sensor_right = sim.simxGetObjectHandle(clientId, 'Proximity_sensor1', sim.simx_opmode_blocking)
        res, prox_sensor_back = sim.simxGetObjectHandle(clientId, 'Proximity_sensor2', sim.simx_opmode_blocking)
        res, prox_sensor_left= sim.simxGetObjectHandle(clientId, 'Proximity_sensor3', sim.simx_opmode_blocking)
        self.left_joint = left_joint
        self.right_joint = right_joint
        self.prox_sensor_right = prox_sensor_right
        self.prox_sensor_front = prox_sensor_front
        self.prox_sensor_left = prox_sensor_left
        self.prox_sensor_back = prox_sensor_back
        self.clientID = clientId
        self.prevObst = False

        self.motors = MovementApi.WheelModule(self.clientID)
        #sim.simxAddStatusbarMessage(self.clientID,'Obstacle avoidance script initiated.',sim.simx_opmode_oneshot)

    def setWheelVelocity(self, handle, velocity):
        return sim.simxSetJointTargetVelocity(self.clientID, handle, velocity, sim.simx_opmode_blocking)
        
    # main function - Might rename this later
    #renamed and made callable by the main program, commented out some functions
    def avoid_obstacle(self):
        
        res = sim.simxGetObjectGroupData(self.clientID, 5, 13, sim.simx_opmode_blocking)
        returnCode, detectionState = sim.simxReadProximitySensor(self.clientID, self.prox_sensor_front, sim.simx_opmode_streaming)[0:2] #simx_opmode_streaming
        returnCode, detectrightSide = sim.simxReadProximitySensor(self.clientID, self.prox_sensor_right, sim.simx_opmode_streaming)[0:2] #simx_opmode_streaming
        returnCode, detectleftSide = sim.simxReadProximitySensor(self.clientID, self.prox_sensor_left, sim.simx_opmode_streaming)[0:2] #simx_opmode_streaming
        returnCode, detectBack = sim.simxReadProximitySensor(self.clientID, self.prox_sensor_back, sim.simx_opmode_streaming)[0:2] #simx_opmode_streaming

        if detectionState:
            self.prevObst = True
            print("turning")
            self.motors.emergencyStopFunc()
            res = self.setWheelVelocity(self.left_joint, -50*math.pi/180)
            res = self.setWheelVelocity(self.right_joint, 5*math.pi/180)
            t.sleep(1)

        #if right sensor detects an obstacle, robot should turn left
        elif False and detectrightSide:
            self.prevObst = True
            #Make Bot Turn #turning left
            print("turning")
            self.motors.emergencyStopFunc()
            res = self.setWheelVelocity(self.left_joint, 100*math.pi/180)
            res = self.setWheelVelocity(self.right_joint, 50*math.pi/180)
            t.sleep(1)

        elif False and detectleftSide:
            self.prevObst = True
            #Make Bot Turn #turning right
            print("turning")
            self.motors.emergencyStopFunc()
            res = self.setWheelVelocity(self.left_joint, 50*math.pi/180)
            res = self.setWheelVelocity(self.right_joint, 100*math.pi/180)
            t.sleep(1)

        elif False and detectBack:
            self.prevObst = True
            #Make Bot Move straight
            self.motors.emergencyStopFunc()
            res = self.setWheelVelocity(self.left_joint, 200*math.pi/-180)
            res = self.setWheelVelocity(self.right_joint, 200*math.pi/-180)
        else:
            if(self.prevObst):
                self.motors.straight(-2)
                self.prevObst = False
