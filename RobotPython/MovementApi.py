import sim
import math
import time
from threading import Thread

class WheelModule:
    def __init__(self, clientId):
        res, left_joint  = sim.simxGetObjectHandle(clientId, 'left_joint1', sim.simx_opmode_blocking)
        res, right_joint = sim.simxGetObjectHandle(clientId, 'right_joint1', sim.simx_opmode_blocking)
        res, robot = sim.simxGetObjectHandle(clientId, 'robot', sim.simx_opmode_blocking)
        self.left_joint = left_joint
        self.right_joint = right_joint
        self.robot = robot
        self.clientId = clientId
        self.thread = None
        self.stop = False

    def setWheelVelocity(self, handle, velocity):
        return sim.simxSetJointTargetVelocity(self.clientId, handle, velocity, sim.simx_opmode_blocking)

    def stop(self):
        print("Stopping")
        self.stop = True
        self.thread.join()
        res = self.setWheelVelocity(self.left_joint, 0)
        res = self.setWheelVelocity(self.right_joint, 0)

    def turnRight(self):
        res = self.setWheelVelocity(self.right_joint, 100*math.pi/180)
        res = self.setWheelVelocity(self.left_joint, 50*math.pi/180)
        time.sleep(3.5)
        self.stop()

    def turnRight180(self):
        res = self.setWheelVelocity(self.right_joint, 100*math.pi/180)
        res = self.setWheelVelocity(self.left_joint, 50*math.pi/180)
        time.sleep(4)
        self.stop()

    def turnLeft(self, deg):

        rads = 0
        targetRad = deg*math.pi/180
        res, prevRad = sim.simxGetObjectOrientation(self.clientId, self.robot, -1, sim.simx_opmode_blocking)
        prevRad = prevRad[2]
        print(prevRad)
        res = self.setWheelVelocity(self.left_joint, 5*math.pi/180)
        res = self.setWheelVelocity(self.right_joint, -50*math.pi/180)
        while rads  < targetRad:
            res, newRad = sim.simxGetObjectOrientation(self.clientId, self.robot, -1, sim.simx_opmode_blocking)
            newRad = newRad[2]
            rads = rads + math.fabs(prevRad-newRad)
            prevRad = newRad
        self.stop()
    def straightDist(self, distance, velocity):
        res = self.setWheelVelocity(self.left_joint, -velocity)
        res = self.setWheelVelocity(self.right_joint, -velocity)
        revs = distance/(0.9525)
        totalRads = 6.28*revs
        print(totalRads)
        rads = 0

        prevRad = 0
        print(rads >= totalRads)
        while rads  < totalRads:
            print(rads)
            res, newRad = sim.simxGetJointPosition(self.clientId, self.right_joint, sim.simx_opmode_blocking)
            if prevRad < 0 and newRad > 0:
                rads = rads +  6.28 + prevRad - newRad
            else:
                rads = rads + math.fabs(prevRad-newRad)
            prevRad = newRad

        self.stop()
    
    def straight(self, velocity):
        angularV = -1*velocity/0.15164
        res = self.setWheelVelocity(self.left_joint, angularV)
        res = self.setWheelVelocity(self.right_joint, angularV)

        res, targetOrientation = sim.simxGetObjectOrientation(self.clientId, self.robot, -1, sim.simx_opmode_blocking)
        targetOrientation = targetOrientation[2]
        while(not self.stop):
            res, currentOrientation = sim.simxGetObjectOrientation(self.clientId, self.robot, -1, sim.simx_opmode_blocking)
            currentOrientation = currentOrientation[2]
            print("CurrentOrientation: " + str(currentOrientation) + ", Target Orientation: " + str(targetOrientation))
            if currentOrientation > targetOrientation:
                res = self.setWheelVelocity(self.right_joint, angularV*0.95)
            elif currentOrientation < targetOrientation:
                res = self.setWheelVelocity(self.right_joint, angularV*1.05)

    def turnLeft180(self):
        res = self.setWheelVelocity(self.left_joint, 100*math.pi/180)
        res = self.setWheelVelocity(self.right_joint, 50*math.pi/180)
        time.sleep(4)
        self.stop()

    def backward(self, distance, velocity):
        velocity = velocity/0.15164
        res = self.setWheelVelocity(self.left_joint, velocity)
        revs = distance/(0.9525)
        totalRads = 6.28*revs
        print(totalRads)
        rads = 0
        prevRad = 0
        print(rads >= totalRads)
        while rads  < totalRads:
            print(rads)
            res, newRad = sim.simxGetJointPosition(self.clientId, self.right_joint, sim.simx_opmode_blocking)
            if prevRad < 0 and newRad > 0:
                rads = rads +  6.28 + prevRad - newRad
            else:
                rads = rads + math.fabs(prevRad-newRad)
            prevRad = newRad

        self.stop()


                


