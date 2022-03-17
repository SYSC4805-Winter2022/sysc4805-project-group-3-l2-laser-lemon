import sim
import math
import time
import threading

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
        self.straightBool = False
        self.direction = {
            "n": 0,
            "e": -1.57,
            "s": -3.14,
            "w": 1.57
        }
        self.currentDirection = "n"
        self.wheelRadius = 0.30328/2

        self.velocity = -1

    def setWheelVelocity(self, handle, velocity):
        return sim.simxSetJointTargetVelocity(self.clientId, handle, velocity, sim.simx_opmode_blocking)

    def stop(self):
        print("Stopping")
        self.straightBool = False
        res = self.setWheelVelocity(self.left_joint, -0.1)
        res = self.setWheelVelocity(self.right_joint, -0.1)

    def turnRight(self, deg):
        
        self.straightBool = False
        targetDirection = self.changeDirection(self.currentDirection, deg, True)
        res = self.setWheelVelocity(self.left_joint, -150*math.pi/180)
        res = self.setWheelVelocity(self.right_joint, 15*math.pi/180)
        while True:
            res, currRad = sim.simxGetObjectOrientation(self.clientId, self.robot, -1, sim.simx_opmode_blocking)
            currRad = currRad[2]
            print(currRad)
            if(math.fabs(currRad-targetDirection) < 0.1):
                break
        self.stop()
    def turnLeft(self, deg):
        self.straightBool = False
        targetDirection = self.changeDirection(self.currentDirection, deg, False)
        res = self.setWheelVelocity(self.left_joint, 15*math.pi/180)
        res = self.setWheelVelocity(self.right_joint, -150*math.pi/180)
        while True:
            res, currRad = sim.simxGetObjectOrientation(self.clientId, self.robot, -1, sim.simx_opmode_blocking)
            currRad = currRad[2]
            print(currRad)
            if(math.fabs(currRad-targetDirection) < 0.1):
                break
        self.stop()
    def straightDist(self, distance, velocity):
        self.straight(velocity)
        revs = distance/(0.9525)
        totalRads = 6.28*revs
        print(totalRads)
        rads = 0

        res, prevRad = sim.simxGetJointPosition(self.clientId, self.right_joint, sim.simx_opmode_blocking)
        print(rads >= totalRads)
        while rads  < totalRads:
            
            print(rads)
            res, newRad = sim.simxGetJointPosition(self.clientId, self.right_joint, sim.simx_opmode_blocking)
            #print("Prev Rads: " + str(prevRad) + ", CurrentRad: " + str(newRad))
            if(not math.isnan(newRad)):
                if prevRad < 0 and newRad > 0:
                    rads = rads +  6.28 + prevRad - newRad
                else:
                    rads = rads + math.fabs(prevRad-newRad)
                prevRad = newRad

        self.stop()
    
    def straight(self, velocity):
        self.straightBool = True
        angularV = -1*velocity/self.wheelRadius
        self.velocity = angularV
        res = self.setWheelVelocity(self.right_joint, angularV)
        res = self.setWheelVelocity(self.left_joint, angularV)
        
        res, self.targetOrientation = sim.simxGetObjectOrientation(self.clientId, self.robot, -1, sim.simx_opmode_blocking)
        self.targetOrientation = self.targetOrientation[2]

        res, ori = sim.simxGetObjectOrientation(self.clientId, self.robot, -1, sim.simx_opmode_blocking)
        sim.simxSetObjectOrientation(self.clientId, self.robot, -1, [ori[0], ori[1], self.direction[self.currentDirection]], sim.simx_opmode_blocking)

    def backward(self, distance, velocity):
        angularV = velocity/self.wheelRadius
        res = self.setWheelVelocity(self.left_joint, angularV)
        res = self.setWheelVelocity(self.right_joint, angularV)
        time.sleep(2)
        print("Sleep: " + str(distance*math.fabs(velocity)))
        self.stop()

    def stayOnPath(self):
        if(self.straightBool == True):
            res, currentOrientation = sim.simxGetObjectOrientation(self.clientId, self.robot, -1, sim.simx_opmode_blocking)
            currentOrientation = currentOrientation[2]
            
            targetOrientation = self.direction[self.currentDirection]
            print("CurrentOrientation: " + str(currentOrientation) + ", Target Orientation: " + str(targetOrientation))
            if(not math.isnan(currentOrientation)):
                if(self.currentDirection in ["n", "s"]):
                    if currentOrientation < targetOrientation:
                        self.velocity = self.velocity + self.velocity*0.01
                        res = self.setWheelVelocity(self.right_joint, self.velocity)
                    elif currentOrientation > targetOrientation:
                        self.velocity = self.velocity - self.velocity*0.01
                        res = self.setWheelVelocity(self.right_joint, self.velocity)
                elif(self.currentDirection in ["e"]):
                    if currentOrientation < targetOrientation:
                        self.velocity = self.velocity - self.velocity*0.01
                        res = self.setWheelVelocity(self.left_joint, self.velocity)
                        
                    elif currentOrientation > targetOrientation:
                        self.velocity = self.velocity + self.velocity*0.01
                        res = self.setWheelVelocity(self.left_joint, self.velocity)
                elif(self.currentDirection in ["w"]):
                    print()
                    if currentOrientation > 0 and math.fabs(currentOrientation/3.14) > 0.05:
                        #self.velocity = self.velocity - self.velocity*0.01
                        
                        #res = self.setWheelVelocity(self.left_joint, self.velocity)
                        res, ori = sim.simxGetObjectOrientation(self.clientId, self.robot, -1, sim.simx_opmode_blocking)
                        sim.simxSetObjectOrientation(self.clientId, self.robot, -1, [ori[0], ori[1], self.direction[self.currentDirection]], sim.simx_opmode_blocking)
                        
                    elif currentOrientation < 0 and math.fabs(currentOrientation/3.14) > 0.05:
                        #self.velocity = self.velocity + self.velocity*0.01
                        #res = self.setWheelVelocity(self.left_joint, self.velocity)
                        res, ori = sim.simxGetObjectOrientation(self.clientId, self.robot, -1, sim.simx_opmode_blocking)
                        sim.simxSetObjectOrientation(self.clientId, self.robot, -1, [ori[0], ori[1], self.direction[self.currentDirection]], sim.simx_opmode_blocking)
                    print("Left Wheel Velocity: " + str(self.velocity))
    
    """
    param leftOrRight, true if right, false if left
    """
    def changeDirection(self, current, deg, leftOrRight):
        turns = deg//90
        i = list(self.direction.keys()).index(self.currentDirection)
        dir = 1 if leftOrRight else -1
        print("Current Direction: " + str(self.currentDirection))
        newI = (i + turns*dir) % 4
        print("New I: " + str(newI) + " Turns: " + str(turns) + " Dir: " + str(dir))
        self.currentDirection = list(self.direction.keys())[int(newI)]
        print("Current Direction: " + str(self.currentDirection))

        return self.direction[self.currentDirection]

    def getDirection(self):
        return self.currentDirection


                


