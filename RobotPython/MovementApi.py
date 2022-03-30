import sim
import math
import time
import ObstacleAvoidanceApi as ObsAvoid

class WheelModule:
    def __init__(self, clientId):
        res, left_joint  = sim.simxGetObjectHandle(clientId, '/robot/leftJoint', sim.simx_opmode_blocking)
        res, right_joint = sim.simxGetObjectHandle(clientId, '/robot/rightJoint', sim.simx_opmode_blocking)
        res, leftWheel = sim.simxGetObjectHandle(clientId, '/robot/leftJoint/respondableLeftWheel_', sim.simx_opmode_blocking)
        res, rightWheel = sim.simxGetObjectHandle(clientId, '/robot/rightJoint/respondableRightWheel_', sim.simx_opmode_blocking)
        res, robot = sim.simxGetObjectHandle(clientId, '/robot', sim.simx_opmode_blocking)
        self.left_joint = left_joint
        self.right_joint = right_joint
        self.rightWheel = rightWheel
        self.leftWheel = leftWheel
        self.robot = robot
        self.clientId = clientId
        self.thread = None
        self.straightBool = False
        self.emergencyStop = False
        self.direction = {
            "n": 0,
            "e": -1.57,
            "s": -3.14,
            "w": 1.57
        }
        self.currentDirection = "n"
        self.wheelRadius = 0.172/2

        self.velocity = -1

        res, self.initPos = sim.simxGetObjectPosition(self.clientId, self.robot, -1, sim.simx_opmode_blocking)

    def setWheelVelocity(self, handle, velocity):
        return sim.simxSetJointTargetVelocity(self.clientId, handle, velocity, sim.simx_opmode_blocking)

    def stop(self):
        print("Stopping")
        self.straightBool = False
        self.callStraight(0)
        time.sleep(0.2)

    def turnRight(self, deg):
        if(not self.emergencyStop):
            radToTurn = deg*math.pi/180
            res, currRad = sim.simxGetObjectOrientation(self.clientId, self.robot, -1, sim.simx_opmode_blocking)
            if (currRad[2] < 0):
                currRad[2] += 6.28
            targetDirection = (currRad[2] - radToTurn)
            if(targetDirection < 0):
                targetDirection = 6.28 + targetDirection
            res = self.setWheelVelocity(self.left_joint, 0.7/self.wheelRadius)
            res = self.setWheelVelocity(self.right_joint, -0.1/self.wheelRadius)
            while not self.emergencyStop:
                if ObsAvoid.checkForObstacle(self, self.clientId, -2/self.wheelRadius):
                    self.emergencyStopFunc()
                res, currRad = sim.simxGetObjectOrientation(self.clientId, self.robot, -1, sim.simx_opmode_blocking)
                currRad = currRad[2]
                if (currRad < 0):
                   currRad += 6.28
                if(math.fabs(currRad-targetDirection) < 0.3):
                    break
            self.stop()

        # if(not self.emergencyStop):
        #     self.straightBool = False
        #     targetDirection = self.changeDirection(self.currentDirection, deg, True)
        #     res = self.setWheelVelocity(self.left_joint, 0.7/self.wheelRadius)
        #     res = self.setWheelVelocity(self.right_joint, -0.1/self.wheelRadius)

        #     while not self.emergencyStop:
        #         res, currRad = sim.simxGetObjectOrientation(self.clientId, self.robot, -1, sim.simx_opmode_blocking)
        #         currRad = currRad[2]
        #         if(math.fabs(currRad-targetDirection) < 0.3):
        #             break
        #     self.stop()
    def turnLeft(self, deg):
        if(not self.emergencyStop):
            radToTurn = deg*math.pi/180
            res, currRad = sim.simxGetObjectOrientation(self.clientId, self.robot, -1, sim.simx_opmode_blocking)
            if (currRad[2] < 0):
                currRad[2] += 6.28
            targetDirection = (currRad[2] + radToTurn)
            if(targetDirection > 6.28):
                targetDirection -= 6.28
            res = self.setWheelVelocity(self.left_joint, -0.1/self.wheelRadius)
            res = self.setWheelVelocity(self.right_joint, 0.7/self.wheelRadius)
            while not self.emergencyStop:
                if ObsAvoid.checkForObstacle(self, self.clientId, -2/self.wheelRadius):
                    self.emergencyStopFunc()
                res, currRad = sim.simxGetObjectOrientation(self.clientId, self.robot, -1, sim.simx_opmode_blocking)
                currRad = currRad[2]
                if (currRad < 0):
                    currRad += 6.28
                if(math.fabs(currRad-targetDirection) < 0.3):
                    break
            self.stop()
        # if(not self.emergencyStop):
        #     self.straightBool = False

        #     targetDirection = self.changeDirection(self.currentDirection, deg, False)

        #     res = self.setWheelVelocity(self.left_joint, -0.1/self.wheelRadius)
        #     res = self.setWheelVelocity(self.right_joint, 0.7/self.wheelRadius)
        #     while not self.emergencyStop:
        #         res, currRad = sim.simxGetObjectOrientation(self.clientId, self.robot, -1, sim.simx_opmode_blocking)
        #         currRad = currRad[2]
        #         if(math.fabs(currRad-targetDirection) < 0.3):
        #             break
        #     self.stop()
    def straightDist(self, distance, velocity):
        if(not self.emergencyStop):
            self.straight(velocity)
            revs = distance/(math.pi*self.wheelRadius)
            totalRads = 6.28*revs
            rads = 0

            res, prevRad = sim.simxGetJointPosition(self.clientId, self.right_joint, sim.simx_opmode_blocking)
            res, origPosition = sim.simxGetObjectPosition(self.clientId, self.robot, -1, sim.simx_opmode_blocking)
            distanceTravelled = 0
            while distanceTravelled  < distance and not self.emergencyStop:
                if ObsAvoid.checkForObstacle(self, self.clientId, -velocity/self.wheelRadius):
                    self.emergencyStopFunc()
                res, pos = sim.simxGetObjectPosition(self.clientId, self.robot, -1, sim.simx_opmode_blocking)
                distanceTravelled = math.sqrt((origPosition[0]-pos[0])**2 + (origPosition[1]-pos[1])**2)
            self.stop()
    
    def straight(self, velocity):
        if(not self.emergencyStop):
            self.straightBool = True
            angularV = -1*velocity/self.wheelRadius

            res, ori = sim.simxGetObjectOrientation(self.clientId, self.robot, -1, sim.simx_opmode_blocking)
            #sim.simxSetObjectOrientation(self.clientId, self.robot, -1, [ori[0], ori[1], self.direction[self.currentDirection]], sim.simx_opmode_blocking)
            self.callStraight(angularV)

        
        

    def backward(self, distance, velocity):
        if(not self.emergencyStop):
            angularV = velocity/self.wheelRadius
            res, ori = sim.simxGetObjectOrientation(self.clientId, self.robot, -1, sim.simx_opmode_blocking)
            #sim.simxSetObjectOrientation(self.clientId, self.robot, -1, [ori[0], ori[1], self.direction[self.currentDirection]], sim.simx_opmode_blocking)
            self.callStraight(angularV)
            
            res, origPosition = sim.simxGetObjectPosition(self.clientId, self.robot, -1, sim.simx_opmode_blocking)
            distanceTravelled = 0
            while distanceTravelled  < distance and not self.emergencyStop:
                if ObsAvoid.checkForObstacle(self, self.clientId, -angularV):
                    self.emergencyStopFunc()
                res, pos = sim.simxGetObjectPosition(self.clientId, self.robot, -1, sim.simx_opmode_blocking)
                distanceTravelled = math.sqrt((origPosition[0]-pos[0])**2 + (origPosition[1]-pos[1])**2)
            self.stop()                    
             
    def callStraight(self, velocity):
        inputIntegers = []
        inputFloats = [velocity]
        inputStrings = []
        inputBuffer = bytearray()
        sim.simxCallScriptFunction(self.clientId, 'robot', sim.sim_scripttype_childscript, 'straight',
                                    inputIntegers, inputFloats, inputStrings, inputBuffer, sim.simx_opmode_blocking)

    def emergencyStopFunc(self):
        self.emergencyStop = True
        self.stop()
        time.sleep(0.2)
        self.emergencyStop = False

    def turnDirection(self, dirLeft):
        if(not self.emergencyStop):
            self.straightBool = False
            if(dirLeft):
                res = self.setWheelVelocity(self.left_joint, -0.1/self.wheelRadius)
                res = self.setWheelVelocity(self.right_joint, 0.7/self.wheelRadius)
            else:
                res = self.setWheelVelocity(self.left_joint, 0.7/self.wheelRadius)
                res = self.setWheelVelocity(self.right_joint, -0.1/self.wheelRadius)

