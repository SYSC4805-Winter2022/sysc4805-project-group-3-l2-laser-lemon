
import math
#import time
import ObstacleAvoidanceApi

class MovingState:
    STOP = 1
    STRAIGHT = 2
    BACKWARD = 3
    TURNING_LEFT = 4
    TURNING_RIGHT = 5

class WheelModule:
    def __init__(self):
        left_joint  = sim.getObjectHandle('/robot/leftJoint')
        right_joint = sim.getObjectHandle('/robot/rightJoint')
        robot = sim.getObjectHandle('/robot')
        self.left_joint = left_joint
        self.right_joint = right_joint
        self.robot = robot
        self.emergencyStop = False
        self.wheelRadius = 0.172/2
        self.robotState = MovingState.STOP
        self.velocity = -1
        #self.ObsAvoid = ObstacleAvoidanceApi.ObstacleAvoidance(self.clientId, self)

    def setWheelVelocity(self, handle, velocity):
        return sim.setJointTargetVelocity(handle, velocity)

    def stop(self):
        self.robotState = MovingState.STOP
        print("Stopping")
        self.callStraight(0)
        time.sleep(0.2)

    def turnRight(self, deg):
        self.robotState = MovingState.TURNING_RIGHT
        if(not self.emergencyStop):
            radToTurn = deg*math.pi/180
            currRad = sim.getObjectOrientation(self.robot, -1)
            if (currRad[2] < 0):
                currRad[2] += 6.28
            targetDirection = (currRad[2] - radToTurn)
            if(targetDirection < 0):
                targetDirection = 6.28 + targetDirection
            self.setWheelVelocity(self.left_joint, 0.7/self.wheelRadius)
            self.setWheelVelocity(self.right_joint, -0.1/self.wheelRadius)
            while not self.emergencyStop:
                if False: #self.ObsAvoid.checkForObstacle():
                    self.emergencyStopFunc()
                currRad = sim.getObjectOrientation(self.robot, -1)
                currRad = currRad[2]
                if (currRad < 0):
                   currRad += 6.28
                if(math.fabs(currRad-targetDirection) < 0.3):
                    break
            self.stop()

    def turnLeft(self, deg):
        self.robotState = MovingState.TURNING_LEFT
        if(not self.emergencyStop):
            radToTurn = deg*math.pi/180
            currRad = sim.getObjectOrientation(self.robot, -1)
            if (currRad[2] < 0):
                currRad[2] += 6.28
            targetDirection = (currRad[2] + radToTurn)
            if(targetDirection > 6.28):
                targetDirection -= 6.28
            self.setWheelVelocity(self.left_joint, -0.1/self.wheelRadius)
            self.setWheelVelocity(self.right_joint, 0.7/self.wheelRadius)
            while not self.emergencyStop:
                if False: #self.ObsAvoid.checkForObstacle():
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
        self.robotState = MovingState.STRAIGHT
        if(not self.emergencyStop):
            self.straight(velocity)
            revs = distance/(math.pi*self.wheelRadius)
            totalRads = 6.28*revs
            rads = 0

            res, prevRad = sim.simxGetJointPosition(self.clientId, self.right_joint, sim.simx_opmode_blocking)
            res, origPosition = sim.simxGetObjectPosition(self.clientId, self.robot, -1, sim.simx_opmode_blocking)
            distanceTravelled = 0
            while distanceTravelled  < distance and not self.emergencyStop:
                if False: #self.ObsAvoid.checkForObstacle():
                    self.emergencyStopFunc()
                res, pos = sim.simxGetObjectPosition(self.clientId, self.robot, -1, sim.simx_opmode_blocking)
                distanceTravelled = math.sqrt((origPosition[0]-pos[0])**2 + (origPosition[1]-pos[1])**2)
            self.stop()
    
    def straight(self, velocity):
        self.robotState = MovingState.STRAIGHT
        if(not self.emergencyStop):
            angularV = 1*velocity/self.wheelRadius
            self.setWheelVelocity(self.left_joint, angularV)
            self.setWheelVelocity(self.right_joint, angularV)

        
        

    def backward(self, distance, velocity):
        self.robotState = MovingState.BACKWARD
        if(not self.emergencyStop):
            angularV = velocity/self.wheelRadius
            res, ori = sim.simxGetObjectOrientation(self.clientId, self.robot, -1, sim.simx_opmode_blocking)
            #sim.simxSetObjectOrientation(self.clientId, self.robot, -1, [ori[0], ori[1], self.direction[self.currentDirection]], sim.simx_opmode_blocking)
            self.callStraight(angularV)
            
            res, origPosition = sim.simxGetObjectPosition(self.clientId, self.robot, -1, sim.simx_opmode_blocking)
            distanceTravelled = 0
            while distanceTravelled  < distance and not self.emergencyStop:
                if False: #self.ObsAvoid.checkForObstacle():
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
            if(dirLeft):
                self.robotState = MovingState.TURNING_LEFT
                res = self.setWheelVelocity(self.left_joint, -0.05/self.wheelRadius)
                res = self.setWheelVelocity(self.right_joint, 0.2/self.wheelRadius)
            else:
                self.robotState = MovingState.TURNING_RIGHT
                res = self.setWheelVelocity(self.left_joint, 0.2/self.wheelRadius)
                res = self.setWheelVelocity(self.right_joint, -0.05/self.wheelRadius)
    
    def getRobotState(self):
        return self.robotState

