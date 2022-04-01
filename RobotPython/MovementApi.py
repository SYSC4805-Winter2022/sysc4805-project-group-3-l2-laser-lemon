import sim
import math
import time
import ObstacleAvoidanceApi as ObsAvoid

class MovingState:
    STOP = 1
    STRAIGHT = 2
    BACKWARD = 3
    TURNING_LEFT = 4
    TURNING_RIGHT = 5

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
        self.emergencyStop = False
        self.wheelRadius = 0.172/2
        self.robotState = MovingState.STOP
        self.velocity = 2/self.wheelRadius
        self.initPos = sim.simxGetObjectPosition(self.clientId, self.robot, -1, sim.simx_opmode_blocking)[1]

    def setWheelVelocity(self, handle, velocity):
        return sim.simxSetJointTargetVelocity(self.clientId, handle, velocity, sim.simx_opmode_oneshot)

    def stop(self):
        self.robotState = MovingState.STOP
        self.callStraight(0)
        time.sleep(0.2)

    def turnRight(self, deg):
        self.robotState = MovingState.TURNING_RIGHT
        if(not self.emergencyStop):
            sim.simxGetObjectOrientation(self.clientId, self.robot, -1, sim.simx_opmode_streaming)
            radToTurn = deg*math.pi/180
            res, currRad = simxGetObjectOrientation(self.clientId, self.robot, -1, sim.simx_opmode_buffer)
            if (currRad[2] < 0):
                currRad[2] += 6.28
            targetDirection = (currRad[2] - radToTurn)
            if(targetDirection < 0):
                targetDirection = 6.28 + targetDirection
            res = self.setWheelVelocity(self.left_joint, 0.7/self.wheelRadius)
            res = self.setWheelVelocity(self.right_joint, -0.1/self.wheelRadius)
            while not self.emergencyStop:
                res, currRad = simxGetObjectOrientation(self.clientId, self.robot, -1, sim.simx_opmode_buffer)
                currRad = currRad[2]
                if (currRad < 0):
                   currRad += 6.28
                if(math.fabs(currRad-targetDirection) < 0.3):
                    break
            sim.simxGetObjectOrientation(self.clientId, self.robot, -1, sim.simx_opmode_discontinue)
            self.stop()
    def turnLeft(self, deg):
        self.robotState = MovingState.TURNING_LEFT
        objectDetect = False
        if(not self.emergencyStop):
            radToTurn = deg*math.pi/180
            sim.simxGetObjectOrientation(self.clientId, self.robot, -1, sim.simx_opmode_streaming)
            res, currRad = sim.simxGetObjectOrientation(self.clientId, self.robot, -1, sim.simx_opmode_buffer)
            if (currRad[2] < 0):
                currRad[2] += 6.28
            targetDirection = (currRad[2] + radToTurn)
            if(targetDirection > 6.28):
                targetDirection -= 6.28
            res = self.setWheelVelocity(self.left_joint, -0.1/self.wheelRadius)
            res = self.setWheelVelocity(self.right_joint, 0.7/self.wheelRadius)
            while not self.emergencyStop:
                res, currRad = sim.simxGetObjectOrientation(self.clientId, self.robot, -1, sim.simx_opmode_buffer)
                currRad = currRad[2]
                if (currRad < 0):
                    currRad += 6.28
                if(math.fabs(currRad-targetDirection) < 0.3):
                    break
            sim.simxGetObjectOrientation(self.clientId, self.robot, -1, sim.simx_opmode_discontinue)
            self.stop()

    def straightDist(self, distance, velocity):
        self.robotState = MovingState.STRAIGHT
        if(not self.emergencyStop):
            self.straight(velocity)
            revs = distance/(math.pi*self.wheelRadius)
            sim.simxGetObjectPosition(self.clientId, self.robot, -1, sim.simx_opmode_streaming)
            res, origPosition = simxGetObjectPosition(self.clientId, self.robot, -1, sim.simx_opmode_buffer)
            distanceTravelled = 0
            while distanceTravelled  < distance and not self.emergencyStop:
                res, pos = simxGetObjectPosition(self.clientId, self.robot, -1, sim.simx_opmode_buffer)
                distanceTravelled = math.sqrt((origPosition[0]-pos[0])**2 + (origPosition[1]-pos[1])**2)
            sim.simxGetObjectPosition(self.clientId, self.robot, -1, sim.simx_opmode_discontinue)
            self.stop()
    
    def straight(self, velocity):
        self.robotState = MovingState.STRAIGHT
        if(not self.emergencyStop):
            angularV = -1*velocity/self.wheelRadius
            self.callStraight(angularV)

        
        

    def backward(self, distance, velocity):
        self.robotState = MovingState.BACKWARD
        if(not self.emergencyStop):

            angularV = velocity/self.wheelRadius
            sim.simxGetObjectPosition(self.clientId, self.robot, -1, sim.simx_opmode_streaming)
            #sim.simxSetObjectOrientation(self.clientId, self.robot, -1, [ori[0], ori[1], self.direction[self.currentDirection]], sim.simx_opmode_blocking)
            self.callStraight(angularV)
            
            res, origPosition = simxGetObjectPosition(self.clientId, self.robot, -1, sim.simx_opmode_buffer)
            distanceTravelled = 0
            while distanceTravelled  < distance and not self.emergencyStop:
                res, pos = simxGetObjectPosition(self.clientId, self.robot, -1, sim.simx_opmode_buffer)
                distanceTravelled = math.sqrt((origPosition[0]-pos[0])**2 + (origPosition[1]-pos[1])**2)
            sim.simxGetObjectPosition(self.clientId, self.robot, -1, sim.simx_opmode_discontinue)
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

def simxGetObjectPosition(clientId, handle, reference, mode):
    res, orientation = sim.simxGetObjectPosition(clientId, handle, reference, mode)
    while (res != sim.simx_return_ok):
        res, orientation = sim.simxGetObjectPosition(clientId, handle, reference, mode)
    return res, orientation

def simxGetObjectOrientation(clientId, handle, reference, mode):
    res, orientation = sim.simxGetObjectOrientation(clientId, handle, reference, mode)
    while (res != sim.simx_return_ok):
        res, orientation = sim.simxGetObjectOrientation(clientId, handle, reference, mode)
    return res, orientation

