import math
import time
from random import randrange
#import ObstacleAvoidanceApi


class VisionModule:

    def __init__(self):
        left_vSensor  = sim.getObjectHandle("/robot/LeftSensor")
        middle_vSensor = sim.getObjectHandle("/robot/MiddleSensor")
        right_vSensor = sim.getObjectHandle("/robot/RightSensor")
        #res, right_vSensor_side = sim.simxGetObjectHandle(clientId, "RightSensor_side", sim.simx_opmode_blocking)
        self.vSensors = [left_vSensor, middle_vSensor, right_vSensor]#, right_vSensor_side]
        pass
    def detectLine(self):
        sensor_values=[0,0,0]#,0]
        for i in range(0,len(self.vSensors)):
            visionHandle = sim.readVisionSensor(self.vSensors[i])
            if visionHandle != -1:
                print(visionHandle)
                if(visionHandle[0] > -1): #If the vision sensor detected new thing, update
                    if(visionHandle[1][10] < 0.4): #If it sees black (low intensity)
                        sensor_values[i] = 1
                    else:
                        sensor_values[i] = 0
                return sensor_values
        return -1

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
        self.straight(0)
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

            prevRad = sim.getJointPosition(self.right_joint)
            origPosition = sim.getObjectPosition(self.robot, -1)
            distanceTravelled = 0
            while distanceTravelled  < distance and not self.emergencyStop:
                if False: #self.ObsAvoid.checkForObstacle():
                    self.emergencyStopFunc()
                pos = sim.getObjectPosition(self.clientId, self.robot, -1)
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
            angularV = -velocity/self.wheelRadius
            self.straight(angularV)
            
            origPosition = sim.getObjectPosition(self.robot, -1)
            distanceTravelled = 0
            while distanceTravelled  < distance and not self.emergencyStop:
                if False: #self.ObsAvoid.checkForObstacle():
                    self.emergencyStopFunc()
                pos = sim.getObjectPosition(self.robot, -1)
                distanceTravelled = math.sqrt((origPosition[0]-pos[0])**2 + (origPosition[1]-pos[1])**2)
            self.stop()                    

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

class ObstacleAvoidance:
    def __init__(self, clientId, motorControl):
        res, left_joint  = sim.simxGetObjectHandle(clientId, 'leftJoint', sim.simx_opmode_blocking)
        res, right_joint = sim.simxGetObjectHandle(clientId, 'rightJoint', sim.simx_opmode_blocking)
        res, prox_sensor_front = sim.simxGetObjectHandle(clientId, 'Proximity_sensorFront', sim.simx_opmode_blocking)
        res, prox_sensor_right = sim.simxGetObjectHandle(clientId, 'Proximity_sensorRight', sim.simx_opmode_blocking)
        res, prox_sensor_left = sim.simxGetObjectHandle(clientId, 'Proximity_sensorLeft', sim.simx_opmode_blocking)
        res, prox_sensor_back= sim.simxGetObjectHandle(clientId, 'Proximity_sensorBack', sim.simx_opmode_blocking)
        self.left_joint = left_joint
        self.right_joint = right_joint
        self.prox_sensor_right = prox_sensor_right
        self.prox_sensor_front = prox_sensor_front
        self.prox_sensor_left = prox_sensor_left
        self.prox_sensor_back = prox_sensor_back
        self.clientID = clientId
        self.prevObst = False
        self.turningLeft = False
        self.robot = motorControl
        #sim.simxAddStatusbarMessage(self.clientID,'Obstacle avoidance script initiated.',sim.simx_opmode_oneshot)

    def setWheelVelocity(self, handle, velocity):
        return sim.simxSetJointTargetVelocity(self.clientID, handle, velocity, sim.simx_opmode_blocking)
        
    # main function - Might rename this later
    #renamed and made callable by the main program, commented out some functions
    def checkForObstacle(self):
        #res = sim.simxGetObjectGroupData(self.clientID, 5, 13, sim.simx_opmode_blocking)
        returnCode, detectionState = sim.simxReadProximitySensor(self.clientID, self.prox_sensor_front, sim.simx_opmode_blocking)[0:2] #simx_opmode_streaming
        returnCode, detectrightSide = sim.simxReadProximitySensor(self.clientID, self.prox_sensor_right, sim.simx_opmode_blocking)[0:2] #simx_opmode_streaming
        returnCode, detectleftSide = sim.simxReadProximitySensor(self.clientID, self.prox_sensor_left, sim.simx_opmode_blocking)[0:2] #simx_opmode_streaming
        returnCode, detectBack = sim.simxReadProximitySensor(self.clientID, self.prox_sensor_back, sim.simx_opmode_blocking)[0:2] #simx_opmode_streaming

        if detectionState:
            if(not self.prevObst):
                self.robot.emergencyStopFunc()
                self.prevObst = True
                if(detectrightSide and not detectleftSide):
                    print("Detected Right Side, turning left")
                    self.robot.turnDirection(True)
                elif(not detectrightSide and detectleftSide):
                    print("Detected left Side, turning right")
                    self.robot.turnDirection(False)
                else:
                    print("Detection")
                    self.robot.turnDirection(randrange(2))
            return True

        elif detectBack and self.robot.getRobotState()==3:
            print("Detected Backward")
            self.prevObst = True
            #Make Bot Move straight
            self.robot.emergencyStopFunc()
            return True
            
        else:
            self.robot.straight(-2)
            self.prevObst = False
            return False





class Plow:
    def __init__(self):
        self.open = False
        self.close = False
        self.rightHinge = getObjectHandle("/robot/plow_middle_panel/plow_right_hinge")
        self.leftHinge = getObjectHandle("/robot/plow_middle_panel/plow_left_hinge")
        self.rightPanel = getObjectHandle("/robot/plow_middle_panel/plow_right_hinge/plow_right_panel")
        self.leftPanel = getObjectHandle("/robot/plow_middle_panel/plow_left_hinge/plow_left_panel")
        self.middlePanel = getObjectHandle("/robot/plow_middle_panel")
    
        self.leftPanelBool = False
        self.rightPanelBool = False

    def onOpen(self):
        print("Beginning to Open Plow!")
        self.leftPanelBool = True
        self.rightPanelBool = True
        setJointTargetVelocity(self.rightHinge, -1.5)
        setJointTargetVelocity(self.leftHinge, 1.5)

    def onClose(self):
        print("Beginning to Close Plow!")
        while True:
            if self.rightPanelBool:
                sim.setJointTargetVelocity(self.rightHinge, 0.5)
            if self.leftPanelBool:
                sim.setJointTargetVelocity(self.leftHinge, -0.5)
            rotRight = sim.getObjectOrientation(self.rightPanel, self.middlePanel)[2] * 57.2957795
            if rotRight <= 2:
                sim.setJointTargetVelocity(self.rightHinge, 0)
                self.rightPanelBool = False
        
            rotLeft = sim.getObjectOrientation(self.leftPanel, self.middlePanel)[2] * 57.2957795
            if rotLeft >= -2:
                sim.setJointTargetVelocity(self.leftHinge, 0)
                self.leftPanelBool = True

            if not self.leftPanelBool and not self.rightPanelBool:
                close = False
                break
    def plow_actuation(self):
        while self.leftPanelBool and self.rightPanelBool:
            rotRight = getObjectOrientation(self.rightPanel, self.middlePanel)[2] * 57.2957795
            if rotRight > 135.0:
                sim.setJointTargetVelocity(self.rightHinge, 0)
                self.rightPanelBool = False
            rotLeft = getObjectOrientation(self.leftPanel, self.middlePanel)[2] * 57.2957795
            print(rotLeft)
            if math.fabs(rotLeft +135.0) < 1:
                print("stopping")
                sim.setJointTargetVelocity(self.leftHinge, 0)
                self.leftPanelBool = False

    def plow_cleanup(self):
        print("Cleaning")
        #self.setObjectToDynamic(self.leftPanel)
        #self.setObjectToDynamic(self.rightPanel)
        

    def setObjectToStatic(self, handle):
        sim.setObjectInt32Parameter(handle,sim.shapeintparam_static,1)
        sim.resetDynamicObject(handle)
        return {},{},{},''

    def setObjectToDynamic(self, handle):
        sim.setObjectInt32Parameter(handle,sim.shapeintparam_static,0)
        sim.resetDynamicObject(handle)
        return {},{},{},''



def setJointTargetVelocity(handle, velocity):
    print("Set Joint")
    print(sim.setJointTargetVelocity(handle, velocity))
    while(sim.setJointTargetVelocity(handle, velocity) == -1):
        continue

def getObjectOrientation(handle, reference):
    orientation = sim.getObjectOrientation(handle, reference)
    while orientation == -1:
        orientation = sim.getObjectOrientation(handle, reference)
    return orientation

def getObjectHandle(handle):
    print("Handle")
    handle = sim.getObjectHandle(handle)
    while handle == -1:
        handle = sim.getObjectHandle(handle)
    return handle

class Robot:
    
    def __init__(self):
        self.turning = False
        self.mainThread = None
        self.insideBoundary = True
        self.vision = VisionModule()
        self.motorControl = WheelModule()
        #self.obsAvoidance = ObstacleAvoidance(self.clientId, self.motorControl)
        self.plow = Plow()
        #self.plow.onOpen()
        #time.sleep(4)
        self.motorControl.straightDist(1, -2)
        self.motorControl.turnRight(90)
        self.motorControl.stop()
        self.motorControl.straight(-2)
        self.turningLeft = 1
        print("Entering Detection Loop")
    
    def robot_actuation(self):
        self.plow.plow_actuation()

    def checkForLine(self):
        
        newDetection = True
        loop = None
        bounds = True#self.checkIfInBounds()
        sensors = self.vision.detectLine()
        if(bounds):
            if any(s == 1 for s in sensors):
                self.motorControl.straightDist(0.2, -2)
                self.motorControl.stop()
                self.motorControl.backward(0.6,-2)

                if(self.turningLeft):
                    self.motorControl.turnLeft(randrange(135, 270))
                else:
                    self.motorControl.turnRight(randrange(135, 270))
                self.motorControl.stop()
                self.turningLeft = (self.turningLeft  + 1) % 2
                self.motorControl.straight(-2)
            else:
                newDetection = True   