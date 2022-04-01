"""
Author: Emma Boulay
SYSC 4805 L2G3

The WheelModule class is responsible for the drive control of the robot. 
It provides functionality to driving straight, backwards, and turn left or right.
"""

import sim, math, time
import ObstacleAvoidanceApi as ObsAvoid, LineDetectionApi

class MovingState:
    """States of the robot
    """
    STOP = 1
    STRAIGHT = 2
    BACKWARD = 3
    TURNING_LEFT = 4
    TURNING_RIGHT = 5

class WheelModule:
    """The WheelModule class is responsible for the drive control of the robot. 
    It provides functionality to driving straight, backwards, and turn left or right.
    """
    def __init__(self, clientId):
        #Instatiate the motors
        res, self.left_joint  = sim.simxGetObjectHandle(clientId, '/robot/leftJoint', sim.simx_opmode_blocking)
        res, self.right_joint = sim.simxGetObjectHandle(clientId, '/robot/rightJoint', sim.simx_opmode_blocking)
        res, self.robot = sim.simxGetObjectHandle(clientId, '/robot', sim.simx_opmode_blocking)
        self.clientId = clientId
        #Set emergencyStop to False. This allows for the robot to exit any loop incase of emergencies.
        self.emergencyStop = False
        #The wheel radius is 1/2 the diameter
        self.wheelRadius = 0.172/2
        #Robot is initially in the stop state
        self.robotState = MovingState.STOP
        #The velocity is set to 2m/s -> This is 2/radius for angular velocity
        self.velocity = 2/self.wheelRadius
        self.initPos = sim.simxGetObjectPosition(self.clientId, self.robot, -1, sim.simx_opmode_blocking)[1]
        self.vision = LineDetectionApi.VisionModule(self.clientId)

    def setWheelVelocity(self, handle, velocity):
        """Helper function to set wheel motor velocity

        Args:
            handle (tuple): _Object handle for motor
            velocity (float): The velocity in rad/s
        """
        sim.simxSetJointTargetVelocity(self.clientId, handle, velocity, sim.simx_opmode_oneshot)

    def stop(self):
        """Stops the robot and moves it to the "STOP" state
        """
        self.robotState = MovingState.STOP
        self.callStraight(0)
        time.sleep(0.2)

    def turnRight(self, deg):
        """Turns the robot deg degrees to the right and moves the robot to the
        "TURNING_RIGHT" state. If an emergencyStop is requested the robot will stop turning
        immediately. The preferred way to turn would be to measure the wheel odometry. However,
        because of poor computational estimates made by CoppeliaSim this resulted in unexpected behaviour.

        Args:
            deg (int): The amount of degrees to turn
        """
        #Set robot state to "TURNING_RIGHT"
        self.robotState = MovingState.TURNING_RIGHT
        if(not self.emergencyStop):
            #Start stream to optimize remote api connection
            sim.simxGetObjectOrientation(self.clientId, self.robot, -1, sim.simx_opmode_streaming)
            #Set wheels to turn right
            res = self.setWheelVelocity(self.left_joint, 0.7/self.wheelRadius)
            res = self.setWheelVelocity(self.right_joint, -0.1/self.wheelRadius) 
            res, previousAngle = simxGetObjectOrientation(self.clientId, self.robot, -1, sim.simx_opmode_buffer)
            previousAngle = previousAngle[2]*180/math.pi
            rot = 0
            #If an emergency stop immediately exit
            while not self.emergencyStop:
                #If obstacle is detected that will interfere with plow stop turning
                if(ObsAvoid.checkForObstacleTurning(self.clientId)):
                    break
                #If the line is detected turn other way slightly
                sensors = self.vision.detectLine()
                if (sensors[0] == 1):
                    self.turnRight(45)
                    break
                elif(sensors[2] == 1):
                    self.turnLeft(45)
                    break
                #Check new rotation distance
                res, angle = simxGetObjectOrientation(self.clientId, self.robot, -1, sim.simx_opmode_buffer)
                angle = angle[2]*180/math.pi
                da = previousAngle-angle
                if da >= 0 :
                    rot -= angle - previousAngle 
                else:
                    rot += angle - previousAngle
                #If rotated desired amount, exit
                if rot >= deg:
                    #CopelliaSim's orientation layout is weird. Turn 90degrees or less at a time 
                    if (deg > 90):
                        self.turnRight(deg-90) 
                    break
            sim.simxGetObjectOrientation(self.clientId, self.robot, -1, sim.simx_opmode_discontinue) #Stop stream
            #The robot has turned, now stop
            self.stop()
    def turnLeft(self, deg):
        """Turns the robot deg degrees to the right and moves the robot to the
        "TURNING_RIGHT" state. If an emergencyStop is requested the robot will stop turning
        immediately. The preferred way to turn would be to measure the wheel odometry. However,
        because of poor computational estimates made by CoppeliaSim this resulted in unexpected behaviour.

        Args:
            deg (int): The amount of degrees to turn
        """
        #Set robot state to "TURNING_RIGHT"
        self.robotState = MovingState.TURNING_LEFT
        objectDetect = False
        if(not self.emergencyStop):
            #Start stream to optimize remote api connection
            sim.simxGetObjectOrientation(self.clientId, self.robot, -1, sim.simx_opmode_streaming)
            #Set wheels to turn right
            res = self.setWheelVelocity(self.left_joint, -0.1/self.wheelRadius)
            res = self.setWheelVelocity(self.right_joint, 0.7/self.wheelRadius) 
            res, previousAngle = simxGetObjectOrientation(self.clientId, self.robot, -1, sim.simx_opmode_buffer)
            previousAngle = previousAngle[2]*180/math.pi
            rot = 0
            #If an emergency stop immediately exit
            while not self.emergencyStop:
                #If obstacle is detected that will interfere with plow stop turning
                if(ObsAvoid.checkForObstacleTurning(self.clientId)):
                    break
                #If the line is detected stop turning
                sensors = self.vision.detectLine()
                if (sensors[0] == 1):
                    self.turnRight(45)
                    break
                elif(sensors[2] == 1):
                    self.turnLeft(45)
                    break
                #Check new rotation distance
                res, angle = simxGetObjectOrientation(self.clientId, self.robot, -1, sim.simx_opmode_buffer)
                angle = angle[2]*180/math.pi
                da = previousAngle-angle
                if da >= 0 :
                    rot -= angle - previousAngle 
                else:
                    rot += angle - previousAngle
                #If rotated desired amount, exit
                if rot >= deg:
                    #CopelliaSim's orientation layout is weird. Turn 90degrees or less at a time 
                    if (deg > 90):
                        self.turnLeft(deg-90) 
                    break
            sim.simxGetObjectOrientation(self.clientId, self.robot, -1, sim.simx_opmode_discontinue) #Stop stream
            #The robot has turned, now stop
            self.stop()

    def straightDist(self, distance, velocity):
        """This function drives the robot straight for a specified distance 
           and then stops. The preferred way to turn would be to measure the wheel 
           odometry. However, because of poor computational estimates made by CoppeliaSim 
           this resulted in unexpected behaviour.

        Args:
            distance (float): The distance to travel in meters
            velocity (floate): The velocity in m/s
        """
        #Set robot state to "STRAIGHT"
        self.robotState = MovingState.STRAIGHT
        if(not self.emergencyStop):
            self.straight(velocity)
            revs = distance/(math.pi*self.wheelRadius)
            
            #Start stream to optimize remote api connection
            sim.simxGetObjectPosition(self.clientId, self.robot, -1, sim.simx_opmode_streaming)
            res, origPosition = simxGetObjectPosition(self.clientId, self.robot, -1, sim.simx_opmode_buffer)
            distanceTravelled = 0
            
            #Keep moving until travelled desired distance
            while distanceTravelled  < distance and not self.emergencyStop:
                res, pos = simxGetObjectPosition(self.clientId, self.robot, -1, sim.simx_opmode_buffer)
                distanceTravelled = math.sqrt((origPosition[0]-pos[0])**2 + (origPosition[1]-pos[1])**2)
            #Close Remote API stream
            sim.simxGetObjectPosition(self.clientId, self.robot, -1, sim.simx_opmode_discontinue)
            #Stop the robot
            self.stop()
    
    def straight(self, velocity):
        """This function drives the robot straight.
        Args:
            velocity (float): The velocity in m/s
        """
        #Set robot state to "STRAIGHT"
        self.robotState = MovingState.STRAIGHT
        if(not self.emergencyStop):
            #Calculate Angular velocity from linear velocity
            angularV = -1*velocity/self.wheelRadius
            self.callStraight(angularV)  

    def backward(self, distance, velocity):
        """This function drives the robot backward for a specified distance 
           and then stops. The preferred way to turn would be to measure the wheel 
           odometry. However, because of poor computational estimates made by CoppeliaSim 
           this resulted in unexpected behaviour.

        Args:
            distance (float): The distance to travel in meters
            velocity (float): The velocity in m/s
        """
        self.robotState = MovingState.BACKWARD
        if(not self.emergencyStop):
            #Calculate the angular velocity from linear velocity
            angularV = velocity/self.wheelRadius
            
            #Start a remote API stream connection to optimize communication
            sim.simxGetObjectPosition(self.clientId, self.robot, -1, sim.simx_opmode_streaming)
            self.callStraight(angularV) #Start going backwards
            res, origPosition = simxGetObjectPosition(self.clientId, self.robot, -1, sim.simx_opmode_buffer)
            
            distanceTravelled = 0
            #Keep moving until travelled desired distance
            while distanceTravelled  < distance and not self.emergencyStop:
                if(ObsAvoid.checkForObstacleBackwards(self.clientId)):
                    break
                res, pos = simxGetObjectPosition(self.clientId, self.robot, -1, sim.simx_opmode_buffer)
                distanceTravelled = math.sqrt((origPosition[0]-pos[0])**2 + (origPosition[1]-pos[1])**2)
            #Close remote API stream
            sim.simxGetObjectPosition(self.clientId, self.robot, -1, sim.simx_opmode_discontinue)
            self.stop()                    
             
    def callStraight(self, velocity):
        """Makes a call to the Lua function to set both wheels at the same time.
        This was necessary as delays in the RemoteApi connection had unexpected results.

        Args:
            velocity (float): The velocity in rad/s
        """
        sim.simxCallScriptFunction(self.clientId, 'robot', sim.sim_scripttype_childscript, 'straight',
                                    [], [velocity], [], bytearray(), sim.simx_opmode_blocking)

    def emergencyStopFunc(self):
        """This function will override what the robot is currently doing and stop
        """
        self.emergencyStop = True
        self.stop()
        time.sleep(0.2)
        self.emergencyStop = False

    def turnDirection(self, dirLeft):
        """This function will turn left or right until stop is called

        Args:
            dirLeft (int): 1 to turn left, 0 to turn right
        """
        if(not self.emergencyStop):
            #Turn left
            if(dirLeft):
                self.robotState = MovingState.TURNING_LEFT
                res = self.setWheelVelocity(self.left_joint, -0.05/self.wheelRadius)
                res = self.setWheelVelocity(self.right_joint, 0.2/self.wheelRadius)
            #Turn right 
            else:
                self.robotState = MovingState.TURNING_RIGHT
                res = self.setWheelVelocity(self.left_joint, 0.2/self.wheelRadius)
                res = self.setWheelVelocity(self.right_joint, -0.05/self.wheelRadius)
    
    def getRobotState(self):
        """Returns the current robot state

        Returns:
            RobotState (int): STOP = 1, STRAIGHT = 2, BACKWARD = 3, TURNING_LEFT = 4, TURNING_RIGHT = 5
        """
        return self.robotState

def simxGetObjectPosition(clientId, handle, reference, mode):
    """Helper function for getting the object position. When using
    the buffer mode it may return -1 the first time.

    Args:
        clientId (int): The client ID for the Remote API server
        handle (tuple): Object handle 
        reference (tuple): Object handle for reference
        mode (simx): The connection type

    Returns:
        res, position: The position of the handle
    """
    res, orientation = sim.simxGetObjectPosition(clientId, handle, reference, mode)
    while (res != sim.simx_return_ok):
        res, orientation = sim.simxGetObjectPosition(clientId, handle, reference, mode)
    return res, orientation

def simxGetObjectOrientation(clientId, handle, reference, mode):
    """Helper function for getting the object orientation. When using
    the buffer mode it may return -1 the first time.

    Args:
        clientId (int): The client ID for the Remote API server
        handle (tuple): Object handle 
        reference (tuple): Object handle for reference
        mode (simx): The connection type

    Returns:
        res, orientation: The orientation of the handle
    """
    res, orientation = sim.simxGetObjectOrientation(clientId, handle, reference, mode)
    while (res != sim.simx_return_ok):
        res, orientation = sim.simxGetObjectOrientation(clientId, handle, reference, mode)
    return res, orientation

