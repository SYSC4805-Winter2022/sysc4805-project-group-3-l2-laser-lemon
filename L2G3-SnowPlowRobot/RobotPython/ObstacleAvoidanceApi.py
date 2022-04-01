"""
Author: Emma Boulay
SYSC 4805 L2G3

ObstacleAvoidanceApi.py is responsible for communicating with the proximity sensors
on the robot. It also utilizes the Braitenberg algorithm to detect and avoid obstacles.
"""

import sim, math

def checkForObstacle(motorControl, clientId, velocity):
    """This function checks if an object is detected and uses the Braitenberg algorithm
    to avoid obstacles.

    Args:
        motorControl (WheelModule): The wheel module for the robot
        clientId (int): The client ID for the Remote API server
        velocity (float): The velocity in rad/s

    Returns:
        _type_: _description_
    """
    #Make call to Lua program to run object detection and avoidance function
    res,retInts,retFloats,retStrings,retBuffer = sim.simxCallScriptFunction(clientId, 'robot', sim.sim_scripttype_childscript, 'detectObstacle',
                                [], [velocity[0], velocity[1]], [], bytearray(), sim.simx_opmode_blocking)
    #The Lua call returns 1 if an object was detected
    if res == sim.simx_return_ok:
        if(len(retInts) > 0):
            if(retInts[0] == 1):
                return True
        return False

def checkForObstacleBackwards(clientId):
    """This function checks if an object is detected while backing up

    Args:
        clientId (int): The client ID for the Remote API server
    Returns:
        obstacleDetected (boolean): True if obstacle detected, False otherwise
    """
    #Make call to Lua program to run object detection and avoidance function
    res,retInts,retFloats,retStrings,retBuffer = sim.simxCallScriptFunction(clientId, 'robot', sim.sim_scripttype_childscript, 'detectObstacleBackwards',
                                [], [], [], bytearray(), sim.simx_opmode_blocking)
    #The Lua call returns 1 if an object was detected
    if res == sim.simx_return_ok:
        if(len(retInts) > 0):
            if(retInts[0] == 1):
                return True
        return False

def checkForObstacleTurning(clientId):
    """This function checks if an object is detected while turning

    Args:
    clientId (int): The client ID for the Remote API server

    Returns:
        obstacleDetected (boolean): True if obstacle detected, False otherwise
    """
    #Make call to Lua program to run object detection and avoidance function
    res,retInts,retFloats,retStrings,retBuffer = sim.simxCallScriptFunction(clientId, 'robot', sim.sim_scripttype_childscript, 'detectObstacleTurning',
                                [], [], [], bytearray(), sim.simx_opmode_blocking)
    #The Lua call returns 1 if an object was detected
    if res == sim.simx_return_ok:
        if(len(retInts) > 0):
            if(retInts[0] == 1):
                return True
        return False

def checkForObstaclePlow(clientId):
    """This function checks if an object is detected in the plow area

    Args:
        clientId (int): The client ID for the Remote API server

    Returns:
        obstacleDetected (boolean): True if obstacle detected, False otherwise
    """
    #Make call to Lua program to run object detection and avoidance function
    res,retInts,retFloats,retStrings,retBuffer = sim.simxCallScriptFunction(clientId, 'robot', sim.sim_scripttype_childscript, 'detectObstaclePlow',
                                [], [], [], bytearray(), sim.simx_opmode_blocking)
    #The Lua call returns 1 if an object was detected
    if res == sim.simx_return_ok:
        if(len(retInts) > 0):
            if(retInts[0] == 1):
                return True
        return False

########################## IGNORE#########################
def checkIfInBounds(motorControl, clientId):
    res, currentPos = sim.simxGetObjectPosition(clientId, motorControl.robot, -1, sim.simx_opmode_blocking)
    print(currentPos)
    print(motorControl.initPos)
    inBounds = True
    if motorControl.initPos[0] + 6.2 < currentPos[0]:
        inBounds = False
    elif motorControl.initPos[0] - 6.2 > currentPos[0]:
        inBounds = False
    elif (motorControl.initPos[1] + 12.2) < currentPos[1]:
        inBounds = False
    elif motorControl.initPos[1] > currentPos[1]:
        inBounds = False
    if (not inBounds):
        print("Not in bounds")
        RotateToCenter(motorControl, clientId)
    print("In Bounds")
    return
def RotateToCenter(motorControl, clientId):
    motorControl.stop()
    res, currentAngle = sim.simxGetObjectOrientation(clientId, motorControl.robot, -1, sim.simx_opmode_blocking)
    uAngle = currentAngle[2]
    res, currentPos = sim.simxGetObjectPosition(clientId, motorControl.robot, -1, sim.simx_opmode_blocking)
    u = [math.cos(uAngle), math.sin(uAngle)]
    r = [-currentPos[0], -currentPos[1]]
    rDistance = VectorDistance(r, [0,0])
    r = [-currentPos[0]/rDistance, -currentPos[1]/rDistance]
    angle = dotProduct(u, r)
    motorControl.turnLeft(angle)

def dotProduct(v1, v2):
    dot = (v1[0]*v2[0]+v1[1]*v2[1])
    print(dot)
    return math.acos()
    
def VectorDistance(v1, v2):
    return math.sqrt((v2[0]-v1[0])**2 + (v2[1]+v1[1])**2)