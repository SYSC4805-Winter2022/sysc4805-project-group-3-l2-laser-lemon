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
    res,retInts,retFloats,retStrings,retBuffer = sim.simxCallScriptFunction(clientId, '/robot', sim.sim_scripttype_childscript, 'detectObstacle',
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
    res,retInts,retFloats,retStrings,retBuffer = sim.simxCallScriptFunction(clientId, '/robot', sim.sim_scripttype_childscript, 'detectObstacleBackwards',
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
    res,retInts,retFloats,retStrings,retBuffer = sim.simxCallScriptFunction(clientId, '/robot', sim.sim_scripttype_childscript, 'detectObstacleTurning',
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
    res,retInts,retFloats,retStrings,retBuffer = sim.simxCallScriptFunction(clientId, '/robot', sim.sim_scripttype_childscript, 'detectObstaclePlow',
                                [], [], [], bytearray(), sim.simx_opmode_blocking)
    #The Lua call returns 1 if an object was detected
    if res == sim.simx_return_ok:
        if(len(retInts) > 0):
            if(retInts[0] == 1):
                return True
        return False