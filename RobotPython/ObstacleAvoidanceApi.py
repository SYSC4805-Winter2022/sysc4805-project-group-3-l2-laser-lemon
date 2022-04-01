import sim
import math

def checkForObstacle(motorControl, clientId, velocity):
    inputIntegers = []
    inputFloats = [velocity[0], velocity[1]]
    inputStrings = []
    inputBuffer = bytearray()
    #checkIfInBounds(motorControl, clientId)
    inputFloats = [2/motorControl.wheelRadius, 2/motorControl.wheelRadius]
    res,retInts,retFloats,retStrings,retBuffer = sim.simxCallScriptFunction(clientId, 'robot', sim.sim_scripttype_childscript, 'detectObstacle',
                                inputIntegers, inputFloats, inputStrings, inputBuffer, sim.simx_opmode_blocking)
    
    if res == sim.simx_return_ok:
        if(len(retInts) > 0):
            if(retInts[0] == 1):
                return True
        return False

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