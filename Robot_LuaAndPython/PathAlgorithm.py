
import PlowApi as plow
import ObstacleAvoidanceApi
import LineDetectionApi
import MovementApi
import time
import threading
import multiprocessing

import sim
import sys
from ctypes import c_wchar_p
from random import randrange


flagEastWest = True
def startSimulation():
    print ('Program started')
    sim.simxFinish(-1)
    clientId = sim.simxStart('127.0.0.1',19996,True,True,5000,1)
    if clientId == -1:
        print("Connection Unsuccesful")
        sys.exit(-1)
    
    return clientId


class SnowPlowRobot:
    def __init__(self):
         #n-North, e-East, w-West, s-South
        self.turning = False
        self.mainThread = None
        self.insideBoundary = True
        manager = multiprocessing.Manager()
        self.boundaryHit = manager.Value(c_wchar_p, "-1")
        self.clientId = startSimulation()
        self.vision = LineDetectionApi.VisionModule(self.clientId)
        self.motorControl = MovementApi.WheelModule(self.clientId)
        self.obsAvoidance = ObstacleAvoidanceApi.ObstacleAvoidance(self.clientId, self.motorControl)

        self.runMainLoop = False
        plow.open(self.clientId)

        self.motorControl.straightDist(1, -2)
        self.motorControl.turnRight(90)
        self.motorControl.stop()
        self.motorControl.straight(-2)
        self.turningLeft = 1
        print("Entering Detection Loop")
        while True:
            self.checkForLine()
            self.obsAvoidance.checkForObstacle()

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



if __name__ == "__main__":
    
    #obsAvoid = ObstacleAvoidanceApi.ObstacleAvoidance(clientId)
    print("Welcome to Lemon-Laser's Autonomous Snow Plow Robot")
    LemonLaserPlow = SnowPlowRobot()
    while True:
        continue

