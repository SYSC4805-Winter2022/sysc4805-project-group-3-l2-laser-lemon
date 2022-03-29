
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


flagEastWest = True
def startSimulation():
    print ('Program started')
    sim.simxFinish(-1)
    clientId = sim.simxStart('127.0.0.1',19999,True,True,5000,1)
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
        lineDetectionThread = threading.Thread(target=self.checkForLine, args=())
        
        self.obstAvoidModule = ObstacleAvoidanceApi.ObstacleAvoidance(self.clientId)
        ObstacleDetectionThread = threading.Thread(target=self.checkForObstacle, args=())

        self.runMainLoop = False
        plow.open(self.clientId)

        self.motorControl.straightDist(1, -2)
        self.motorControl.turnRight(90)
        self.motorControl.stop()
        self.motorControl.straight(-2)
        print("Entering Detection Loop")
        while True:
            self.checkForLine()
            self.checkForObstacle()


            
    def mainLoop(self, motorControl):
        print("Hello From Main Thread")
        print("main")
        self.turning = True
        if(self.boundaryHit.value != "-1"):
            print(self.boundaryHit.value == "e")
            if (self.boundaryHit.value == "n"): 
                motorControl.backward(1, -1)
                motorControl.turnRight(90)
                motorControl.stop()
                motorControl.straightDist(0.5, -1)
                motorControl.stop()
                motorControl.turnRight(90)
                motorControl.stop()
            elif (self.boundaryHit.value == "e"):
                print("Turning to West")
                motorControl.backward(1, -1)
                motorControl.turnLeft(90)
                motorControl.stop()
                motorControl.straightDist(0.5, -1)
                motorControl.stop()
                motorControl.turnLeft(90)
                motorControl.stop()
            elif (self.boundaryHit.value == "s"):
                motorControl.backward(1, -1)
                motorControl.turnLeft(90)
                motorControl.stop()
                motorControl.straightDist(0.5, -1)
                motorControl.stop()
                motorControl.turnLeft(90)
                motorControl.stop()
            elif (self.boundaryHit.value == "w"): 
                motorControl.backward(1, -1)
                motorControl.turnRight(90)
                motorControl.stop()
                motorControl.straightDist(0.5, -1)
                motorControl.stop()
                motorControl.turnRight(90)
                motorControl.stop()
            self.boundaryHit.value = "-1"
            self.turning = False
            motorControl.straight(-2)

        print("Completing Main")


    def checkForLine(self):
        
        newDetection = True
        loop = None
        print("Vision")
        bounds = self.checkIfInBounds()
        sensors = self.vision.detectLine()
        if(bounds):
            if any(s == 1 for s in sensors):
                if(newDetection):
                    newDetection = False
                    self.motorControl.stop()
                    print("Boundary Hit")
                    self.boundaryHit.value = self.motorControl.getDirection()
                    print("Vision: " + str(self.boundaryHit.value))
                    if(loop is None or not loop.is_alive()):
                        loop = threading.Thread(target=self.mainLoop, args=(self.motorControl,))
                        loop.run()
                    time.sleep(0.2)
            else:
                newDetection = True 
    def checkForObstacle(self):
        print("Obst")
        self.obstAvoidModule.avoid_obstacle()       

    def checkIfInBounds(self):
        bounds = self.motorControl.checkIfInBounds()
        if (bounds is not None):
            print("Not in bounds")
            if bounds == "n":
                self.motorControl.emergencyStopFunc()
                self.motorControl.turnDirection("s")
                self.motorControl.straight(-1)
                time.sleep(0.5)
                self.boundaryHit.value = "-1"
            elif bounds == "e":
                self.motorControl.emergencyStopFunc()
                self.motorControl.turnDirection("w")
                self.motorControl.straight(-1)
                time.sleep(0.5)
                self.boundaryHit.value = "-1"
            elif bounds == "s":
                self.motorControl.emergencyStopFunc()
                self.motorControl.turnDirection("n")
                self.motorControl.straight(-1)
                time.sleep(0.5)
                self.boundaryHit.value = "-1"
            elif bounds == "w":
                self.motorControl.emergencyStopFunc()
                self.motorControl.turnDirection("e")
                self.motorControl.straight(-1)
                time.sleep(0.5)
                self.boundaryHit.value = "-1"
            return False        
        else:
            return True




if __name__ == "__main__":
    
    #obsAvoid = ObstacleAvoidanceApi.ObstacleAvoidance(clientId)
    print("Welcome to Lemon-Laser's Autonomous Snow Plow Robot")
    LemonLaserPlow = SnowPlowRobot()
    while True:
        continue

