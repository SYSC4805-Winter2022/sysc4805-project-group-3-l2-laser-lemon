
import PlowApi as plow
import ObstacleAvoidanceApi
import LineDetectionApi
import MovementApi
import time

import sim
import sys

flagEastWest = not True

def startSimulation():
    print ('Program started')
    sim.simxFinish(-1)
    clientId = sim.simxStart('127.0.0.1',19998,True,True,5000,1)
    if clientId == -1:
        print("Connection Unsuccesful")
        sys.exit(-1)
    
    return clientId

if __name__ == "__main__":
    clientId = startSimulation()
    #obsAvoid = ObstacleAvoidanceApi.ObstacleAvoidance(clientId)
    vision = LineDetectionApi.VisionModule(clientId)
    motorControl = MovementApi.WheelModule(clientId)
    plow.open(clientId)
    #motorControl.straightDist(1, 1)
    #motorControl.turnRight(90)
    #motorControl.stop()
    motorControl.straight(2)


    while True:
        #obsAvoid.avoid_obstacle()
        sensors = vision.detectLine()
        #motorControl.stayOnPath()
        
        if any(s == 1 for s in sensors[0:2]):
            motorControl.stop()
            if sensors[3] ==1:
                flagEastWest = flagEastWest

                print("Stopping")
                print("Backwards")
            if flagEastWest:
                motorControl.backward(1.2, 0.5)
                if motorControl.getDirection() == "e":
                    motorControl.turnLeft(90)
                    motorControl.straightDist(0.05, 1)
                    motorControl.turnLeft(90)
                    motorControl.stop()
                    
                    motorControl.stop()
                elif motorControl.getDirection() == "w":
                    motorControl.turnRight(90)
                    motorControl.straightDist(0.05, 1)
                    motorControl.turnRight(90)
                    motorControl.stop()
                motorControl.straight(2)
            else:
                print("Stopping")
                print("Backwards")
                motorControl.backward(1.2, 0.5)
                if motorControl.getDirection() == "n":
                    motorControl.turnLeft(90)
                    motorControl.straightDist(0.05, 1)
                    motorControl.turnLeft(90)
                    motorControl.stop()
                    
                    motorControl.stop()
                elif motorControl.getDirection() == "s":
                    motorControl.turnRight(90)
                    motorControl.straightDist(0.05, 1)
                    motorControl.turnRight(90)
                    motorControl.stop()
                motorControl.straight(2)







