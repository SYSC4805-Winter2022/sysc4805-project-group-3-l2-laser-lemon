
import PlowApi as plow
import ObstacleAvoidanceApi
import LineDetectionApi
import MovementApi
import time

import sim
import sys


def startSimulation():
    print ('Program started')
    sim.simxFinish(-1)
    clientId = sim.simxStart('127.0.0.1',19997,True,True,5000,5)
    if clientId == -1:
        print("Connection Unsuccesful")
        sys.exit(-1)
    
    return clientId

if __name__ == "__main__":
    clientId = startSimulation()
    obsAvoid = ObstacleAvoidanceApi.ObstacleAvoidance(clientId)
    vision = LineDetectionApi.VisionModule(clientId)
    motorControl = MovementApi.WheelModule(clientId)
    #plow.open(clientId)
    while True:
        obsAvoid.avoid_obstacle()
        sensors = vision.detectLine()
        if any(s == 1 for s in sensors):
            motorControl.stop()
            time.sleep(0.5)
            motorControl.backward(2, 2)
            time.sleep(0.5)
            motorControl.turnLeft(90)
            time.sleep(0.5)
            motorControl.straightDist(0.1, 2)
            time.sleep(0.5)
            motorControl.turnLeft(90)







