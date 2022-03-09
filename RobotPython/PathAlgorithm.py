
import PlowApi as plow
import ObstacleAvoidanceApi
import LineDetectionApi
import MovementApi

import sim
import sys


def startSimulation():
    print ('Program started')
    sim.simxFinish(-1)
    clientId = sim.simxStart('127.0.0.1',19998,True,True,5000,5)
    if clientId == -1:
        sys.exit(-1)
    
    return clientId

if __name__ == "__main__":
    clientId = startSimulation()
    #obsAvoid = ObstacleAvoidanceApi.ObstacleAvoidance(clientId)
    #vision = LineDetectionApi.VisionModule(clientId)
    motorControl = MovementApi.WheelModule(clientId)
    #plow.open(clientId)
    motorControl.straightDist(0.4)
    print("Went 0.4 m")
    #motorControl.turnLeft()
    #motorControl.straight()

    while False:
        #obsAvoid.avoid_obstacle()
        sensors = vision.detectLine()
        if any(s == 1 for s in sensors):
            motorControl.stop()
            motorControl.backward(0.6)
            motorControl.turnLeft180()







