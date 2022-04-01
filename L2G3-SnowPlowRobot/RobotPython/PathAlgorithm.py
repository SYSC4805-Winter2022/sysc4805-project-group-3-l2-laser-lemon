"""
Author: Emma Boulay
SYSC 4805 L2G3

The SnowPlow Robot class is responsible for controlling all the 
robot modules (Vision, Obstacle Avoidance, Plow, Motors) and implementing
the pathing algorithm.
"""
#Robot Module Imports
import PlowApi as plow, ObstacleAvoidanceApi, LineDetectionApi, MovementApi

# System Library Imports
import sim, sys
from random import randrange

def startSimulation():
    """Function to start to the remote api client

    Returns:
        int: clientId if successful connection, -1 otherwise
    """
    print ('Program started')
    sim.simxFinish(-1)
    clientId = sim.simxStart('127.0.0.1',19998,True,True,5000,1)
    if clientId == -1:
        print("Connection Unsuccesful")
        sys.exit(-1)
    return clientId


class SnowPlowRobot:
    """The SnowPlow Robot class. It is responsible for controlling all the 
    robot modules (Vision, Obstacle Avoidance, Plow, Motors) and implementing
    the pathing algorithm.
    """
    def __init__(self):
        """This initializes all the robot modules. 
        The robot will leave the starting area by driving forward 
        """
        #Connect to Remote Api Server
        self.clientId = startSimulation()
        #Initialize robot modules
        self.vision = LineDetectionApi.VisionModule(self.clientId)
        self.motorControl = MovementApi.WheelModule(self.clientId)
        #Open Plow
        plow.open(self.clientId)
        #Drive straight 1m, turn right and continue until obstacle or line
        self.motorControl.straightDist(1, -2)
        self.motorControl.turnRight(90)
        self.motorControl.stop()
        self.motorControl.straight(-2)
        self.turningLeft = 1
        
        #Start the main robot loop sequence
        while True:
            self.pathFindingAlgorithm()

    def pathFindingAlgorithm(self):
        """
        This is the main path finding algorithm for the robot. The robot will avoid obstalces and
        if it detects line it will turn in a new direction. Moving in a pinp-pong pattern.
        """
        try:
            self.checkForLine()
            obs = ObstacleAvoidanceApi.checkForObstacle(self.motorControl, self.clientId, [2/self.motorControl.wheelRadius, 2/self.motorControl.wheelRadius])

            #Check if object is detect in plow area
            plowArea = ObstacleAvoidanceApi.checkForObstaclePlow(self.clientId)
            if(plowArea):
                self.motorControl.backward(0.3, -2)
                if(self.turningLeft):
                    #Turn left a random degree from 150-230
                    self.motorControl.turnLeft(90)
                else:
                    #Turn right a random degree from 150-230
                    self.motorControl.turnRight(90)
                self.turningLeft = (self.turningLeft  + 1) % 2
        except:
                sim.simxFinish(self.clientId) 
                print("Connection Ended")
                sys.exit(1)
    def checkForLine(self):
        """This function checks if a line is detected. 
        If a line is detected the robot will drive forward a bit more to 
        ensure all snow is cleared from arena. Then it will drive backwards and 
        turn a random amount and begin driving forward again.
        """
        #Poll sensors
        sensors = self.vision.detectLine()
        #If any sensor detects a line launch turning sequence
        if any(s == 1 for s in sensors):
            self.motorControl.straightDist(0.2, -2)
            self.motorControl.stop()
            self.motorControl.backward(0.6,-2)
            #This ensures that the robot alternates if it turns left or right
            #after detecting a line
            if(self.turningLeft):
                #Turn left a random degree from 150-230
                self.motorControl.turnLeft(randrange(150, 230))
            else:
                #Turn right a random degree from 150-230
                self.motorControl.turnRight(randrange(150, 230))
            self.motorControl.stop()
            self.turningLeft = (self.turningLeft  + 1) % 2
            #Continue driving straight
            self.motorControl.straight(-2)  

#Launches the Python Robot Bindings
if __name__ == "__main__":
    print("Welcome to Lemon-Laser's Autonomous Snow Plow Robot")
    LemonLaserPlow = SnowPlowRobot()