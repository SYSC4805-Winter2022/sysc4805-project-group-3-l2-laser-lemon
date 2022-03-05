#################
## ROBOT BODY STATE MACHINE V1 
## INITIAL: FEB 19 2022
## DENISE MAYO
#################

from body_states import RobotState
from plow_states_prototype import *


try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')

class BodyStateMachine:
   
    def __init__(self, plowAddress, frontProx, backProx, vision1, vision2, vision3):
      

        #Define States
        initial = RobotState('Initial')
        deployPlow = RobotState('Deploy Plow')
        forward = RobotState('Forward Movement') #straight movement until we detect an object or a path
        followLine = RobotState('Follow Perimeter Line')
        backward = RobotState('Backward Motion (Avoid obstacle)')
        zigzag = RobotState('Zig-Zag Movement Pattern (Clear largest area)')
        stuck = RobotState('Robot Stuck, Error state')

        #Define next states - list of all possible next states for each current states
        #at simulation start, we're in initial state
        initial.addNextState(deployPlow)
        # Immediately after sim start (t > 0), we deploy plow
        deployPlow.addNextState(forward) 
        #after plow deployment, we start moving. We can move in 3 ways from here
        forward.addNextState(backward)
        forward.addNextState(followLine)
        forward.addNextState(zigzag)
        #if we are going backwards, we want to start moving forwards before we switch pattern
        backward.addNextState(forward)
        #If we're in a movement pattern, we can switch between them 
        zigzag.addNextState(followLine)
        zigzag.addNextState(forward)
        followLine.addNextState(zigzag)
        followLine.addNextState(forward)
        #If the robot gets stuck for any reason, we are in an error state. This can happen from any state
        initial.addNextState(stuck)
        deployPlow.addNextState(stuck)
        forward.addNextState(stuck)
        zigzag.addNextState(stuck)
        followLine.addNextState(stuck)
        backward.addNextState(stuck)

        #define sensors
        visionSensors = [vision1, vision2, vision3]
        proxSensors = [backProx, frontProx]

        #define plow object
        super.addPlow(plowAddress)

        #add state object 
        self.currState = initial

        #indicates if plow deployed
        self.plowDeployed = False

    
    
    #def nextState(self):
        #TBD

    def setState(self, newState):
        self.currState = newState

    def getState(self):
        return self.currState

        

    

def main():

    #initialize body, plow, and sim
    body = BodyStateMachine()
    clientID = 0

    #plow = State()

    #associate scripts as objects

    #start simulation
    #clientID = sim.simxStart('127.0.0.1',19999,True,True,5000,5)
   

    return 0; 



      

