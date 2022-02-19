#################
## ROBOT BODY STATE MACHINE V1 
## INITIAL: FEB 19 2022
## DENISE MAYO
#################

from body_states import RobotState
class BodyStateMachine:
   
    def __init__(self):
      

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



      

