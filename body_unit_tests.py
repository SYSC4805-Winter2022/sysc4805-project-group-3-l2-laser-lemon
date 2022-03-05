#########################
# ROBOT BODY UNIT TESTS
# INITIAL: MAR 5 2022
# DENISE MAYO
#########################
# NOTES: 
    # There is no integration with other modules functionality at initial coding

import robot_body_statemach
import time
import unittest

class BodyUnitTest(unittest.TestCase):
    def __init__(self):
        super.__init__()
        
    

class UT01(BodyUnitTest): # purpose: Test if robot can change states correctly
    def __init__(self):
        super().__init__()
        #uut_body = robot_body_statemach.BodyStateMachine()
        
    
    def testStateChanges(self):
        # State Change Criteria:
        # Initial -> Deploy Plow - on sim start
        uut_body = robot_body_statemach.BodyStateMachine(plowAddress=1, frontProx=0, backProx=0, vision1=0, vision2=0, vision3=0)
        self.assertEqual(uut_body.getState(), uut_body.initial)
        
        # @sim start, deploy plow
        uut_body.setState(uut_body.deployPlow)
        self.assertEqual(uut_body.getState(), uut_body.deployPlow)

        # Deploy Plow ->  forward - once plow is successfully deployed -- requires return value/state change
        simStart = False
        

        # Any -> Stuck
        # Forward -> ZigZag - ??
        # Forward -> Follow Line - ??
        # Forward -> Backward - front proximity sensor gives a 1 
        # TBD !! ZigZag or Follow Line -> Forward - front proximity gives 1 OR vision sensors detect line
        # Backward -> Forward - return from obstacle avoidance 
        

        

    