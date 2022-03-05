#########################
# ROBOT BODY UNIT TESTS
# INITIAL: MAR 5 2022
# DENISE MAYO
#########################

import robot_body_statemach
import plow_states_prototype
import time

class BodyUnitTest():
    def __init__(self):
        uut_body = robot_body_statemach.BodyStateMachine()
        uut_plow = plow_states_prototype.PlowFSM()

        
class UT01(BodyUnitTest):
    def __init__(self):
        super().__init__()
    
    print("")