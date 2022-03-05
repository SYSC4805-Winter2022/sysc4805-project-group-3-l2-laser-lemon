#################
## ROBOT BODY STATES
## INITIAL: FEB 19 2022
## DENISE MAYO
#################

import base_state_class
import obstacle_avoidance_proximitysensor

class RobotState(BasicState):
    def __init__(self, name):
        super().__init__(self, name)
        self.plowObj = None # Can be used to reference plow API objects

    def addEntryAction(self, entry_action):
        super().addEntryAction(self, entry_action)

    def addExitAction(self, exit_action):
        super.addExitAction(self, exit_action)

    def addStateAction(self, state_action):
        super().addStateAction(self, state_action)

    def addNextState(self, next_state):
        super().addNextState(self, next_state)

    #to be implemented later
    def addPlow(self, plowObject):
        self.plowObj = plowObject


    