#################
## ROBOT BODY STATES
## INITIAL: FEB 19 2022
## DENISE MAYO
#################

import base_state_class

class RobotState(BasicState):
    def __init__(self, name, sensors):
        super().__init__(self, name)
        self.sensors = sensors

    def addEntryAction(self, entry_action):
        super().addEntryAction(self, entry_action)

    def addExitAction(self, exit_action):
        super.addExitAction(self, exit_action)

    def addStateAction(self, state_action):
        super().addStateAction(self, state_action)

    def addNextState(self, next_state):
        super().addNextState(self, next_state)

    