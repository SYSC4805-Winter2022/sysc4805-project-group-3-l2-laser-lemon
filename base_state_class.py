#################
## BASIC ROBOT OR PLOW STATES
## INITIAL: FEB 19 2022
## DENISE MAYO
#################

class BasicState:
    def __init__(self, name):
        self.name = name
        self.entry_actions = []
        self.exit_actions = []
        self.state_actions = []
        self.next_states = []

    def addEntryAction(self, entry_action):
        self.entry_actions += entry_action

    def addExitAction(self, exit_action):
        self.exit_actions += exit_action

    def addStateAction(self, state_action):
        self.state_actions += state_action

    def addNextState(self, next_state):
        self.next_states += next_state