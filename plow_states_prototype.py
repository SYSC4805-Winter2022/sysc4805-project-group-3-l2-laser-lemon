# ~~~ PLOW STATES PROTOTYPE IMPLEMENTATION/SIMULATION ~~~
#
# plow_states_prototype.py
# SYSC 4805 - Robot Snow Plow Project
# Author: Timothy Knowles
# Student Number: 101097700
# Version: February 20th, 2022

#import all the necessary libraries
import datetime
import sys
import os
from time import sleep

#generic state object class
class State(object):
  
    #upon entering state, print
    def __init__(self):
        print ("Current state is now:", str(self))

    def on_event(self, event):
        pass

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return self.__class__.__name__

#intial-state state
class InitialState(State):

    def on_event(self,event):
        print("ACTIVATING PLOW MOTOR TO OPEN PLOW")
        return PlowOpenState()
        return self
    

#plow is open state  
class PlowOpenState(State):
  
    def on_event(self, event):
        
        if (event == '01') or (event == '11'):
            print("ACTIVATING PLOW MOTOR TO CLOSE PLOW")
            return PlowClosedState()
        else:
            return PlowOpenState()
        return self

#plow is now closed state
class PlowClosedState(State):
  
    def on_event(self, event):
        if event == '10':
            print("ACTIVATING PLOW MOTOR TO OPEN PLOW")
            return PlowOpenState()
        else:
            return PlowClosedState()
        return self


#plow FSM object
class PlowFSM(object):
    def __init__(self):
        self.state = InitialState()

    def on_event(self,event):

        self.state = self.state.on_event(event)

#plow fsm prototype that takes user input for sensory values
class Prototype_Sim():
    plow = PlowFSM()
    plow.on_event('')
    scriptCtl = True
    while(scriptCtl):
        exit_control = input("Would you like to exit? (Y/N)")
        if(exit_control.upper() == "Y"):
            scriptCtl = False
        else:
            front_Sensor_Value = input("What is new front sensor value? (0 or 1)")
            back_Sensor_Value = input("What is new back sensor value? (0 or 1)")
            print("New Sensor Input: ", str(back_Sensor_Value),str(front_Sensor_Value))
            plow.on_event(str(back_Sensor_Value) + str(front_Sensor_Value))


#main, simply run prototype simulation
if __name__ == "__main__":
    Prototype_Sim()

