"""
Author: Emma Boulay
SYSC 4805 L2G3
"""
import sim
import time

def open(clientID):
    """This function tells the Lua program to initiate openning the plow

    Args:
        clientID (int): The client id for the Remote Api Server
    """
     # This makes a call to the Lua program to open the plow
    sim.simxCallScriptFunction(clientID, 'plow_middle_panel', sim.sim_scripttype_childscript, 'onOpen',
                                [],[], [], bytearray(), sim.simx_opmode_blocking)]
    opening = True
    #Wait until plow finishes opening before continuing
    while opening:
        errorFlag, signalFlag = sim.simxGetStringSignal(clientID, 'openFlag', sim.simx_opmode_blocking)
        flag = signalFlag.decode('utf-8')
        time.sleep(0.15)
        if (flag == "1"):
            opening = False

def close(clientID):
    """This function tells the Lua program to initiate closing the plow

    Args:
        clientID (int): The client id for the Remote Api Server
    """
    # This makes a call to the Lua program to close the plow
    sim.simxCallScriptFunction(clientID, 'plow_middle_panel', sim.sim_scripttype_childscript, 'onClose',
                                [],[], [], bytearray(), sim.simx_opmode_blocking)
    closing = True
    #Wait until plow finishes closing before continuing
    while closing:
        errorFlag, signalFlag = sim.simxGetStringSignal(clientID, 'closeFlag', sim.simx_opmode_blocking)
        flag = signalFlag.decode('utf-8')
        time.sleep(0.15)
        if (flag == "1"):
            closing = False
