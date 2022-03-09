"""
SYSC 4805 L2-G3
Plow Api provides functions to open and close the plow
"""

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
import time

def open(clientID):
    # print("Beginning to Open Plow!")
    inputIntegers = []
    inputFloats = []
    inputStrings = []
    inputBuffer = bytearray()
    sim.simxCallScriptFunction(clientID, 'plow_middle_panel', sim.sim_scripttype_childscript, 'onOpen',
                                inputIntegers, inputFloats, inputStrings, inputBuffer, sim.simx_opmode_blocking)
    opening = True
    while opening:
        errorFlag, signalFlag = sim.simxGetStringSignal(clientID, 'openFlag', sim.simx_opmode_blocking)
        flag = signalFlag.decode('utf-8')
        time.sleep(0.15)
        if (flag == "1"):
            opening = False
        
    print("Plow is opened!")
    return True

def close(clientID):
    inputIntegers = []
    inputFloats = []
    inputStrings = []
    inputBuffer = bytearray()
    sim.simxCallScriptFunction(clientID, 'plow_middle_panel', sim.sim_scripttype_childscript, 'onClose',
                                inputIntegers, inputFloats, inputStrings, inputBuffer, sim.simx_opmode_blocking)
    closing = True
    while closing:
        errorFlag, signalFlag = sim.simxGetStringSignal(clientID, 'closeFlag', sim.simx_opmode_blocking)
        flag = signalFlag.decode('utf-8')
        time.sleep(0.15)
        if (flag == "1"):
            closing = False
        
    print("Plow is closed!")
    return True