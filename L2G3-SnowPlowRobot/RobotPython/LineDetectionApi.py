"""
Author: Emma Boulay
SYSC 4805 L2G3

The VisionModule class is responsible for communicating with the vision sensors.
It provides functionality to check which vision sensors detect a line.

"""

import sim
class VisionModule:
    """
    This class is responsible for communicating with the vision sensors.
    """
    def __init__(self, clientId):
        """This instatistiates the Vision Module.
        All of the vision sensors on the robot

        Args:
            clientId (int): The client ID for the Remote API server
        """
        self.clientId = clientId
        #Instatiate all the vision sensors
        res, left_vSensor  = sim.simxGetObjectHandle(self.clientId, "/robot/LeftSensor",sim.simx_opmode_blocking)
        res, middle_vSensor = sim.simxGetObjectHandle(self.clientId, "/robot/MiddleSensor",sim.simx_opmode_blocking)
        res, right_vSensor = sim.simxGetObjectHandle(self.clientId, "/robot/RightSensor",sim.simx_opmode_blocking)
        self.vSensors = [left_vSensor, middle_vSensor, right_vSensor]#, right_vSensor_side]
        
    def detectLine(self):
        """This function detects if a line has been crossed.

        Returns:
            sensor_values (list): A list of all sensor_values. A sensor is 1 if line detected, 0 otherwise.
        """
        sensor_values=[0,0,0]
        #Loop over each sensor and check if line detected
        for i in range(0,len(self.vSensors)):
            returnCode, detectionState,Data=sim.simxReadVisionSensor(self.clientId, self.vSensors[i],sim.simx_opmode_blocking);
            if(detectionState > -1): #If the vision sensor detected new thing, update
                
                if(Data[0][10] < 0.4): #If it sees black (low intensity)
                    sensor_values[i] = 1
                else:
                    sensor_values[i] = 0
        return sensor_values
