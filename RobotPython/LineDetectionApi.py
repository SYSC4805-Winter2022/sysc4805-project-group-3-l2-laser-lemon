from http import client
import sim

global vSensors
class VisionModule:

    def __init__(self, clientId):
        res, left_vSensor  = sim.simxGetObjectHandle(clientId, "LeftSensor", sim.simx_opmode_blocking)
        res, middle_vSensor = sim.simxGetObjectHandle(clientId, "MiddleSensor", sim.simx_opmode_blocking)
        res, right_vSensor = sim.simxGetObjectHandle(clientId, "RightSensor", sim.simx_opmode_blocking)
        self.vSensors = [left_vSensor, middle_vSensor, right_vSensor]
        self.clientId = clientId
    
    def detectLine(self):
        sensor_values=[0,0,0]
        for i in range(0,3):
            returnCode,detectionState,Data=sim.simxReadVisionSensor(self.clientId,self.vSensors[i],sim.simx_opmode_blocking);
            if(detectionState > -1): #If the vision sensor detected new thing, update
                if(Data[0][10] <0.3): #If it sees black (low intensity)
                    sensor_values[i] = 1
                else:
                    sensor_values[i] = 0
        return sensor_values
