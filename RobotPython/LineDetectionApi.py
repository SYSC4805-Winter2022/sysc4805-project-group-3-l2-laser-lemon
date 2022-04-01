import sim
class VisionModule:

    def __init__(self, clientId):
        self.clientId = clientId
        res, left_vSensor  = sim.simxGetObjectHandle(self.clientId, "/robot/LeftSensor",sim.simx_opmode_blocking)
        res, middle_vSensor = sim.simxGetObjectHandle(self.clientId, "/robot/MiddleSensor",sim.simx_opmode_blocking)
        res, right_vSensor = sim.simxGetObjectHandle(self.clientId, "/robot/RightSensor",sim.simx_opmode_blocking)
        #res, right_vSensor_side = sim.simxGetObjectHandle(clientId, "RightSensor_side", sim.simx_opmode_blocking)
        self.vSensors = [left_vSensor, middle_vSensor, right_vSensor]#, right_vSensor_side]
        
    
    def detectLine(self):
        sensor_values=[0,0,0]#,0]
        for i in range(0,len(self.vSensors)):
            returnCode, detectionState,Data=sim.simxReadVisionSensor(self.clientId, self.vSensors[i],sim.simx_opmode_blocking);
            if(detectionState > -1): #If the vision sensor detected new thing, update
                
                if(Data[0][10] < 0.4): #If it sees black (low intensity)
                    sensor_values[i] = 1
                else:
                    sensor_values[i] = 0
        return sensor_values
