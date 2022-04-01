
class VisionModule:

    def __init__(self):
        left_vSensor  = sim.getObjectHandle("/robot/LeftSensor")
        middle_vSensor = sim.getObjectHandle("/robot/MiddleSensor")
        right_vSensor = sim.getObjectHandle("/robot/RightSensor")
        #res, right_vSensor_side = sim.simxGetObjectHandle(clientId, "RightSensor_side", sim.simx_opmode_blocking)
        self.vSensors = [left_vSensor, middle_vSensor, right_vSensor]#, right_vSensor_side]
        pass
    def detectLine(self):
        sensor_values=[0,0,0]#,0]
        for i in range(0,len(self.vSensors)):
            visionHandle = sim.readVisionSensor(self.vSensors[i])
            if visionHandle != -1:
                print(visionHandle)
                if(visionHandle[0] > -1): #If the vision sensor detected new thing, update
                    if(visionHandle[1][10] < 0.4): #If it sees black (low intensity)
                        sensor_values[i] = 1
                    else:
                        sensor_values[i] = 0
                return sensor_values
        return -1
