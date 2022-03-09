import sim
import math
import time

class WheelModule:
    def __init__(self, clientId):
        res, left_joint  = sim.simxGetObjectHandle(clientId, 'left_joint1', sim.simx_opmode_blocking)
        res, right_joint = sim.simxGetObjectHandle(clientId, 'right_joint1', sim.simx_opmode_blocking)
        self.left_joint = left_joint
        self.right_joint = right_joint
        self.clientId = clientId

    def setWheelVelocity(self, handle, velocity):
        return sim.simxSetJointTargetVelocity(self.clientId, handle, velocity, sim.simx_opmode_oneshot)

    def stop(self):
        res = self.setWheelVelocity(self.left_joint, 0)
        res = self.setWheelVelocity(self.right_joint, 0)

    def turnRight(self):
            res = self.setWheelVelocity(self.right_joint, 100*math.pi/180)
            res = self.setWheelVelocity(self.left_joint, 50*math.pi/180)
            time.sleep(2)
            self.stop()

    def turnRight180(self):
            res = self.setWheelVelocity(self.right_joint, 100*math.pi/180)
            res = self.setWheelVelocity(self.left_joint, 50*math.pi/180)
            time.sleep(4)
            self.stop()

    def turnLeft(self):
            res = self.setWheelVelocity(self.left_joint, 100*math.pi/180)
            res = self.setWheelVelocity(self.right_joint, 50*math.pi/180)
            time.sleep(2)
            self.stop()

    def straight(self, distance):
        res = self.setWheelVelocity(self.left_joint, -2)
        res = self.setWheelVelocity(self.right_joint, -2)
        time.sleep(2/distance)
        self.stop()

    def turnLeft180(self):
            res = self.setWheelVelocity(self.left_joint, 100*math.pi/180)
            res = self.setWheelVelocity(self.right_joint, 50*math.pi/180)
            time.sleep(4)
            self.stop()

    def backward(self, distance):
        res = self.setWheelVelocity(self.left_joint, 2)
        res = self.setWheelVelocity(self.right_joint, 2)
        time.sleep(2/distance)
        self.stop()
