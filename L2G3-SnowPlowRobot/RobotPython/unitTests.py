#
#Author: Denise Mayo
#SYSC 4805 L2G3
#

from unittest import TestCase
import sim
from PathAlgorithm import startSimulation, SnowPlowRobot
from LineDetectionApi import VisionModule
import ObstacleAvoidanceApi as oa



#Unit Tests for Line Detection Module
class LineDetectionTest(TestCase):

    def setUp(self):
        self.clientIDTest = startSimulation()
        self.dut = VisionModule(self.clientIDTest)

    def testSensorValues(self, caseNum):
        if caseNum is 0: # no line
            TestCase.assertEqual(dut.detectLine(), [0, 0, 0])
        else: # else we found a line
            TestCase.assertEqual(dut.detectLine(), [1, 1, 1])

    def tearDown(self):
        sim.simxFinish()
        return super().tearDown()
        
class ObstacleAvoidanceApiTest(TestCase):
    def setUp(self):
        self.clientIDTest = startSimulation()
        self.motorControlTest = MovementApi.WheelModule
        
    def avoidFrontTest(self, caseNum):
        if caseNum is 0: #no obstacle
            TestCase.assertEqual(oa.checkForObstacle(self.motorControlTest, self.clientIDTest, 2.5), False)
        else: 
            TestCase.assertEqual(oa.checkForObstacle(self.motorControlTest, self.clientIDTest, 2.5), True)
        
    def avoidBackTest(self, caseNum):    
        if caseNum is 0:
            TestCase.assertEqual(oa.checkForObstacleBackwards(self.clientIDTest), False)
        else:
            TestCase.assertEqual(oa.checkForObstacleBackwards(self.clientIDTest), True)

    def tearDown(self):
        sim.simxFinish()
        return super().tearDown()


