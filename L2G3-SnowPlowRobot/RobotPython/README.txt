These are the Python Scripts used to control the robot through the Remote API client

File hierarchy:

PathAlgorithm
    |--> LineDetectionApi
    |--> MovementApi
    |--> ObstacleAvoidanceApi
    |--> PlowApi

Files:
    PathAlgorithm: Responsible for main pathing algorithm. 
                   The robot will turn a random degree when a line detected
    
    LineDetectionApi: Responsible for communicating with vision sensors

    MovementApi: Provides functionality for conrolling the wheels to drive the robot

    ObstacleAvoidanceApi: Responsible for detecting and avoiding objects

    PlowApi: Provides functionality to open and close the plow.