1. Where Files are Located
--Python Files are located in "Robot Python" folder
    |--> "PathAlgorithm.py" contains the main function
-- The Snow Plow Robot model is in "Model>L2G3-SnowPlowModel.ttm"
-- Training maps with our current robot model loaded in is found in "Training Maps" folder

2. Requirments
- CoppelisSim Education 4.3
    |--> Other editions may yield unexpected beviour
    |--> The robot was built and tested in 4.3
- Python 3.9
    |--> Other versions of Python3 may yield unexpected beviour
    |--> The robot was built and tested in Python 3.9

3. Instructions
    1. Unzip the "L2G3-SnowPlowRobot.zip" file onto your local machine
    2. Open the CoppeliaSim test scene you want to test the robot in
    3. To load the Snow Plow Model, in CoppeliaSim select File>Load Model
    4. Navigate to folder you unzipped earlier in the "Loading Model" popup window. 
    5. Select Model>L2G3-SnowPlowModel.ttm
    **The robot should have the correct starting coordinates and be loaded on to the "Starting Line"
    6. Run the simulation by clicking the "Start/Resume" button in the toolbar. This will launch the Remote API Server.
    7. To lauch the Remote API Python client run the RobotPython/PathAlgorithm.py
        $ python3 RobotPython/PathAlgorithm.py