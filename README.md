# SYSC 4805 Project group 3-L2 Team Laser Lemon


sysc4805-project-group-3-l2-laser-lemon created by GitHub Classroom

The overall objective is to design an autonomous snow plough robot using CoppeliaSim that will clear the snow off an area enclosed by a closed path while avoiding fixed and moving obstacles. 

-------------------------------------------------------------------------------------------------------------
# Report
* The final report is locacted in the "Reports" Folder

-------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------
# Requirements:
1. CoppelisSim Education 4.3
    |--> Other editions may yield unexpected beviour
    |--> The robot was built and tested in 4.3
2. Python 3.9
    |--> Other versions of Python3 may yield unexpected beviour
    |--> The robot was built and tested in Python 3.9
-------------------------------------------------------------------------------------------------------------

-------------------------------------------------------------------------------------------------------------
# How to set up environment
1. Dowload CoppeliaSim Education 4.3 from https://www.coppeliarobotics.com/downloads
2. Python 3.9 can be downloaded from https://www.python.org/downloads/release/python-3912/
2. Unzip the "L2G3-SnowPlowRobot.zip" file onto your local machine
3. Open the CoppeliaSim test scene you want to test the robot in
4. To load the Snow Plow Model, in CoppeliaSim select File > Load Model
5. Navigate to folder you unzipped earlier in the "Loading Model" popup window.
6. Select Model > L2G3-SnowPlowModel.ttm
    **The robot should have the correct starting coordinates and be loaded on to the "Starting Line"
7. Run the simulation by clicking the "Start/Resume" button in the toolbar. This will launch the Remote API Server
8. To lauch the Remote API Python client run the RobotPython/PathAlgorithm.py
        $ python3 RobotPython/PathAlgorithm.py
-------------------------------------------------------------------------------------------------------------

-------------------------------------------------------------------------------------------------------------
# File Directory:
* Python Files are located in "L2G3-SnowPlowRobot/Robot Python" folder
    * "PathAlgorithm.py" contains the main function
* The Snow Plow Robot model is in "L2G3-SnowPlowRobot > Model > L2G3-SnowPlowModel.ttm"
* Training maps with our current robot model loaded in is found in "Training Maps" folder
-------------------------------------------------------------------------------------------------------------

-------------------------------------------------------------------------------------------------------------
# Simulation Speed:
The simulation is best runned with
* Physics Engine: Bullet 2.78
* Dynamic Settings: balanced
* Simulation time step: 50 ms
    
-------------------------------------------------------------------------------------------------------------
